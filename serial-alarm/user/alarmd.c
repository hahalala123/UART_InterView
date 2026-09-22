// SPDX-License-Identifier: MIT
/*
 * alarmd.c — 用户态驱动: 警报实时处理守护进程 (方案 5.5)
 *
 * 初始化序列 (只执行一次, 不进热路径):
 *   1) mlockall(MCL_CURRENT | MCL_FUTURE)      — 禁热路径页错误
 *   2) SCHED_FIFO 99 + 绑定 CPU1               — 独占核 (方案 D3)
 *   3) mmap /dev/valarm0、/dev/valarm1          — MMIO 窗口
 *   4) 预热调用 foo() + LD_BIND_NOW            — 消除 PLT 懒绑定/缺页
 *   5) 预分配样本缓冲
 *
 * 热路径 (零系统调用、零调度、零中断):
 *   for (;;) {
 *       if (lsr(uart0) & DR) { byte = rbr(uart0); ts = inject_ts; t1 = now;
 *                              ack; foo(0, byte, ts); 记录 (t1-ts); }
 *       if (lsr(uart1) & DR) { ... }
 *   }
 *
 * 测量 (方案 7.1 通道二): 模拟硬件的 INJECT_TS 即"字节就绪"真值 t0,
 * t1 = foo() 入口时刻; 每次警报记录 (t1 - t0), SIGUSR1/SIGINT 时输出
 * avg / p99 / max / 丢字节 / 错序, 并写 CSV。
 *
 * 用法:
 *   alarmd [-c cpu] [-p fifo_pri] [-o out.csv] /dev/valarm0 /dev/valarm1
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "uart_mmio.h"
#include "timestamp.h"
#include "alarm.h"

#define NPORTS       2
#define MAX_SAMPLES  (1u << 20)   /* 预分配, mlockall 后无页错误 */

struct sample {
	uint64_t lat;     /* t1 - t0 (ns), 两侧均为 CNTVCT 域 */
	uint64_t t0;
	uint8_t  port;
	uint8_t  byte;
};

static volatile sig_atomic_t g_stop;
static struct sample *g_samples;
static volatile uint32_t g_n;
static uart_mmio_t g_uarts[NPORTS];
static uint8_t g_expected[NPORTS];
static int g_have_expected[NPORTS];
static uint64_t g_seq_err, g_overflow;

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static int cmp_u64(const void *a, const void *b)
{
	uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
	return (x > y) - (x < y);
}

/*
 * 时钟域说明: INJECT_TS 由内核模块读 CNTVCT_EL1 锁存, 与 vDSO
 * CLOCK_MONOTONIC_RAW (cntvct_el0) 同源, t1 - t0 无跨域偏移。
 */
static void report_and_save(const char *csv)
{
	uint32_t n = g_n;
	double sum = 0;
	uint64_t *lats;
	uint32_t i;

	if (n == 0) {
		printf("no samples\n");
		return;
	}
	lats = malloc(n * sizeof(*lats));
	if (!lats)
		return;
	for (i = 0; i < n; i++)
		lats[i] = g_samples[i].lat;
	qsort(lats, n, sizeof(*lats), cmp_u64);
	for (i = 0; i < n; i++)
		sum += (double)lats[i];

	printf("==== latency report ====\n");
	printf("samples      : %u\n", n);
	printf("avg          : %.1f ns\n", sum / n);
	printf("p50          : %llu ns\n", (unsigned long long)lats[n / 2]);
	printf("p99          : %llu ns\n",
	       (unsigned long long)lats[(uint64_t)n * 99 / 100]);
	printf("max          : %llu ns\n", (unsigned long long)lats[n - 1]);
	printf("seq_errors   : %llu\n", (unsigned long long)g_seq_err);
	printf("ring_overflow: %llu\n", (unsigned long long)g_overflow);

	if (csv) {
		FILE *f = fopen(csv, "w");
		if (f) {
			uint32_t j;
			fprintf(f, "idx,port,byte,t0_ns,latency_ns\n");
			for (j = 0; j < n; j++)
				fprintf(f, "%u,%u,%u,%llu,%llu\n", j,
					g_samples[j].port, g_samples[j].byte,
					(unsigned long long)g_samples[j].t0,
					(unsigned long long)g_samples[j].lat);
			fclose(f);
			printf("csv written  : %s\n", csv);
		} else {
			perror("fopen csv");
		}
	}
	free(lats);
}

int main(int argc, char **argv)
{
	int cpu = 1, prio = 99;
	const char *csv = "alarm_latency.csv";
	const char *dev[NPORTS] = { "/dev/valarm0", "/dev/valarm1" };
	int opt, i, fd[NPORTS];
	struct sched_param sp;
	cpu_set_t set;
	struct sigaction sa = { .sa_handler = on_signal };

	while ((opt = getopt(argc, argv, "c:p:o:")) != -1) {
		switch (opt) {
		case 'c': cpu  = atoi(optarg); break;
		case 'p': prio = atoi(optarg); break;
		case 'o': csv  = optarg; break;
		default:
			fprintf(stderr, "usage: %s [-c cpu] [-p prio] [-o csv] "
				"[dev0 dev1]\n", argv[0]);
			return 2;
		}
	}
	if (optind + NPORTS <= argc) {
		dev[0] = argv[optind];
		dev[1] = argv[optind + 1];
	}

	/* 1) 预分配并锁定全部内存 */
	g_samples = malloc(MAX_SAMPLES * sizeof(*g_samples));
	if (!g_samples) {
		perror("malloc");
		return 1;
	}
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
		perror("mlockall (ignored)");

	/* 2) 实时属性: FIFO 优先级 + 独占核; 无权限时降级并告警 */
	if (sched_getparam(0, &sp) == 0) {
		sp.sched_priority = prio;
		if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
			fprintf(stderr, "warn: SCHED_FIFO %d failed (%s), "
				"running without RT priority\n",
				prio, strerror(errno));
	}
	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	if (sched_setaffinity(0, sizeof(set), &set) != 0)
		fprintf(stderr, "warn: bind cpu%d failed (%s)\n",
			cpu, strerror(errno));

	/* 3) 映射两个虚拟 UART
	 *
	 * mmap 是"零系统调用热路径"的前提: 这一步之后, g_uarts[i].base
	 * 是指向那页"寄存器内存"的普通用户态指针, 循环里对它的读写
	 * 全部是普通内存访问, 不再经过内核。
	 * O_SYNC 对真实设备有意义 (绕过页缓存); 对本模拟无影响。
	 */
	for (i = 0; i < NPORTS; i++) {
		void *p;
		fd[i] = open(dev[i], O_RDWR | O_SYNC);
		if (fd[i] < 0) {
			fprintf(stderr, "open %s: %s\n", dev[i], strerror(errno));
			return 1;
		}
		p = mmap(NULL, UART_MAP_SIZE, PROT_READ | PROT_WRITE,
			 MAP_SHARED, fd[i], 0);
		if (p == MAP_FAILED) {
			fprintf(stderr, "mmap %s: %s\n", dev[i], strerror(errno));
			return 1;
		}
		g_uarts[i].base = p;
	}

	/* 4) 预热: 消除 PLT 懒绑定与代码页缺页
	 *
	 * 动态库函数第一次调用时才解析地址 (PLT 懒绑定), 调用时还可能
	 * 触发缺页中断, 都是几十~几百微秒级的意外开销。提前调用一次
	 * foo(), 把这些一次性成本全部在初始化阶段付掉, 热路径上就是
	 * 纯粹的寄存器读写。
	 */
	foo(0, 0, 0);
	foo(1, 0, 0);

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGUSR1, &sa, NULL);
	printf("alarmd ready: polling %s %s on cpu%d (SIGUSR1/SIGINT to "
	       "report)\n", dev[0], dev[1], cpu);
	fflush(stdout);

	/* 热路径: 永不睡眠
	 *
	 * 这是本方案的灵魂, 对照传统做法就明白为什么快:
	 *   传统: 硬件中断 -> 内核唤醒线程 -> 调度器挑中 -> 恢复上下文
	 *         (每次事件都要进内核, 延迟 3~10 us 起步)
	 *   本方案: 一个死循环不停看标志位, 看到了就立刻处理
	 *         (不进内核、不被调度, 延迟 = 最多一个循环周期, ~百 ns)
	 *
	 * 循环体故意写得极小: 读状态位 -> 有数据则读数据/时间戳/清标志
	 * -> 调用处理函数。volatile 保证编译器不会把"反复读同一地址"
	 * 优化成只读一次。
	 */
	while (!g_stop) {
		for (i = 0; i < NPORTS; i++) {
			if (uart_lsr(&g_uarts[i]) & UART_LSR_DR) {
				uint8_t  b  = uart_rbr(&g_uarts[i]);      /* 取数据 */
				uint64_t t0 = uart_inject_ts(&g_uarts[i]); /* 报警时刻 */
				uint64_t t1 = ts_now_ns();                 /* 现在时刻 */
				uart_ack(&g_uarts[i]);                     /* 清标志 */

				foo(i, b, t0);   /* 需求方要求的用户态处理入口 */

				/* 序列检查: 字节应严格 +1 递增 (方案 7.3)。
				 * 模拟报警器按 0,1,2... 发号, 一旦出现不连号
				 * 就说明期间丢了字节 (云服务器调度 stall 时,
				 * 多个字节会在寄存器里互相覆盖) */
				if (!g_have_expected[i]) {
					g_expected[i] = b;
					g_have_expected[i] = 1;
				} else if (b != (uint8_t)(g_expected[i] + 1)) {
					g_seq_err++;
				}
				g_expected[i] = b;

				/* 记录本次延迟样本, 供报告统计 */
				if (g_n < MAX_SAMPLES) {
					g_samples[g_n].lat  = t1 - t0;
					g_samples[g_n].t0   = t0;
					g_samples[g_n].port = (uint8_t)i;
					g_samples[g_n].byte = b;
					g_n++;
				} else {
					g_overflow++;
				}
			}
		}
	}

	report_and_save(csv);
	return 0;
}
