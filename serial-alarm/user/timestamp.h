/* timestamp.h — 用户态纳秒时间戳 (方案 D4)
 *
 * 默认: vDSO clock_gettime(CLOCK_MONOTONIC_RAW), 无系统调用, ~20-30 ns,
 *       且与内核侧 INJECT_TS (ktime_get_ns) 同为 monotonic 域,
 *       t1 - t0 差值不受时钟域/偏移影响。
 *
 * 可选: 定义 USE_CNTPCT 后改用 ARMv8 mrs cntpct_el0 直读架构计数器
 *       (~几 ns)。注意: Linux arm64 默认只开放 EL0VCTEN (虚拟计数器
 *       CNTVCT), 物理计数器 CNTPCT 的 EL0 访问默认关闭, 未开放时
 *       执行 mrs cntpct_el0 会收到 SIGILL (本项目在华为鲲鹏 HCE
 *       内核上实测如此)。真实目标板若确认 CNTKCTL_EL1.EL0PCTEN=1
 *       可编译加 -DUSE_CNTPCT。
 */
#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>
#include <time.h>

#if defined(__aarch64__) && defined(USE_CNTPCT)

static inline uint64_t ts_now_ns(void)
{
	uint64_t t, f;
	__asm__ volatile("mrs %0, cntpct_el0" : "=r"(t));
	__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
	return (uint64_t)((__uint128_t)t * 1000000000ull / f);
}

#else /* 默认: vDSO, x86/arm64 通用, 无 syscall */

static inline uint64_t ts_now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

#endif

#endif /* TIMESTAMP_H */
