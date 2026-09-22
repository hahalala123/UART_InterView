// SPDX-License-Identifier: GPL-2.0
/*
 * valarm.c — 虚拟 UART + 软件模拟报警器 (内核驱动)
 *
 * 【这份代码在干什么?】
 * 技术方案要求: 串口收到 1 字节报警数据后, 用户态要在 < 1 us 内调用
 * 处理函数。方案选择的路线是"用户态程序直接读串口寄存器"(忙轮询),
 * 不走内核中断。为了在没有真实硬件的服务器上验证这条路线的延迟,
 * 本模块用软件扮演两块硬件:
 *   1) 虚拟 UART  —— 一组内存模拟的寄存器 (收数据字节 + 状态位)
 *   2) 模拟报警器 —— 你往 sysfs 文件写个数, 它就"发出一次警报"
 *
 * 用软件模拟硬件 (内核驱动)
 *   - 两个 8250 布局兼容的虚拟 UART (40 Mbaud 语义: 单字节即一次 DR 事件)
 *   - 报警器 = sysfs 单次注入 + hrtimer 突发注入, 注入时:
 *       1) 写入 RBR 与硬件时间戳 INJECT_TS (模拟警报器 GPIO 标记 / 示波器 t0)
 *       2) 置位 LSR.DR, 用户态轮询即可见
 *       3) 内核慢路径记账 (方案 D5, 不进热路径)
 *
 * 【内核模块是什么?】
 * Linux 内核不允许普通程序直接读写硬件/任意物理内存。内核模块是一段
 * 可以动态加载进内核的代码 (insmod 加载, rmmod 卸载), 运行在内核态,
 * 有权限做这些事。本模块加载后, 创建两个设备文件 /dev/valarm0/1,
 * 用户态程序 open + mmap 之后就能直接读写那页"寄存器内存"了。
 *
 * 寄存器布局 (与用户态 user/uart_mmio.h 严格一致, 偏移单位 byte):
 *   0x00 RBR        [RO] 报警字节
 *   0x04            保留
 *   0x05 LSR        混合: bit0 = DR; 用户态取数后写 0 清除 DR
 *                        (真实硬件读 RBR 自清 DR; 模拟硬件由用户态
 *                        显式清, 内核注入侧为 |= DR 置位, 二者不冲突)
 *   0x08 INJECT_TS  [RO] u64, 注入时刻 (ns, CLOCK_MONOTONIC_RAW 域,
 *                        与用户态 vDSO 同源)
 *   0x10 IRQ_CNT    [RO] u32, 内核慢路径计数 (诊断)
 *
 * 一致性说明: 用户态经 vm_insert_page 映射为普通缓存映射, 与内核
 * 侧 page_address 别名同为 WB 属性且访问同一物理地址; arm64 为
 * PIPT cache, 不存在别名不一致问题。用 vm_insert_page 而非
 * remap_pfn_range: 后者映射普通 RAM 页会被部分加固内核拒绝 (EINVAL),
 * 且 vm_insert_page 自动处理页引用计数。若移植到 VIPT 平台需重新
 * 评估缓存属性。注意 mmap 的 vma 按页取整, 长度检查须与 PAGE_SIZE
 * 比较而非寄存器窗口大小。
 *
 * 接口:
 *   /dev/valarm0, /dev/valarm1        — mmap (寄存器窗口) + read (统计)
 *   /sys/.../valarmN/inject  (WO)     — 写 0-255, 单次注入该字节
 *   /sys/.../valarmN/burst   (WO)     — 写 "<count> <interval_ns>", 突发注入
 *   /sys/.../valarmN/stats   (RO)     — injected/acked/irq/loss 计数
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/string.h>

#define VALARM_NDEV  2
#define VALARM_REGSZ 0x100

#define REG_RBR    0x00
#define REG_ACK    0x04
#define REG_LSR    0x05
#define REG_TS     0x08
#define REG_IRQCNT 0x10

#define LSR_DR 0x01

/*
 * struct valarm —— 一个虚拟串口的全部状态。
 * 真实驱动里这里会放硬件资源 (寄存器基地址、中断号等),
 * 我们放的是它们的软件替身:
 *   page/regs  —— 用 alloc_page 申请的一页物理内存, 充当"寄存器块"。
 *                  内核通过 regs 指针读写它; 同一页还会被映射到用户
 *                  态 (见 valarm_mmap), 两边看到同一份数据
 *   timer      —— 内核高精度定时器, 扮演"警报器"来发突发数据
 *   burst_*    —— 突发注入的参数: 共发几个、已发几个、间隔多少纳秒
 *   next_byte  —— 发送字节按 0,1,2... 递增, 用户态检查序号是否连号
 *                  就能知道"有没有丢字节、有没有错序" (验收标准之一)
 *   injected   —— 已注入总数 (原子变量: 可能被中断上下文并发访问)
 *   irq_count  —— 慢路径诊断计数 (见 valarm_inject 注释)
 */
struct valarm {
	struct miscdevice misc;
	struct page *page;
	u8 *regs;                 /* 内核缓存别名 */
	struct hrtimer timer;
	u32 burst_total;
	u32 burst_sent;
	u64 burst_interval_ns;
	u8  next_byte;            /* 注入字节按序号递增, 供用户态查错序/丢字节 */
	atomic_t injected;
	atomic_t irq_count;       /* 慢路径诊断计数 (方案 D5) */
};

static struct valarm g_dev[VALARM_NDEV];

/*
 * 时间戳统一到 CLOCK_MONOTONIC_RAW 域:
 * ktime_get_raw_ns() 是内核侧 MONOTONIC_RAW (timekeeping raw 基准),
 * 用户态 vDSO clock_gettime(CLOCK_MONOTONIC_RAW) 与之同源同速。
 * 不能用 ktime_get_ns(): 那是 MONOTONIC 域, 含 NTP/steering 调整,
 * 与 RAW 域之差非恒定 (鲲鹏 ECS 实测基线偏移 ~0.84 s 且持续漂移)。
 * (注: 曾考虑内核 mrs 读 cntvct_el1, 但 HCE 工具链汇编器不认该
 * 系统寄存器名, 放弃。)
 */
static u64 valarm_now_ns(void)
{
	return ktime_get_raw_ns();
}

/* ---- 注入: 模拟"警报器发出 1 字节" ----
 *
 * 这是整个模拟框架的核心动作, 对应真实世界里的:
 *   警报器拉低 TX 线 -> UART 逐位采样 -> 字节进入 RBR 寄存器 -> LSR.DR 置 1
 * 在这里只是三次内存写。注意写入顺序与内存屏障:
 *   必须先写好"数据"和"时间戳", 最后才置 DR 位。
 *   否则用户态可能先看到 DR=1, 读到的却是还没来得及写的旧数据。
 *   smp_wmb() 是"写内存屏障", 保证它之前的写操作先对其它 CPU 可见。
 */
static void valarm_inject(struct valarm *v, u8 byte)
{
	u64 ts = valarm_now_ns();

	*(u64 *)(v->regs + REG_TS) = ts;   /* ① 时间戳: "警报发生的时刻" */
	v->regs[REG_RBR] = byte;           /* ② 数据字节 */
	smp_wmb();                         /* ③ 屏障: ①②先于④对外可见 */
	v->regs[REG_LSR] |= LSR_DR;        /* ④ 置"数据就绪"标志位 */

	atomic_inc(&v->injected);
	/*
	 * 内核慢路径诊断记账 (方案 D5): 真实系统中注入会触发串口 IRQ、
	 * 由内核中断处理程序统计; 本模拟框架不注册真实中断 (华为 HCE
	 * 内核魔改了 irq_alloc_descs 宏, 标准申请方式不可用; 且虚拟
	 * 中断的 desc->owner 机制会导致模块 rmmod 死锁), 改为在注入
	 * 路径直接累加同一计数器, 慢路径的可观测语义保持不变,
	 * 且该记账不处于用户态热路径, 不影响延迟指标。
	 */
	*(u32 *)(v->regs + REG_IRQCNT) = (u32)atomic_inc_return(&v->irq_count);
}

/* ---- 突发注入器: hrtimer 模拟警报器按间隔连发 ----
 *
 * hrtimer 是内核的高精度定时器 (可到纳秒级)。回调函数返回
 * HRTIMER_RESTART 并把定时器推向下一个时刻 = "连发模式";
 * 发够数量后返回 HRTIMER_NORESTART = 停止。
 */
static enum hrtimer_restart valarm_burst_fn(struct hrtimer *t)
{
	struct valarm *v = container_of(t, struct valarm, timer);

	valarm_inject(v, v->next_byte++);

	if (++v->burst_sent < v->burst_total) {
		hrtimer_forward_now(t, ns_to_ktime(v->burst_interval_ns));
		return HRTIMER_RESTART;
	}
	return HRTIMER_NORESTART;
}

/* ---- sysfs 接口 ----
 *
 * sysfs 是内核导出的"虚拟文件系统" (/sys/...), 用户态用读写文件的
 * 方式与驱动交互。这里暴露三个文件:
 *   inject (只写) — echo 42 > inject   => 发 1 个字节 0x42
 *   burst  (只写) — echo "1000 1000000" > burst
 *                   => 每 1,000,000 ns (1ms) 发 1 个, 共 1000 个
 *   stats  (只读) — cat stats          => 查看注入/慢路径计数
 */

/* 通过 struct device 找回 struct valarm:
 * misc 框架把 miscdevice* 存进了 drvdata, 而 miscdevice 内嵌在
 * valarm 里, container_of 宏根据成员地址反推结构体首地址 */
static struct valarm *dev_to_valarm(struct device *dev)
{
	return container_of(dev_get_drvdata(dev), struct valarm, misc);
}

/* echo 42 > inject 时, 内核把 "42\n" 交给这个函数 */
static ssize_t inject_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct valarm *v = dev_to_valarm(dev);
	unsigned int byte;
	int ret = kstrtouint(buf, 0, &byte);   /* 字符串转无符号整数 */

	if (ret || byte > 255)
		return -EINVAL;
	valarm_inject(v, (u8)byte);
	return count;
}
static DEVICE_ATTR_WO(inject);

static ssize_t burst_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct valarm *v = dev_to_valarm(dev);
	unsigned int cnt;
	unsigned long long interval;

	if (sscanf(buf, "%u %llu", &cnt, &interval) != 2 || cnt == 0)
		return -EINVAL;

	hrtimer_cancel(&v->timer);
	v->burst_total = cnt;
	v->burst_sent  = 0;
	v->burst_interval_ns = interval;
	hrtimer_start(&v->timer, ns_to_ktime(interval), HRTIMER_MODE_REL);
	return count;
}
static DEVICE_ATTR_WO(burst);

static ssize_t stats_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct valarm *v = dev_to_valarm(dev);
	int injected = atomic_read(&v->injected);

	/* 丢字节/错序由用户态按字节序号检查 (见 alarmd 报告 seq_errors) */
	return sysfs_emit(buf,
		"injected=%d irq=%d next_byte=%u\n",
		injected, atomic_read(&v->irq_count), v->next_byte);
}
static DEVICE_ATTR_RO(stats);

static struct attribute *valarm_attrs[] = {
	&dev_attr_inject.attr,
	&dev_attr_burst.attr,
	&dev_attr_stats.attr,
	NULL,
};
ATTRIBUTE_GROUPS(valarm);

/* ---- 文件操作: mmap 是用户态能"直接摸寄存器"的关键 ----
 *
 * 用户态调用 mmap(NULL, 0x100, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0)
 * 时, 内核最终走到这里。我们的任务: 把申请的那页物理内存"挂"进
 * 该进程的地址空间, 之后用户态用普通指针读写这页 = 读写寄存器,
 * 全程不需要再进内核 (零系统调用, 这就是低延迟的来源)。
 */
static int valarm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *misc = filp->private_data;
	struct valarm *v = container_of(misc, struct valarm, misc);

	/*
	 * 注意: mmap 长度按页向上取整, 请求 0x100 字节时 vma 覆盖整页
	 * (4096), 故与 PAGE_SIZE 比较而非 VALARM_REGSZ (曾因误用
	 * VALARM_REGSZ 导致用户态 mmap 永远 EINVAL)。
	 */
	if (vma->vm_end - vma->vm_start > PAGE_SIZE)
		return -EINVAL;
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	/*
	 * 用 vm_insert_page 而非 remap_pfn_range: 后者把普通 RAM 页映射
	 * 进用户态会被部分加固内核拒绝 (EINVAL), 且 vm_insert_page 自动
	 * 处理页引用计数。映射为普通 WB 缓存属性, arm64 PIPT 下与内核
	 * 别名天然一致 (见文件头注释)。
	 */
	return vm_insert_page(vma, vma->vm_start, v->page);
}

/* 本驱动支持的文件操作表: 只实现了 mmap */
static const struct file_operations valarm_fops = {
	.owner = THIS_MODULE,
	.mmap  = valarm_mmap,
};

/* ---- 初始化 ----
 *
 * insmod 加载模块时执行。对两个虚拟串口各做四件事:
 *   1) alloc_page        申请一页物理内存当"寄存器块"
 *   2) hrtimer_init      准备好突发注入用的定时器
 *   3) misc_register     注册 misc 设备 -> 自动生成 /dev/valarmN
 *                        (misc 是内核提供的简易字符设备框架,
 *                         适合这种只有 mmap 一个功能的小设备)
 *   4) dev_info          打一条日志, 可以从 dmesg 看到
 */
static int __init valarm_init(void)
{
	int i, ret;

	pr_info("valarm: timestamp domain = CLOCK_MONOTONIC_RAW (ktime_get_raw_ns)\n");

	for (i = 0; i < VALARM_NDEV; i++) {
		struct valarm *v = &g_dev[i];

		v->page = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!v->page)
			return -ENOMEM;
		v->regs = page_address(v->page);

		/*
		 * 内核慢路径诊断 (方案 D5) 说明: 真实系统中报警字节到达会触发
		 * 串口 IRQ 并由内核中断统计; 本模拟框架不注册真实中断——
		 * 华为 HCE 内核魔改了 irq_alloc_descs 宏 (固定 THIS_MODULE
		 * 参数), 且虚拟中断的 desc->owner 机制会导致模块 rmmod
		 * 死锁 (free_irq 的 module_put 在 exit 中, exit 因引用计数
		 * 不为 0 永不运行)。慢路径统计改由 valarm_inject() 在注入时
		 * 直接记账 (见该函数注释), 可观测语义不变, 不在用户态热路径。
		 */
		hrtimer_init(&v->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		v->timer.function = valarm_burst_fn;

		v->misc.minor = MISC_DYNAMIC_MINOR;
		v->misc.name  = kasprintf(GFP_KERNEL, "valarm%d", i);
		v->misc.fops  = &valarm_fops;
		v->misc.groups = valarm_groups;
		ret = misc_register(&v->misc);
		if (ret)
			return ret;

		dev_info(v->misc.this_device,
			 "valarm%d ready: regs=%px\n", i, v->regs);
	}
	pr_info("valarm: init done, refs = %d\n", module_refcount(THIS_MODULE));
	return 0;
}

/* rmmod 卸载模块时执行, 顺序与申请相反: 先停定时器, 再注销设备
 * (设备节点随之消失), 最后释放内存 */
static void __exit valarm_exit(void)
{
	int i;

	for (i = 0; i < VALARM_NDEV; i++) {
		struct valarm *v = &g_dev[i];

		hrtimer_cancel(&v->timer);
		misc_deregister(&v->misc);
		if (v->page)
			__free_page(v->page);
		kfree(v->misc.name);
	}
}

module_init(valarm_init);
module_exit(valarm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("serial-alarm project");
MODULE_DESCRIPTION("Virtual UART + software-simulated alarm injector for "
		   "userspace MMIO busy-polling latency framework");
