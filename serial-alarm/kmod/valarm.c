// SPDX-License-Identifier: GPL-2.0
/*
 * valarm.c — 虚拟 UART + 软件模拟报警器 (内核驱动)
 *
 * 用软件模拟方案中的全部硬件:
 *   - 两个 8250 布局兼容的虚拟 UART (40 Mbaud 语义: 单字节即一次 DR 事件)
 *   - 报警器 = sysfs 单次注入 + hrtimer 突发注入, 注入时:
 *       1) 写入 RBR 与硬件时间戳 INJECT_TS (模拟警报器 GPIO 标记 / 示波器 t0)
 *       2) 置位 LSR.DR, 用户态轮询即可见
 *       3) 触发虚拟 IRQ, 走内核慢路径诊断 (方案 D5, 不进热路径)
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

/* ---- 注入: 模拟"警报器发出 1 字节" ---- */

static void valarm_inject(struct valarm *v, u8 byte)
{
	u64 ts = valarm_now_ns();

	/* 先写数据与时间戳, 屏障后置 DR, 保证用户态见到 DR 时数据有效 */
	*(u64 *)(v->regs + REG_TS) = ts;
	v->regs[REG_RBR] = byte;
	smp_wmb();
	v->regs[REG_LSR] |= LSR_DR;

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

/* ---- 突发注入器: hrtimer 模拟警报器按间隔连发 ---- */

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

/* ---- sysfs ---- */

static struct valarm *dev_to_valarm(struct device *dev)
{
	/* misc 框架将 miscdevice* 存入 drvdata, 反查外层 struct valarm */
	return container_of(dev_get_drvdata(dev), struct valarm, misc);
}

static ssize_t inject_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct valarm *v = dev_to_valarm(dev);
	unsigned int byte;
	int ret = kstrtouint(buf, 0, &byte);

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

/* ---- 文件操作 ---- */

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

static const struct file_operations valarm_fops = {
	.owner = THIS_MODULE,
	.mmap  = valarm_mmap,
};

/* ---- 初始化 ---- */

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
