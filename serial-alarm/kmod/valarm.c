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
 *   0x04 ACK        [WO] 用户态取数后写任意值, 清除 DR
 *                        (真实硬件读 RBR 自清; 模拟硬件需要显式 ack)
 *   0x05 LSR        [RO] bit0 = DR
 *   0x08 INJECT_TS  [RO] u64, 注入时刻 ktime (ns) — 模拟硬件时间戳通道
 *   0x10 IRQ_CNT    [RO] u32, 内核慢路径中断计数 (诊断)
 *
 * 一致性说明: 用户态经 vm_insert_page 映射为普通缓存映射, 与内核
 * 侧 page_address 别名同为 WB 属性且访问同一物理地址; arm64 为
 * PIPT cache, 不存在别名不一致问题。不用 remap_pfn_range + 非缓存
 * 的原因: 部分厂商加固内核 (如华为 HCE) 禁止将普通 RAM 页 remap
 * 进用户态 (mmap 返回 EINVAL), vm_insert_page 是映射已分配页的
 * 标准 API。若移植到 VIPT 平台需重新评估缓存属性。
 *
 * 接口:
 *   /dev/valarm0, /dev/valarm1        — mmap (寄存器窗口) + read (统计)
 *   /sys/.../valarmN/inject  (WO)     — 写 0-255, 单次注入该字节
 *   /sys/.../valarmN/burst   (WO)     — 写 "<count> <interval_ns>", 突发注入
 *   /sys/.../valarmN/stats   (RO)     — injected/acked/irq/loss 计数
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
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
	int virq;
	struct hrtimer timer;
	u32 burst_total;
	u32 burst_sent;
	u64 burst_interval_ns;
	u8  next_byte;            /* 注入字节按序号递增, 供用户态查错序/丢字节 */
	atomic_t injected;
	atomic_t acked;
	atomic_t irq_count;
};

static struct valarm g_dev[VALARM_NDEV];

/* ---- 虚拟 IRQ: 内核慢路径诊断 (方案 D5) ---- */

static void virq_noop(struct irq_data *d) { }
static struct irq_chip valarm_chip = {
	.name         = "valarm",
	.irq_mask     = virq_noop,
	.irq_unmask   = virq_noop,
	.irq_ack      = virq_noop,
};

static irqreturn_t valarm_irq_handler(int irq, void *dev_id)
{
	struct valarm *v = dev_id;
	/* 慢路径诊断: 模拟内核串口中断处理 (统计用, 不参与热路径) */
	*(u32 *)(v->regs + REG_IRQCNT) = (u32)atomic_inc_return(&v->irq_count);
	return IRQ_HANDLED;
}

/* ---- 注入: 模拟"警报器发出 1 字节" ---- */

static void valarm_inject(struct valarm *v, u8 byte)
{
	u64 ts = ktime_get_ns();

	/* 先写数据与时间戳, 屏障后置 DR, 保证用户态见到 DR 时数据有效 */
	*(u64 *)(v->regs + REG_TS) = ts;
	v->regs[REG_RBR] = byte;
	smp_wmb();
	v->regs[REG_LSR] |= LSR_DR;

	atomic_inc(&v->injected);
	/*
	atomic_inc(&v->injected);
	/*
	 * generic_handle_irq_safe() 为 5.11+ 接口; 5.10 及更早用
	 * generic_handle_irq() 并自行关中断, 语义等价 (该路径可能在
	 * 中断上下文被 hrtimer 调用, 关中断防止嵌套触发虚拟 IRQ)。
	 * virq < 0 表示 IRQ 诊断通道已降级禁用, 跳过。
	 */
	if (v->virq >= 0) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
		generic_handle_irq_safe(v->virq);
#else
		unsigned long flags;

		local_irq_save(flags);
		generic_handle_irq(v->virq);
		local_irq_restore(flags);
#endif
	}
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

	if (vma->vm_end - vma->vm_start > VALARM_REGSZ)
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

	for (i = 0; i < VALARM_NDEV; i++) {
		struct valarm *v = &g_dev[i];

		v->page = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!v->page)
			return -ENOMEM;
		v->regs = page_address(v->page);

		/* 虚拟 IRQ 初始化 (可选, 方案 D5 诊断通道, 不参与热路径) */
		/*
		 * owner 必须传 NULL 而非用 irq_alloc_desc() 宏:
		 * 该宏默认给 desc->owner 填 THIS_MODULE, request_irq 会
		 * 对其 try_module_get 使模块引用 +1; 而 free_irq 的
		 * module_put 在模块 exit 中, exit 又因引用计数不为 0
		 * 永远不会被调用 —— rmmod 永久死锁 (实测 refcnt=2)。
		 * 部分加固内核对 NULL owner 的申请返回 EINVAL, 此时
		 * 降级为禁用 IRQ 诊断 (仅损失 D5 中断计数演示, 不影响
		 * 主测试路径), 保证模块可加载可卸载。
		 */
		v->virq = irq_alloc_descs(0, 1, numa_node_id(), NULL);
		if (v->virq < 0) {
			pr_warn("valarm%d: irq_alloc_descs(NULL owner) failed: %d, "
				"IRQ diagnostic disabled\n", i, v->virq);
			v->virq = -1;
		} else {
			irq_set_chip_and_handler_name(v->virq, &valarm_chip,
						      handle_simple_irq, "valarm");
			ret = request_irq(v->virq, valarm_irq_handler, 0,
					  "valarm", v);
			if (ret) {
				pr_warn("valarm%d: request_irq failed: %d, "
					"IRQ diagnostic disabled\n", i, ret);
				irq_free_desc(v->virq);
				v->virq = -1;
			}
		}
		pr_info("valarm%d: virq=%d refs=%d\n",
			i, v->virq, module_refcount(THIS_MODULE));

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
			 "valarm%d ready: regs=%px virq=%d\n", i, v->regs, v->virq);
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
		if (v->virq >= 0) {
			free_irq(v->virq, v);
			irq_free_desc(v->virq);
		}
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
