# 会话状态（2026-09-21 00:35）

## 项目
串口警报实时处理系统（技术方案验证框架），工程在 `serial-alarm/`，
部署于华为鲲鹏 ECS（HCE2 内核 5.10.0-182.0.0.95.r3582_286，aarch64）。
架构：kmod/valarm.ko（misc 驱动，模拟双 UART + sysfs/hrtimer 报警注入）
+ user/alarmd（mmap 忙轮询热路径）。用法：服务器上 `make test`。

## 状态：✅ 全流程跑通，实测数据有效
- 1000 次×2 口突发 @1ms：p50=140ns，p99=190ns，avg=511ns（含 2 次云 stall 离群）
- max=509µs / seq_errors=2 / 丢 ~96B 均为云 steal time 冻结 hrtimer 所致，非框架缺陷
- 模块加载/卸载干净（refs=0）

## 关键坑（面试素材，详见各文件注释）
1. `irq_alloc_desc()` 宏给 desc->owner 填 THIS_MODULE → request_irq 引用+1，
   free_irq 在 exit 里而 exit 因 refs>0 永不执行 → rmmod 死锁。解法：不用虚拟 IRQ。
2. HCE 魔改 `irq_alloc_descs` 宏（固定 THIS_MODULE 参数，参数个数都对不上）。
3. mmap 的 vma 按页取整：请求 0x100 实际覆盖 4096，长度检查须与 PAGE_SIZE 比。
4. HCE 加固内核拦 remap_pfn_range 映射 RAM 页 → 用 vm_insert_page（WB 映射，
   arm64 PIPT 双别名天然一致）。
5. aarch64 默认只开 EL0VCTEN，`mrs cntpct_el0` 未授权 → SIGILL。
6. 云虚拟机 MONOTONIC 与 MONOTONIC_RAW 域偏移 ~0.84s 且漂移（单次标定无效）
   → 注入时间戳与测量都必须用同一域：ktime_get_raw_ns() ↔ clock_gettime(RAW)。
7. HCE 工具链汇编器不认 `cntvct_el1`/`cntfrq_el1` 寄存器名。
8. 后台 `sudo cmd &` 后 `$!` 是 sudo 的 PID，kill 信号不转发给子进程 → 去掉 sudo。

## 待办（可选）
- 把实测数据整理成测试报告（方案 7.3 验收对照）
- 坑点整理进 docs/pitfalls.md
- 移植真实硬件：valarm.ko 换 uio_pdrv_genirq + 设备树（README 第"差异"节）
