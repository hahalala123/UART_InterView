# serial-alarm —— 高速串口警报实时处理系统（远程 ARM 服务器版）

依据《串口警报实时处理技术方案》实现的完整代码框架。
**硬件全部软件模拟**：内核驱动模拟两个 UART 与报警器，用户态 alarmd
通过 mmap 忙轮询寄存器（对应真实方案的 uio_pdrv_genirq + /dev/uioN 路径）。
编译、加载、测试全部在远程 ARM 服务器上完成。

## 架构

```
┌────────────────────── 用户态 ──────────────────────┐
│ alarmd (SCHED_FIFO 99, 绑核, mlockall, 预热 PLT)    │
│   热循环: lsr&DR → rbr → inject_ts → ack → foo()   │
│        mmap (/dev/valarm0, /dev/valarm1, vm_insert_page)  │
├────────────────────── 内核态 ──────────────────────┤
│ valarm.ko (misc 驱动)                               │
│   ├─ 虚拟 UART0/1: RBR/LSR/INJECT_TS 寄存器页        │
│   ├─ 报警器模拟: sysfs inject(单次) / burst(hrtimer 突发) │
│   └─ 慢路径诊断: 注入路径直接记账 (方案 D5, 不进热路径)     │
└─────────────────────────────────────────────────────┘
```

延迟测量（方案 7.1）：`INJECT_TS`（内核注入时刻，模拟硬件时间戳/示波器 t0）
与 alarmd 中 `foo()` 入口的 `ts_now_ns()` 之差，输出 avg / p99 / max /
丢字节 / 错序，并导出 CSV。

## 寄存器布局（内核/用户态共享，见 `user/uart_mmio.h`）

| 偏移 | 名称 | 说明 |
|---|---|---|
| 0x00 | RBR | 报警字节（用户态读） |
| 0x04 | ACK | 用户态读后写任意值清 DR（真实硬件自清，模拟需显式 ack） |
| 0x05 | LSR | bit0 = DR |
| 0x08 | INJECT_TS | u64 注入时刻 ns（模拟硬件时间戳） |
| 0x10 | IRQ_CNT | u32 内核慢路径中断计数（诊断） |

## 快速开始（在 ARM 服务器上操作）

把整个 `serial-alarm/` 目录拷贝到服务器（scp/U盘/共享目录均可），然后：

```bash
cd serial-alarm

# 1) 编译（需要内核 headers: /lib/modules/$(uname -r)/build）
make

# 2) 一键测试：加载模块 + 突发注入 + 延迟报告 + 计数比对 + 卸载
make test                      # 默认 1000 次/口, 间隔 1 ms
make test TEST_COUNT=100 TEST_INTERVAL=100000   # 100 次突发, 间隔 100 us
```

服务器上手动测试：

```bash
sudo insmod kmod/valarm.ko
sudo ./user/alarmd &                     # 需要 root 以获得 SCHED_FIFO 99
echo "1000 1000000" | sudo tee /sys/class/misc/valarm0/burst   # 口 0 突发
sudo kill -USR1 %1                       # 输出延迟报告并写 CSV
python3 tools/stats.py alarm_latency.csv --hist 20             # 直方图
sudo cat /sys/class/misc/valarm0/stats   # 内核侧计数比对
sudo rmmod valarm
```

## 目录结构

```
serial-alarm/
├── Makefile                 # 服务器本机入口: make / make test / make clean
├── README.md
├── kmod/
│   ├── valarm.c             # 内核驱动: 虚拟 UART + 报警器注入器 + 慢路径 IRQ
│   └── Makefile             # kbuild
├── user/
│   ├── alarmd.c             # 用户态驱动: 初始化 + 热轮询 + 测量报告
│   ├── uart_mmio.h          # 寄存器布局 (与内核一致)
│   ├── timestamp.h          # CNTPCT / CLOCK_MONOTONIC_RAW 双实现 (方案 D4)
│   ├── ring.h               # 环形缓冲 (方案 5.6, 突发背压)
│   ├── alarm.h              # libfoo 回调接口
│   ├── Makefile
│   └── libfoo/foo.c         # libfoo.so 桩 (PLT 预热/执行时间评估)
└── tools/
    ├── run_test.sh          # 服务器端测试主流程 (验收 7.2/7.3)
    └── stats.py             # CSV 统计 + 直方图
```

## 与技术方案的对应

| 方案决策 | 本框架实现 |
|---|---|
| D1 用户态 MMIO 忙轮询 | `alarmd.c` 热循环，零 syscall/调度/中断 |
| D2 寄存器映射进用户态 | `valarm.ko` misc mmap（真实硬件对应 uio_pdrv_genirq + 设备树） |
| D3 CPU 独占 | `mlockall` + `SCHED_FIFO 99` + `sched_setaffinity` |
| D4 CNTPCT 时间戳 | `timestamp.h`（aarch64 用 `mrs cntpct_el0`，退化 vDSO） |
| D5 内核慢路径诊断 | `valarm.ko` 注入路径慢路径记账（原设计为虚拟 IRQ，因 HCE 内核 IRQ API 魔改 + desc->owner 卸载死锁风险改为直接记账） |
| 5.6 突发背压 | `ring.h`（当前 hot path 内联 foo + 样本记录，`ring.h` 供拆分消费线程时使用） |
| 7.1 双通道测量 | 通道一模拟：INJECT_TS 硬件时间戳；通道二：软件统计报告 |
| 7.3 验收 | 字节序号连续性检查（零丢字节/零错序）+ 内核/用户态计数比对 |

## 模拟与真实硬件的差异（移植到目标板时的改动点）

1. `valarm.ko` 整模块删除 → 设备树加 `generic-uio` 节点，由主线
   `uio_pdrv_genirq` 提供 `/dev/uioN`；alarmd 打开路径从
   `/dev/valarmN` 改为 `/dev/uioN`（mmap 偏移 0 即寄存器基址）。
2. 删除 ACK 写（真实 8250 读 RBR 自清 DR）；`INJECT_TS` 若无硬件
   时间戳，t0 改用"轮询到 DR 的本地时刻"，精度损失为一个轮询周期。
3. 注入器换成真实警报器 + GPIO 标记 + 示波器（方案 7.1 通道一）。
