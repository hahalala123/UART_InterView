/* uart_mmio.h — 虚拟 UART 寄存器布局与用户态 MMIO 访问 (方案 D1)
 *
 * 与内核驱动 kmod/valarm.c 的寄存器定义严格一致。
 * 热路径只访问 LSR / RBR / INJECT_TS / ACK 四个偏移。
 * 所有访问必须 volatile, 防止编译器把轮询循环优化掉。
 */
#ifndef UART_MMIO_H
#define UART_MMIO_H

#include <stdint.h>

/* 寄存器偏移 (byte) */
#define UART_REG_RBR    0x00    /* [RO] 报警字节 */
#define UART_REG_ACK    0x04    /* [WO] 取数后写任意值清 DR (模拟硬件) */
#define UART_REG_LSR    0x05    /* [RO] bit0=DR */
#define UART_REG_TS     0x08    /* [RO] u64 注入时间戳 (模拟硬件时间戳) */
#define UART_REG_IRQCNT 0x10    /* [RO] u32 内核慢路径中断计数 (诊断) */

#define UART_LSR_DR (1u << 0)

#define UART_MAP_SIZE 0x100

typedef struct {
	volatile uint8_t *base;     /* mmap 基址 */
} uart_mmio_t;

static inline uint8_t uart_lsr(const uart_mmio_t *u)
{
	return *(volatile uint8_t *)(u->base + UART_REG_LSR);
}

static inline uint8_t uart_rbr(const uart_mmio_t *u)
{
	return *(volatile uint8_t *)(u->base + UART_REG_RBR);
}

/* 注入时间戳: 模拟硬件在报警时刻锁存的时间 (ns) */
static inline uint64_t uart_inject_ts(const uart_mmio_t *u)
{
	return *(volatile uint64_t *)(u->base + UART_REG_TS);
}

/* 模拟硬件: 用户态读后显式清除 DR (真实硬件读 RBR 自清) */
static inline void uart_ack(const uart_mmio_t *u)
{
	*(volatile uint8_t *)(u->base + UART_REG_ACK) = 0;
}

#endif /* UART_MMIO_H */
