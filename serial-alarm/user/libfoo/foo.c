// SPDX-License-Identifier: MIT
/*
 * libfoo 桩实现 — 模拟需求方的处理库。
 *
 * 故意 noinline 且带一点真实工作量, 用于:
 *  - 验证 PLT 预热路径 (LD_BIND_NOW / 首次调用)
 *  - 测量 foo() 执行时间对热路径的影响 (方案 8.3)
 */
#include <stdint.h>
#include "../alarm.h"

static volatile uint64_t g_sink;
static volatile uint64_t g_calls;

__attribute__((noinline))
void foo(int port, uint8_t byte, uint64_t t0)
{
	uint64_t x = g_sink ^ ((uint64_t)byte << 32) ^ (uint64_t)port ^ t0;

	/* 少量计算, 防止被优化成空函数 */
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	g_sink = x;
	g_calls++;
}
