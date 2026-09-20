/* timestamp.h — 用户态纳秒时间戳 (方案 D4)
 *
 * 首选 ARMv8 CNTPCT: mrs 指令直读架构计数器, 无系统调用, ~几 ns。
 * 前提: 内核开放 EL0 访问 (CNTKCTL_EL1.EL0VCTEN=1)。
 * 退化: vDSO clock_gettime(CLOCK_MONOTONIC_RAW), ~20~30 ns。
 *
 * 注意: 模拟 INJECT_TS 来自内核 ktime_get_ns(), 与 CLOCK_MONOTONIC
 * 同源; 若目标板开启 CNTPCT 路径, 需确认两者时钟域一致 (CNTVCT 与
 * ktime 均以 CNTFRQ 为基, 通常一致, 验证阶段用示波器标定)。
 */
#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>
#include <time.h>

#if defined(__aarch64__) && !defined(FORCE_VDSO_TS)

static inline uint64_t ts_now_ns(void)
{
	uint64_t t, f;
	__asm__ volatile("mrs %0, cntpct_el0" : "=r"(t));
	__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
	return (uint64_t)((__uint128_t)t * 1000000000ull / f);
}

#else

static inline uint64_t ts_now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

#endif

#endif /* TIMESTAMP_H */
