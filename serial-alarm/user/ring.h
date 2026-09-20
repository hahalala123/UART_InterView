/* ring.h — 单生产者单消费者环形缓冲 (方案 5.6, 突发背压)
 *
 * 生产者 = 热轮询循环 (取字节+时间戳+入队, 要求 < 1 us)
 * 消费者 = 顺序调用 foo() 的路径
 * capacity 为 2 的幂; head/tail 单调递增, 按位与掩码取模。
 */
#ifndef RING_H
#define RING_H

#include <stdint.h>

#define RING_CAPACITY 1024
#define RING_MASK     (RING_CAPACITY - 1)

typedef struct {
	uint8_t  byte;
	uint8_t  port;
	uint16_t _pad;
	uint64_t t0;
} ring_item_t;

typedef struct {
	volatile uint32_t head;
	volatile uint32_t tail;
	ring_item_t buf[RING_CAPACITY];
} ring_t;

static inline int ring_push(ring_t *r, ring_item_t item)
{
	uint32_t h = r->head;
	if (h - r->tail == RING_CAPACITY)
		return -1;
	r->buf[h & RING_MASK] = item;
	__atomic_store_n(&r->head, h + 1, __ATOMIC_RELEASE);
	return 0;
}

static inline int ring_pop(ring_t *r, ring_item_t *out)
{
	uint32_t t = r->tail;
	if (t == r->head)
		return -1;
	*out = r->buf[t & RING_MASK];
	__atomic_store_n(&r->tail, t + 1, __ATOMIC_RELEASE);
	return 0;
}

#endif /* RING_H */
