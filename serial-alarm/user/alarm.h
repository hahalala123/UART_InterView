/* alarm.h — libfoo 回调接口 (需求定义: 处理点必须在用户态) */
#ifndef ALARM_H
#define ALARM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * foo(port, byte, t0) — 用户态处理入口
 *  - port: 警报来源串口 (0 / 1)
 *  - byte: 报警数据字节
 *  - t0  : 字节 RBR 就绪时刻 (ns, 与测量通道同源)
 */
void foo(int port, uint8_t byte, uint64_t t0);

#ifdef __cplusplus
}
#endif

#endif /* ALARM_H */
