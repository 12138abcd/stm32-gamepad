/**
  ******************************************************************************
  * @file    wireless_tx.h
  * @brief   手柄端无线发送（第 6 步，第 7 步加通道节奏）—— 队列 + 独立任务
  *
  * 设计：
  *   - 1ms 任务调 wireless_tx_on_tick()：把最新 gamepad_report 塞进队列（不阻塞）
  *       · 无线模式：每拍塞【报告帧】
  *       · 有线模式：每 20ms 塞一帧【心跳帧】（只为保活，dongle 收到不写报告）
  *   - 独立任务 wireless_tx_task()：取队列 → proto_pack_ex → nrf24_send
  *
  * 为什么不用 1ms 直接发：nrf24_send() 会阻塞 1~3ms，塞进 1ms 任务会打崩节拍。
  ******************************************************************************
  */
#ifndef __WIRELESS_TX_H
#define __WIRELESS_TX_H

#include <stdint.h>

void wireless_tx_init(void);      /* 上电调一次：nrf24_init + 建队列 */
void wireless_tx_on_tick(void);   /* 1ms 任务里调：按通道节奏塞队列 */
void wireless_tx_task(void *argument);  /* 独立任务入口 */

/* ================== 统计量（Watch 观察）================== */
extern volatile uint32_t wtx_sent;      /* 已发帧数（含心跳） */
extern volatile uint32_t wtx_ok;        /* 收到硬件 ACK 的次数 */
extern volatile uint32_t wtx_maxrt;     /* 重传耗尽次数 */
extern volatile uint32_t wtx_dropped;   /* 队列满丢弃的次数 */
extern volatile uint32_t wtx_heartbeat; /* 心跳帧发送数（有线模式下应 ≈50/秒） */
extern volatile uint32_t wtx_timeout;   /* nrf24_send 返回 TIMEOUT 的次数（异常） */
extern volatile uint16_t wtx_last_seq;  /* 最后发出的序号 */
extern volatile uint8_t  wtx_link_up;   /* 链路状态 */

#endif /* __WIRELESS_TX_H */
