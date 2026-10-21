/**
  ******************************************************************************
  * @file    wireless_tx.h
  * @brief   手柄端无线发送（第 6 步）—— 队列 + 独立任务
  *
  * 设计：
  *   - 1ms 任务调 wireless_tx_on_tick()：把最新 gamepad_report 塞进队列（不阻塞）
  *   - 独立任务 wireless_tx_task()：从队列取 → proto_pack → nrf24_send（2ms 一帧）
  *
  * 为什么不用 1ms 直接发：nrf24_send() 会阻塞 1~3ms，塞进 1ms 任务会打崩节拍。
  ******************************************************************************
  */
#ifndef __WIRELESS_TX_H
#define __WIRELESS_TX_H

#include <stdint.h>

void wireless_tx_init(void);      /* 上电调一次：nrf24_init + 建队列 */
void wireless_tx_on_tick(void);   /* 1ms 任务里调：报告塞队列 */
void wireless_tx_task(void *argument);  /* 独立任务入口 */

/* ================== 统计量（Watch 观察）================== */
extern volatile uint32_t wtx_sent;      /* 已发帧数 */
extern volatile uint32_t wtx_ok;        /* 收到硬件 ACK 的次数 */
extern volatile uint32_t wtx_maxrt;     /* 重传耗尽次数 */
extern volatile uint32_t wtx_dropped;   /* 队列满丢弃的次数 */
extern volatile uint16_t wtx_last_seq;  /* 最后发出的序号 */
extern volatile uint8_t  wtx_link_up;   /* 链路状态 */

#endif /* __WIRELESS_TX_H */
