/**
  ******************************************************************************
  * @file    wireless_rx.h
  * @brief   dongle 端无线接收（第 6 步）—— 收包 → 写报告 → 转 USB
  *
  * 设计：
  *   - 1ms 任务调 wireless_rx_poll()：读 STATUS → 收包 → proto_unpack
  *     → 写 gamepad_report → send_report()（USB）
  *   - 非阻塞（一次 SPI 事务 ≈20µs），可以放心放 1ms 任务
  ******************************************************************************
  */
#ifndef __WIRELESS_RX_H
#define __WIRELESS_RX_H

#include <stdint.h>

void wireless_rx_init(void);   /* 上电调一次：nrf24_init + 进接收模式 */
void wireless_rx_poll(void);   /* 1ms 任务里调：收包 + 转 USB */

/* ================== 统计量（Watch 观察）================== */
extern volatile uint32_t wrx_ok;        /* 收到且校验全过 */
extern volatile uint32_t wrx_bad_type;  /* 帧类型错 */
extern volatile uint32_t wrx_bad_sum;   /* 校验和错 */
extern volatile uint32_t wrx_bad_seq;   /* 序号不连续 */
extern volatile uint32_t wrx_last_seq;  /* 最后收到的序号 */
extern volatile uint32_t wrx_link_lost; /* 断线次数 */
extern volatile uint8_t  wrx_link_up;   /* 链路状态 */
extern volatile uint32_t wrx_usb_sent;  /* 转给 USB 的帧数 */

#endif /* __WIRELESS_RX_H */
