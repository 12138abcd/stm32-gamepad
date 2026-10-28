/**
  ******************************************************************************
  * @file    wireless_rx.h
  * @brief   dongle 端无线接收（第 6 步，第 7 步加帧类型分流 + 断线空闲报告）
  *
  * 设计：
  *   - 1ms 任务调 wireless_rx_poll()：读 STATUS → 收包 → proto_verify
  *     → 按帧类型分流（报告帧 / 心跳帧）→ 写 gamepad_report → send_report()
  *   - 断线时发【空闲报告】，防"卡键"
  *   - 非阻塞（一次 SPI 事务 ≈20µs），可以放心放 1ms 任务
  ******************************************************************************
  */
#ifndef __WIRELESS_RX_H
#define __WIRELESS_RX_H

#include <stdint.h>

void wireless_rx_init(void);   /* 上电调一次：nrf24_init + 进接收模式 */
void wireless_rx_poll(void);   /* 1ms 任务里调：收包 + 分流 + 转 USB */

/* ================== 统计量（Watch 观察）================== */
extern volatile uint32_t wrx_ok;        /* 收到合法【报告帧】且校验全过 */
extern volatile uint32_t wrx_bad_type;  /* 帧类型不认识 */
extern volatile uint32_t wrx_bad_sum;   /* 校验和错 */
extern volatile uint32_t wrx_bad_seq;   /* 序号不连续（断线跳号会涨，属正常） */
extern volatile uint32_t wrx_last_seq;  /* 最后收到的序号 */
extern volatile uint32_t wrx_link_lost; /* 断线次数 */
extern volatile uint8_t  wrx_link_up;   /* 链路状态 */
extern volatile uint32_t wrx_usb_sent;  /* 转给 USB 的报告帧数 */
extern volatile uint32_t wrx_heartbeat; /* 收到的心跳帧数（有线模式下应 ≈50/秒） */
extern volatile uint32_t wrx_idle_sent; /* 断线时补发的空闲报告数 */

/* ---- 第 9 步：配对 ---- */
extern volatile uint8_t  wrx_paired_id; /* 已配对的 ID；0 = 未配对 / 学习模式 */
extern volatile uint32_t wrx_pair_ok;   /* 配对成功次数 */
extern volatile uint32_t wrx_pair_rej;  /* 因 ID 不匹配被丢弃的帧数 */
extern volatile uint32_t wrx_pair_drop; /* 窗口外收到的配对帧数 */

#endif /* __WIRELESS_RX_H */
