/**
  ******************************************************************************
  * @file    proto_test.h
  * @brief   第 5 步协议测试（无线 A 路线）—— 角色开关 + 统计量
  *
  * ============================ 怎么用 ============================
  *   1. 下面 `PROTO_IS_TX` 置 **1** → 编译、烧到【发送端（手柄）】
  *   2. 改成 **0** → 编译、烧到【接收端（dongle）】
  *   3. 两块板都上电，10cm 内，跑 60 秒
  *   4. 分别看两边 Watch
  *
  * **验收判据**（详见 `无线A路线-引脚分配与接线.md` §5.8.5）：
  *     proto_rx_ok 涨、proto_rx_bad_type = 0、proto_rx_bad_sum = 0、proto_rx_bad_seq = 0
  *     拔掉一块板 → proto_link_up 在 100ms 内变 0（proto_link_lost +1）
  *     插回 → proto_link_up 恢复 1
  ******************************************************************************
  */
#ifndef __PROTO_TEST_H
#define __PROTO_TEST_H

#include <stdint.h>

/* ============================ 角色开关 ============================
   1 = 发送端（每 10ms 发一帧）
   0 = 接收端（每 1ms 轮询）

   ⚠️ 两块板必须烧【不同的值】！ */
#ifndef PROTO_IS_TX
#define PROTO_IS_TX   1
#endif

void proto_test_init(void);   /* 上电调一次：nrf24_init +（发送端）建任务 /（接收端）进接收模式 */
void proto_test_poll(void);   /* 1ms 任务里调：接收端轮询收包；发送端空操作 */

/* ================== 共用的链路状态 ================== */
extern volatile uint8_t  proto_link_up;    /* 1 = 链路通（断线判据见 proto.h） */

/* ================== 发送端（Watch 观察）================== */
extern volatile uint32_t proto_tx_sent;    /* 已发帧数 */
extern volatile uint32_t proto_tx_ok;      /* 收到硬件 ACK 的次数 */
extern volatile uint32_t proto_tx_maxrt;   /* 重传耗尽次数（判据：= 0） */
extern volatile uint16_t proto_last_seq;   /* 最后发出的序号 */

/* ================== 接收端（Watch 观察）================== */
extern volatile uint32_t proto_rx_ok;        /* 收到且校验全过 */
extern volatile uint32_t proto_rx_bad_type;  /* 帧类型错（应 = 0） */
extern volatile uint32_t proto_rx_bad_sum;   /* 校验和错（应 = 0） */
extern volatile uint32_t proto_rx_bad_seq;   /* 序号不连续（应 = 0） */
extern volatile uint32_t proto_rx_last_seq;  /* 最后收到的序号 */
extern volatile uint32_t proto_link_lost;    /* 断线次数（状态从通变断的次数） */

#endif /* __PROTO_TEST_H */
