/**
  ******************************************************************************
  * @file    p2p.h
  * @brief   两块板点对点测试（无线 A 路线 · 第 4 步）
  *
  * ============================ 怎么用 ============================
  *   1. 下面 `P2P_IS_TX` 置 **1** → 编译、烧到【发送端】
  *   2. 改成 **0** → 编译、烧到【接收端】
  *   3. 两块板都上电（10cm 内先测近距），跑到 10 秒
  *   4. 分别看两边 Watch 里的统计量
  *
  * **验收判据**（详见 `无线A路线-引脚分配与接线.md` §5.7.4）：
  *     p2p_tx_ok ≈ p2p_tx_sent        （每包都收到硬件 ACK）
  *     p2p_tx_maxrt = 0               （一包都不该重传耗尽）
  *     p2p_rx_ok ≈ p2p_tx_ok          （两边数对得上）
  *     p2p_rx_bad_data = 0            （字节一模一样）
  *     p2p_rx_bad_seq  = 0            （顺序正确）
  ******************************************************************************
  */
#ifndef __P2P_H
#define __P2P_H

#include <stdint.h>

/* ============================ 角色开关 ============================
   1 = 发送端（每 10ms 发一包，统计 ACK）
   0 = 接收端（每 1ms 轮询，统计收包 / 误码 / 乱序）

   ⚠️ 两块板必须烧【不同的值】！ */
#ifndef P2P_IS_TX
#define P2P_IS_TX   0
#endif

void p2p_init(void);   /* 上电调一次：nrf24_init +（发送端）建任务 /（接收端）进接收模式 */
void p2p_poll(void);   /* 在 1ms 任务里调：接收端轮询收包；发送端空操作 */

/* ================== 发送端（Watch 观察）================== */
extern volatile uint32_t p2p_tx_sent;    /* 已发包数 */
extern volatile uint32_t p2p_tx_ok;      /* 收到硬件 ACK 的次数（= 对方真的收到了） */
extern volatile uint32_t p2p_tx_maxrt;   /* 重传耗尽次数（判据：= 0） */
extern volatile uint16_t p2p_last_seq;   /* 最后发出的序号 */

/* ================== 接收端（Watch 观察）================== */
extern volatile uint32_t p2p_rx_ok;       /* 收到且三项校验全过 */
extern volatile uint32_t p2p_rx_bad_data; /* magic 或校验和错（判据：= 0） */
extern volatile uint32_t p2p_rx_bad_seq;  /* 序号不连续（判据：= 0） */
extern volatile uint32_t p2p_rx_last_seq; /* 最后收到的序号 */

#endif /* __P2P_H */
