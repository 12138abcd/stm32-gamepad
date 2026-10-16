/**
  ******************************************************************************
  * @file    nrf24_test.h
  * @brief   nRF24 驱动自检 —— 无线 A 路线 · 第 3 步的验收程序
  *
  * 说明：这【不是】驱动，是站在驱动上面的一次性验证。驱动在 nrf24.c / nrf24.h。
  *       分开的理由与 latency_probe 一样：测试代码混在产品代码里，看代码的人会被淹没。
  *
  * ======================== 为什么单板只能验到这一步 ========================
  * nRF24 是纯收发器：发送时接收机不工作 → 单块模块【不可能】"收到自己发的字节"。
  * 所以"收发一帧且内容一致"必须两块板，归第 4 步；本自检证明的是——
  *
  *   ① 单字节寄存器写-读回环（写 0x4C / 0x25 各读回一次）
  *   ② 多字节地址写-读回环（RX_ADDR_P0 五字节）
  *   ③ FIFO 通路（FLUSH_TX 后 TX_EMPTY=1 → 写一包 → TX_EMPTY=0 → FLUSH_TX → 又=1）
  *   ④ 发送状态机真的在跑（CE 脉冲后能等到 TX_DS 或 MAX_RT，而不是超时）
  *      ⚠️ 判据是"没超时"，不是"必须 MAX_RT"：单板无对端 → MAX_RT（重传耗尽，正常）；
  *         若旁边正好有配对好的对端 → TX_DS（收到 ACK）。两种都算通过。
  *   ⑤ 能切进接收模式（RX_P_NO = 111 且 RX FIFO 空）
  *
  * ======================== 用法 ========================
  * main.c 不用改：nrf24_test_init()（USER CODE 2）里会调 nrf24_init()，
  * nrf24_test_poll()（1ms 任务循环内）第一次被调用时跑一遍完整自检，之后只做
  * 100ms 一次的 STATUS 在线监视。
  * 想在 Watch 里重跑自检：把 nrf24_test_rerun 写成 1（下一拍生效，不用重烧固件）。
  ******************************************************************************
  */
#ifndef __NRF24_TEST_H
#define __NRF24_TEST_H

#include <stdint.h>

/* 上电调一次（内部做 nrf24_init() 硬件初始化） */
void nrf24_test_init(void);

/* 在 1ms 任务里调；内部自带节流，不会占用关键路径 */
void nrf24_test_poll(void);

/* 自检主流程（nrf24_test_poll 第一次被调用时会跑；也可在别处手动调一次） */
void nrf24_test_run(void);

/* ================== Keil Watch 观察项 ==================
   镜像：5 项检查各占一位，bit0~bit4　→　全过 = 0x1F */
extern volatile uint8_t  nrf24_test_mask;      /* ★ 期望 0x1F */
extern volatile uint8_t  nrf24_test_pass;      /* ★ 期望 1（= mask 全 1） */
extern volatile uint8_t  nrf24_test_done;      /* 自检是否跑完 */
extern volatile uint8_t  nrf24_test_rerun;     /* 写 1 → 下一拍重跑一遍自检 */

/* 分项结果（1 = 通过） */
extern volatile uint8_t  nrf24_chk_reg_rw;     /* bit0：单字节寄存器回环 */
extern volatile uint8_t  nrf24_chk_addr_rw;    /* bit1：五字节地址回环 */
extern volatile uint8_t  nrf24_chk_fifo;       /* bit2：TX FIFO 通路 */
extern volatile uint8_t  nrf24_chk_tx;         /* bit3：发送状态机有响应 */
extern volatile uint8_t  nrf24_chk_rx;         /* bit4：能进接收模式 */

/* 出问题时的取证材料 */
extern volatile uint32_t nrf24_fail_code;      /* 第一处失败：(项号 << 24) | 读回的值 */
extern volatile uint8_t  nrf24_addr_readback[5]; /* ②读回的五个字节（正常应等于 TEST9）*/
extern volatile uint8_t  nrf24_fifo_probe[3];  /* ③三次 FIFO_STATUS：清空后 / 写包后 / 再清空后 */

/* ④发送 */
extern volatile uint32_t nrf24_tx_result;      /* 0=TX_DS（有对端）/ 1=MAX_RT（无对端，正常）/ 2=超时（异常）*/
extern volatile uint32_t nrf24_observe_tx;     /* OBSERVE_TX：无对端时低位 ARC_CNT 应 = 3（重传真的发生了）*/
extern volatile uint32_t nrf24_tx_elapsed_us;  /* 这次发送耗时（µs）*/

/* ④的诊断量 —— 用来区分"CE 没触发"和"配置没生效" */
extern volatile uint8_t  nrf24_config_readback; /* init 写完 CONFIG 后读回，期望 0x0E（PWR_UP=1、PRIM_RX=0）*/
extern volatile uint8_t  nrf24_config_txmode;   /* ④发送【后】CONFIG 实测值，期望 0x0E。若为 0x08/0x00 → PWR_UP 没生效 */
extern volatile uint8_t  nrf24_cfg_before_send; /* ④发送【前】CONFIG 实测值，期望 0x0E */
extern volatile uint8_t  nrf24_fifo_after_tx;   /* 发送后 FIFO_STATUS。⚠️ 注意：MAX_RT（重传耗尽）之后
                                                    payload【不会】被自动清出 TX FIFO，所以这里 bit4(TX_EMPTY)=0
                                                    是正常的，不能当"CE 没触发"的证据 ——
                                                    判断"到底发出去没有"要看 nrf24_observe_tx 的 ARC_CNT。 */

/* ============ ④a：连发 NRF24_REPEAT_COUNT 包（盯着"上电后第一包"） ============
   来历：查那个奇怪的 52.4ms 时加的。它当时只在"某些次运行的第一包"出现过
   （正常值应是 3.25ms = 首次发送 + 3 次重传 ≈ 4 × 800µs）。
   把首包和后续包分开记，将来若再现就能一眼看到；清理诊断时保留它，成本极低。
   ⭐ 判读：若 [0] ≈ 52000 而 [1][2] ≈ 3250 → 首包异常再现。 */
#define NRF24_REPEAT_COUNT   3u
extern volatile uint32_t nrf24_tx_result_repeat[NRF24_REPEAT_COUNT];   /* 0=TX_DS / 1=MAX_RT / 2=超时 */
extern volatile uint32_t nrf24_tx_elapsed_repeat[NRF24_REPEAT_COUNT];  /* ⭐ 耗时（µs）*/
extern volatile uint32_t nrf24_observe_repeat[NRF24_REPEAT_COUNT];     /* OBSERVE_TX */

/* ⑤接收 */
extern volatile uint8_t  nrf24_rx_status_raw;  /* 进 RX 模式后读到的 STATUS（RX_P_NO 应为 111）*/
extern volatile uint8_t  nrf24_rx_fifo_raw;    /* 同上的 FIFO_STATUS */
extern volatile uint8_t  nrf24_rpd;            /* 参考值：接收功率检测，附近有 2.4G 载波时为 1（不做判据）*/
extern volatile uint8_t  nrf24_ce_readback;    /* ⚠️ CE(PB3) 引脚回环：1 = 正常。若为 0 → PB3 被 JTAG 抢走了 */

/* 在线监视（自检跑完后每 100ms 一次）—— 沿用第 2 步的观察习惯 */
extern volatile uint32_t nrf24_status;         /* 期望 0x0E */
extern volatile uint8_t  nrf24_status_b;       /* 紧接着再读一次，应与上一个相同 */
extern volatile uint32_t nrf24_reads;          /* 监视轮数 */

#endif /* __NRF24_TEST_H */
