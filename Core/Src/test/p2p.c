/**
  ******************************************************************************
  * @file    p2p.c
  * @brief   两块板点对点测试（无线 A 路线 · 第 4 步）
  *
  * 角色由 `p2p.h` 里的 `P2P_IS_TX` 决定：
  *     1 = 发送端（每 10ms 发一包，统计 ACK）
  *     0 = 接收端（每 1ms 轮询，统计收包 / 误码 / 乱序）
  *
  * ============================ 载荷格式（必须 32 字节）============================
  *   [0..1]   seq（小端，每包 +1）
  *   [2..3]   magic = 0xA5 0x5A     （识别错位 / 脏数据）
  *   [4..5]   模式字节 0x01 0xFE
  *   [6..9]   保留 0
  *   [10..11] 校验和（对 [0..9] 累加，小端）
  *   [12..31] 补 0
  *
  * ⚠️ 长度必须 = nrf24_init() 里的 RX_PW_P0（= 32）。
  *    Enhanced ShockBurst 要求收发双方长度一致，改长度要同时改 RX_PW_P0。
  ******************************************************************************
  */

#include "p2p.h"
#include "nrf24.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

#define P2P_PAYLOAD_LEN   32u   /* 必须与 NRF24_REG_RX_PW_P0 一致 */
#define P2P_SUM_LEN       10u   /* 参与校验和累加的字节数 */

#if P2P_IS_TX
/* ======================================================================
 *                              发 送 端
 * ====================================================================== */

volatile uint32_t p2p_tx_sent  = 0u;   /* 已发包数 */
volatile uint32_t p2p_tx_ok    = 0u;   /* 收到硬件 ACK 的次数 */
volatile uint32_t p2p_tx_maxrt = 0u;   /* 重传耗尽次数 */
volatile uint16_t p2p_last_seq = 0u;   /* 最后发出的序号 */

static uint16_t p2p_seq = 0u;

static void p2p_task(void *argument)
{
    uint8_t  payload[P2P_PAYLOAD_LEN];
    uint16_t sum;
    uint8_t  i;
    uint8_t  ret;

    (void)argument;

    /* 上电后等一会儿再开始：让 nrf24_init 的上电时序（Tpd2stby）走完，
       也避开"刚上电那几包"的偶发异常 */
    osDelay(500u);

    for (;;)
    {
        /* ---------- 组包 ---------- */
        memset(payload, 0, sizeof(payload));

        payload[0] = (uint8_t)(p2p_seq & 0xFFu);   /* seq 低 */
        payload[1] = (uint8_t)(p2p_seq >> 8);      /* seq 高 */
        payload[2] = 0xA5u;                        /* magic */
        payload[3] = 0x5Au;
        payload[4] = 0x01u;                        /* 模式字节 */
        payload[5] = 0xFEu;

        sum = 0u;
        for (i = 0u; i < P2P_SUM_LEN; i++) { sum += payload[i]; }
        payload[10] = (uint8_t)(sum & 0xFFu);
        payload[11] = (uint8_t)(sum >> 8);

        /* ---------- 发一包 ---------- */
        ret = nrf24_send(payload, P2P_PAYLOAD_LEN, 50u);

        p2p_tx_sent++;
        if (ret == NRF24_TX_OK)         { p2p_tx_ok++; }     /* 收到 ACK = 对方真收到了 */
        else if (ret == NRF24_TX_MAXRT) { p2p_tx_maxrt++; }  /* 重传耗尽 = 没对端/配置不一致 */

        p2p_last_seq = p2p_seq;
        p2p_seq++;

        /* ---------- 10ms 一包 ≈ 100 包/秒 ---------- */
        osDelay(10u);
    }
}

void p2p_init(void)
{
    static const osThreadAttr_t attr = {
        .name       = "p2pTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t) osPriorityBelowNormal,   /* 低于 defaultTask */
    };

    nrf24_init();
    osThreadNew(p2p_task, NULL, &attr);
}

void p2p_poll(void)
{
    /* 发送端：1ms 任务里什么都不做（发送在 p2p_task 里，避免阻塞关键路径） */
}

#else  /* ======================================================================
        *                              接 收 端
        * ====================================================================== */

volatile uint32_t p2p_rx_ok       = 0u;   /* 收到且三项校验全过 */
volatile uint32_t p2p_rx_bad_data = 0u;   /* magic 或校验和错 */
volatile uint32_t p2p_rx_bad_seq  = 0u;   /* 序号不连续 */
volatile uint32_t p2p_rx_last_seq = 0u;   /* 最后收到的序号 */

static uint16_t p2p_rx_expect  = 0u;      /* 期望的下一个 seq */
static uint8_t  p2p_rx_started = 0u;      /* 是否已收到过第一包 */

void p2p_init(void)
{
    nrf24_init();
    nrf24_flush_rx();
    nrf24_set_mode_rx();      /* PRIM_RX=1 且 CE 拉高 —— 之后持续接收 */
}

void p2p_poll(void)
{
    uint8_t  buf[P2P_PAYLOAD_LEN];
    uint8_t  st;
    uint16_t sum;
    uint16_t seq;
    uint8_t  i;

    st = nrf24_get_status();
    if ((st & NRF24_STATUS_RX_DR) == 0u) { return; }   /* 没收到包，直接返回 */

    nrf24_read_rx_payload(buf, P2P_PAYLOAD_LEN);
    nrf24_clear_irq(NRF24_STATUS_RX_DR);               /* 清标志，等下一样 */

    /* ---------- 校验 1：magic ---------- */
    if ((buf[2] != 0xA5u) || (buf[3] != 0x5Au))
    {
        p2p_rx_bad_data++;
        return;
    }

    /* ---------- 校验 2：校验和 ---------- */
    sum = 0u;
    for (i = 0u; i < P2P_SUM_LEN; i++) { sum += buf[i]; }
    if ((buf[10] != (uint8_t)(sum & 0xFFu)) || (buf[11] != (uint8_t)(sum >> 8)))
    {
        p2p_rx_bad_data++;
        return;
    }

    /* ---------- 校验 3：seq 连续（第一包不判）---------- */
    seq = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if (p2p_rx_started && (seq != p2p_rx_expect))
    {
        p2p_rx_bad_seq++;
    }

    p2p_rx_ok++;
    p2p_rx_last_seq = seq;
    p2p_rx_expect   = (uint16_t)(seq + 1u);
    p2p_rx_started  = 1u;
}

#endif /* P2P_IS_TX */
