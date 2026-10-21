/**
  ******************************************************************************
  * @file    wireless_rx.c
  * @brief   dongle 端无线接收实现（第 6 步）
  *
  * 数据流：
  *   nrf24 收包 ──► proto_unpack（校验）──► gamepad_report ──► send_report()（USB）
  *
  * 两个细节：
  *   ① 断线检测每 1ms 都查（不是只有收到包才查）—— 否则永远检测不到"断了"
  *   ② 收到合法包立刻恢复链路（proto_link_on_ok）
  ******************************************************************************
  */

#include "wireless_rx.h"
#include "proto.h"
#include "nrf24.h"
#include "main.h"
#include "report.h"
#include "cmsis_os.h"
#include <string.h>

volatile uint32_t wrx_ok        = 0u;
volatile uint32_t wrx_bad_type  = 0u;
volatile uint32_t wrx_bad_sum   = 0u;
volatile uint32_t wrx_bad_seq   = 0u;
volatile uint32_t wrx_last_seq  = 0u;
volatile uint32_t wrx_link_lost = 0u;
volatile uint8_t  wrx_link_up   = 0u;
volatile uint32_t wrx_usb_sent  = 0u;

static uint16_t rx_expect  = 0u;   /* 期望的下一个 seq */
static uint8_t  rx_started = 0u;   /* 是否已收到第一帧 */

void wireless_rx_init(void)
{
    nrf24_init();
    nrf24_flush_rx();
    nrf24_set_mode_rx();          /* PRIM_RX=1 + CE 高 */
    proto_link_reset();
}

void wireless_rx_poll(void)
{
    uint8_t  frame[PROTO_FRAME_LEN];
    uint8_t  report[PROTO_REPORT_LEN];
    uint16_t seq;
    uint8_t  dev_id;
    uint8_t  st;
    uint32_t now;
    uint8_t  link_before;

    now = HAL_GetTick();

    /* ---- ① 断线检测（每 1ms 都查）---- */
    link_before = proto_link_is_up();
    wrx_link_up = proto_link_on_timeout(now);
    if (link_before && !wrx_link_up)
    {
        wrx_link_lost++;
    }

    /* ---- ② 没收到包就返回 ---- */
    st = nrf24_get_status();
    if ((st & NRF24_STATUS_RX_DR) == 0u) { return; }

    nrf24_read_rx_payload(frame, PROTO_FRAME_LEN);
    nrf24_clear_irq(NRF24_STATUS_RX_DR);

    /* ---- ③ 解帧 + 校验（分类型统计）---- */
    if (proto_unpack(frame, report, &seq, &dev_id) == 0u)
    {
        if (frame[11] != PROTO_TYPE_REPORT) { wrx_bad_type++; }
        else                                { wrx_bad_sum++;  }
        return;
    }

    /* ---- ④ 序号连续性（第一帧不判）---- */
    if (rx_started && (seq != rx_expect))
    {
        wrx_bad_seq++;
    }
    rx_expect  = (uint16_t)(seq + 1u);
    rx_started = 1u;

    /* ---- ⑤ 关键：把收到的报告写进全局 gamepad_report ---- */
    memcpy(gamepad_report, report, PROTO_REPORT_LEN);

    /* ---- ⑥ 转给 USB（report.c 的 send_report）---- */
    send_report();
    wrx_usb_sent++;

    wrx_ok++;
    wrx_last_seq = seq;
    wrx_link_up  = proto_link_on_ok(now);
}
