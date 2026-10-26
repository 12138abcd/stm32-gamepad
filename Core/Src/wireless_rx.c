/**
 * @file    wireless_rx.c
 * @brief   dongle 端接收：收包 -> 按帧类型分流 -> USB（第 6 / 7 步）
 *
 * 报告帧(0x01) -> 写 gamepad_report -> send_report()
 * 心跳帧(0x02) -> 只更新链路，不写报告、不碰 USB
 * 断线          -> 周期发空闲报告，防"卡键"
 *
 * @note 为什么断线要发空闲报告、心跳为什么要维护 seq、探针的两个打点为什么有先后
 *       —— 详见《固件代码导读》11-§11.7、12-§12.9。
 */

#include "wireless_rx.h"
#include "proto.h"
#include "nrf24.h"
#include "main.h"
#include "report.h"
#include "latency_probe.h"   /* 无线探针的起点 / 配对 */
#include "cmsis_os.h"
#include <string.h>

/* 断线期间每 100ms 补发一次空闲报告：只发一次可能撞上端点忙而被丢；
   周期重发是幂等的（每次都是"全松开"），不会有副作用。 */
#define RX_IDLE_PERIOD_MS   100u

/* 空闲报告：按键全 0、Hat 中立(8)、四轴居中(0x80)、扳机松开(0) */
static const uint8_t idle_report[PROTO_REPORT_LEN] =
    { 0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };

volatile uint32_t wrx_ok        = 0u;
volatile uint32_t wrx_bad_type  = 0u;
volatile uint32_t wrx_bad_sum   = 0u;
volatile uint32_t wrx_bad_seq   = 0u;
volatile uint32_t wrx_last_seq  = 0u;
volatile uint32_t wrx_link_lost = 0u;
volatile uint8_t  wrx_link_up   = 0u;
volatile uint32_t wrx_usb_sent  = 0u;
volatile uint32_t wrx_heartbeat = 0u;
volatile uint32_t wrx_idle_sent = 0u;

static uint16_t rx_expect    = 0u;   /* 期望的下一个 seq */
static uint8_t  rx_started   = 0u;   /* 是否已收到第一帧 */
static uint8_t  idle_first   = 1u;   /* 1 = 本次断线还没发过（进来先立刻发一次） */
static uint32_t idle_last_ms = 0u;

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

    /* 断线检测：每 1ms 都查（不是只在收到包时查，否则永远检测不到"断了"）*/
    link_before = proto_link_is_up();
    wrx_link_up = proto_link_on_timeout(now);
    if (link_before && !wrx_link_up)
    {
        wrx_link_lost++;
    }

    st = nrf24_get_status();
    if ((st & NRF24_STATUS_RX_DR) != 0u)
    {
        /* 探针起点：必须在 read_rx_payload 之前 —— 读 FIFO 要几百 µs，
           放后面会把读包时间算进"收包 -> USB"，把 dongle 段算大。 */
        LatencyProbe_OnRxPacket();

        nrf24_read_rx_payload(frame, PROTO_FRAME_LEN);
        nrf24_clear_irq(NRF24_STATUS_RX_DR);

        /* 校验（类型 + 校验和），按失败原因分类统计 */
        if (proto_verify(frame) == 0u)
        {
            if ((frame[11] != PROTO_TYPE_REPORT) && (frame[11] != PROTO_TYPE_HEARTBEAT))
            {
                wrx_bad_type++;
            }
            else
            {
                wrx_bad_sum++;
            }
        }
        else if (frame[11] == PROTO_TYPE_HEARTBEAT)
        {
            /* 心跳帧：只更新链路与序号 —— 这一步就是"有线模式下 PC 上只有一个设备在动"的关键 */
            seq = (uint16_t)frame[9] | ((uint16_t)frame[10] << 8);
            if (rx_started && (seq != rx_expect))
            {
                wrx_bad_seq++;      /* 心跳也占 seq，同样要维护期望值 */
            }
            rx_expect  = (uint16_t)(seq + 1u);
            rx_started = 1u;

            wrx_heartbeat++;
            wrx_link_up = proto_link_on_ok(now);
        }
        else if (proto_unpack(frame, report, &seq, &dev_id) != 0u)
        {
            /* 报告帧：正常路径 */
            if (rx_started && (seq != rx_expect))
            {
                wrx_bad_seq++;
            }
            rx_expect  = (uint16_t)(seq + 1u);
            rx_started = 1u;

            /* 探针配对：必须在 send_report() 之前 —— 先开测量，DataIn 才会把 t2 收尾。
               帧里带的是【上一帧】的 t1（见 proto.h 的 PROTO_T1_OFS）。 */
            LatencyProbe_OnRxFrame(seq,
                (uint16_t)((uint16_t)frame[PROTO_T1_OFS] |
                           ((uint16_t)frame[PROTO_T1_OFS + 1u] << 8)));

            memcpy(gamepad_report, report, PROTO_REPORT_LEN);
            send_report();
            wrx_usb_sent++;

            wrx_ok++;
            wrx_last_seq = seq;
            wrx_link_up  = proto_link_on_ok(now);
        }
        else
        {
            /* proto_verify 过、unpack 却失败：理论上到不了（unpack 只多判一次类型）*/
            wrx_bad_sum++;
        }
    }

    /* 断线 -> 发空闲报告，防卡键（等于替玩家松手）。
       断线那一刻若玩家正推着摇杆，不发这个，PC 上就会一直卡在那个方向。 */
    if (wrx_link_up)
    {
        idle_first = 1u;        /* 恢复后重置：下次断线立刻发一次 */
    }
    else if (idle_first || ((uint32_t)(now - idle_last_ms) >= RX_IDLE_PERIOD_MS))
    {
        idle_first   = 0u;
        idle_last_ms = now;
        memcpy(gamepad_report, idle_report, PROTO_REPORT_LEN);
        send_report();
        wrx_idle_sent++;
    }
}
