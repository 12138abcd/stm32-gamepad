/**
  ******************************************************************************
  * @file    proto_test.c
  * @brief   第 5 步协议测试：把 gamepad_report 用 proto 帧发出去 / 收回来
  *
  * 角色由 proto.h 里的 PROTO_IS_TX 决定：
  *     1 = 发送端（每 10ms 发一帧，统计 ACK / 断线）
  *     0 = 接收端（每 1ms 轮询，解帧 + 校验 + 断线检测）
  *
  * 与 p2p.c 的区别：p2p 是"裸 32 字节测试帧"，这里是"带语义的协议帧"
  *   —— 报告在前、seq/类型/设备ID/校验和在后，dongle 收到可以直接转发 USB。
  *
  * ⚠️ 发送端是阻塞的（无对端时等满重传 ≈3ms），必须放独立任务，不能进 1ms 任务。
  ******************************************************************************
  */

#include "proto_test.h"
#include "proto.h"
#include "nrf24.h"
#include "main.h"
#include "report.h"
#include "cmsis_os.h"
#include <string.h>

#if PROTO_IS_TX

/* ============================ 发送端 ============================ */

volatile uint32_t proto_tx_sent  = 0u;   /* 已发帧数 */
volatile uint32_t proto_tx_ok    = 0u;   /* 收到硬件 ACK 的次数 */
volatile uint32_t proto_tx_maxrt = 0u;   /* 重传耗尽次数 */
volatile uint16_t proto_last_seq = 0u;   /* 最后发出的序号 */
volatile uint8_t  proto_link_up  = 0u;   /* 链路状态：1 = 通 */

static uint16_t tx_seq = 0u;

static void proto_task(void *argument)
{
    uint8_t frame[PROTO_FRAME_LEN];
    uint8_t ret;

    (void)argument;

    osDelay(500u);          /* 等接收端上电 + 进接收模式 */

    for (;;)
    {
        /* 组帧：报告 + seq + 类型 + 设备ID + 校验和 */
        proto_pack(gamepad_report, tx_seq, PROTO_DEV_ID_UNPAIRED, frame);

        /* 发一帧 */
        ret = nrf24_send(frame, PROTO_FRAME_LEN, 50u);

        proto_tx_sent++;
        if (ret == NRF24_TX_OK)
        {
            proto_tx_ok++;
            proto_link_up = proto_link_on_ok(HAL_GetTick());
        }
        else if (ret == NRF24_TX_MAXRT)
        {
            proto_tx_maxrt++;
            proto_link_up = proto_link_on_maxrt();     /* 连续 10 次 → 断线 */
        }

        proto_last_seq = tx_seq;
        tx_seq++;

        osDelay(10u);       /* 10ms 一帧 ≈ 100 帧/秒 */
    }
}

void proto_test_init(void)
{
    static const osThreadAttr_t attr = {
        .name       = "protoTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t) osPriorityBelowNormal,
    };

    nrf24_init();
    osThreadNew(proto_task, NULL, &attr);
}

void proto_test_poll(void)
{
    /* 发送端：1ms 任务里什么都不做（发送在 proto_task 里） */
}

#else  /* ============================ 接收端 ============================ */

volatile uint32_t proto_rx_ok        = 0u;   /* 收到且校验全过 */
volatile uint32_t proto_rx_bad_type  = 0u;   /* 帧类型错 */
volatile uint32_t proto_rx_bad_sum   = 0u;   /* 校验和错 */
volatile uint32_t proto_rx_bad_seq   = 0u;   /* 序号不连续 */
volatile uint32_t proto_rx_last_seq  = 0u;   /* 最后收到的序号 */
volatile uint8_t  proto_link_up      = 0u;   /* 链路状态：1 = 通 */
volatile uint32_t proto_link_lost    = 0u;   /* 断线次数（状态从通变断的次数） */

static uint16_t rx_expect  = 0u;
static uint8_t  rx_started = 0u;

void proto_test_init(void)
{
    nrf24_init();
    nrf24_flush_rx();
    nrf24_set_mode_rx();       /* PRIM_RX=1 + CE 高 */
    proto_link_reset();
}

void proto_test_poll(void)
{
    uint8_t  frame[PROTO_FRAME_LEN];
    uint8_t  report[PROTO_REPORT_LEN];
    uint16_t seq;
    uint8_t  dev_id;
    uint8_t  st;
    uint32_t now;
    uint8_t  link_before;

    now = HAL_GetTick();

    /* ---- 断线检测：100ms 没收到合法包 → 断线 ---- */
    link_before = proto_link_is_up();
    proto_link_up = proto_link_on_timeout(now);
    if (link_before && !proto_link_up)
    {
        proto_link_lost++;      /* 状态从"通"变"断"记一次 */
    }

    /* ---- 没收到包就返回 ---- */
    st = nrf24_get_status();
    if ((st & NRF24_STATUS_RX_DR) == 0u) { return; }

    nrf24_read_rx_payload(frame, PROTO_FRAME_LEN);
    nrf24_clear_irq(NRF24_STATUS_RX_DR);

    /* ---- 解帧 + 校验（分类型统计，方便排查）---- */
    if (proto_unpack(frame, report, &seq, &dev_id) == 0u)
    {
        if (frame[11] != PROTO_TYPE_REPORT) { proto_rx_bad_type++; }
        else                                { proto_rx_bad_sum++;  }
        return;
    }

    /* ---- 序号连续性（第一帧不判）---- */
    if (rx_started && (seq != rx_expect))
    {
        proto_rx_bad_seq++;
    }

    proto_rx_ok++;
    proto_rx_last_seq = seq;
    rx_expect         = (uint16_t)(seq + 1u);
    rx_started        = 1u;

    /* ---- 收到合法包 → 链路恢复 ---- */
    proto_link_up = proto_link_on_ok(now);
}

#endif /* PROTO_IS_TX */
