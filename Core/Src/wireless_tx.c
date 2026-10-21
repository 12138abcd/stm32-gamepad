/**
  ******************************************************************************
  * @file    wireless_tx.c
  * @brief   手柄端无线发送实现（第 6 步）
  *
  * 数据流：
  *   1ms 任务：gamepad_report ──► 队列（深度 2，满了丢弃）
  *   独立任务：队列 ──► proto_pack ──► nrf24_send ──► 统计
  *
  * 为什么队列深度只有 2：1ms 生产、2ms 消费，最多积压 1 帧；
  * 深度 2 足够，且满了丢最旧的不影响"最新状态"。
  ******************************************************************************
  */

#include "wireless_tx.h"
#include "proto.h"
#include "nrf24.h"
#include "main.h"
#include "report.h"
#include "cmsis_os.h"
#include <string.h>

#define WTX_QUEUE_DEPTH   2u      /* 队列深度 */
#define WTX_GET_TIMEOUT   10u     /* 取队列超时（ms），防止永久阻塞 */

static osMessageQueueId_t txQueue = NULL;
static uint16_t           txSeq   = 0u;

volatile uint32_t wtx_sent      = 0u;
volatile uint32_t wtx_ok        = 0u;
volatile uint32_t wtx_maxrt     = 0u;
volatile uint32_t wtx_dropped   = 0u;
volatile uint16_t wtx_last_seq  = 0u;
volatile uint8_t  wtx_link_up   = 0u;

void wireless_tx_init(void)
{
    nrf24_init();
    proto_link_reset();

    txQueue = osMessageQueueNew(WTX_QUEUE_DEPTH, PROTO_REPORT_LEN, NULL);
}

void wireless_tx_on_tick(void)
{
    uint8_t old[PROTO_REPORT_LEN];

    if (txQueue == NULL) { return; }

    /* 先塞最新报告；如果队列满，就把最旧的那帧挤出去（丢旧、保新）。
       为什么必须保新：1ms 生产、3ms 消费（nrf24_send 等 ACK），队列永远是满的；
       如果"丢新"，发出去的就全是几毫秒前的旧报告 —— 按键变化永远传不过去。 */
    if (osMessageQueuePut(txQueue, gamepad_report, 0u, 0u) != osOK)
    {
        (void)osMessageQueueGet(txQueue, old, NULL, 0u);   /* 丢掉最旧的 */
        if (osMessageQueuePut(txQueue, gamepad_report, 0u, 0u) != osOK)
        {
            wtx_dropped++;      /* 理论上不会走到这里 */
        }
    }
}

void wireless_tx_task(void *argument)
{
    uint8_t report[PROTO_REPORT_LEN];
    uint8_t frame[PROTO_FRAME_LEN];
    uint8_t ret;

    (void)argument;

    osDelay(500u);          /* 等 dongle 上电 + 进接收模式 */

    for (;;)
    {
        /* 等一帧报告（最多 10ms） */
        if (osMessageQueueGet(txQueue, report, NULL, WTX_GET_TIMEOUT) != osOK)
        {
            continue;
        }

        /* 组帧 + 发送 */
        proto_pack(report, txSeq, PROTO_DEV_ID_UNPAIRED, frame);
        ret = nrf24_send(frame, PROTO_FRAME_LEN, 50u);

        wtx_sent++;
        if (ret == NRF24_TX_OK)
        {
            wtx_ok++;
            wtx_link_up = proto_link_on_ok(HAL_GetTick());
        }
        else if (ret == NRF24_TX_MAXRT)
        {
            wtx_maxrt++;
            wtx_link_up = proto_link_on_maxrt();
        }

        wtx_last_seq = txSeq;
        txSeq++;
    }
}
