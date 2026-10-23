/**
  ******************************************************************************
  * @file    wireless_tx.c
  * @brief   手柄端无线发送实现（第 6 步，第 7 步加通道节奏）
  *
  * 数据流：
  *   1ms 任务：gamepad_report ──► 队列（深度 2，满了丢最旧）
  *              · 无线模式：每拍入队，帧类型 = 报告帧
  *              · 有线模式：每 20ms 入队一次，帧类型 = 心跳帧
  *   独立任务：队列 ──► proto_pack_ex ──► nrf24_send ──► 统计
  *
  * 为什么队列深度只有 2：1ms 生产、约 2~3ms 消费，最多积压 1 帧；
  * 深度 2 足够，且满了丢最旧的不影响"最新状态"。
  *
  * ============================ 第 7 步：心跳 ============================
  * 有线模式下不彻底停发无线，而是每 20ms 发一帧【心跳帧】：
  *   - 目的只有一个：让 dongle 的 100ms 超时不触发，链路保持"热备"；
  *   - 拔线瞬间切回满速，dongle 那边不需要重新 link，切换没有空窗；
  *   - 心跳帧类型是 PROTO_TYPE_HEARTBEAT，dongle 收到后【不】写 gamepad_report、
  *     【不】发 USB —— 所以有线模式下 PC 上只有一个设备在动。
  ******************************************************************************
  */

#include "wireless_tx.h"
#include "proto.h"
#include "nrf24.h"
#include "main.h"
#include "report.h"
#include "app.h"            /* app_is_wired()：第 7 步的通道判据 */
#include "cmsis_os.h"
#include <string.h>

#define WTX_QUEUE_DEPTH   2u      /* 队列深度 */
#define WTX_ITEM_LEN      (PROTO_REPORT_LEN + 1u)  /* 9 字节报告 + 1 字节帧类型 */
#define WTX_GET_TIMEOUT   5u      /* 取队列超时（ms）：从 10 降到 5，切通道响应更快 */
#define WTX_HEARTBEAT_MS  20u     /* 有线模式心跳间隔：20ms = 50 帧/秒，远小于 100ms 超时 */

static osMessageQueueId_t txQueue = NULL;
static uint16_t           txSeq   = 0u;
static uint16_t           hb_acc  = 0u;   /* 有线模式下的心跳累计拍数 */

volatile uint32_t wtx_sent      = 0u;
volatile uint32_t wtx_ok        = 0u;
volatile uint32_t wtx_maxrt     = 0u;
volatile uint32_t wtx_dropped   = 0u;
volatile uint32_t wtx_heartbeat = 0u;
volatile uint32_t wtx_timeout   = 0u;
volatile uint16_t wtx_last_seq  = 0u;
volatile uint8_t  wtx_link_up   = 0u;

void wireless_tx_init(void)
{
    nrf24_init();
    proto_link_reset();

    txQueue = osMessageQueueNew(WTX_QUEUE_DEPTH, WTX_ITEM_LEN, NULL);
}

void wireless_tx_on_tick(void)
{
    uint8_t item[WTX_ITEM_LEN];
    uint8_t old[WTX_ITEM_LEN];
    uint8_t type;

    if (txQueue == NULL) { return; }

    /* ---- 第 7 步：按通道决定"这一拍要不要入队、入什么类型" ---- */
    if (app_is_wired())
    {
        /* 有线：每 20ms 一帧心跳，只为保活 */
        hb_acc++;
        if (hb_acc < WTX_HEARTBEAT_MS) { return; }
        hb_acc = 0u;
        type   = PROTO_TYPE_HEARTBEAT;
    }
    else
    {
        /* 无线（或两条都不通）：每拍入队，帧类型 = 报告帧。
           注意 LOST 状态也要照常入队 —— 不发就永远连不回来。 */
        hb_acc = 0u;
        type   = PROTO_TYPE_REPORT;
    }

    memcpy(item, gamepad_report, PROTO_REPORT_LEN);
    item[PROTO_REPORT_LEN] = type;

    /* 先塞最新报告；如果队列满，就把最旧的那帧挤出去（丢旧、保新）。
       为什么必须保新：1ms 生产、约 3ms 消费（nrf24_send 等 ACK），队列永远是满的；
       如果"丢新"，发出去的就全是几毫秒前的旧报告 —— 按键变化永远传不过去。 */
    if (osMessageQueuePut(txQueue, item, 0u, 0u) != osOK)
    {
        (void)osMessageQueueGet(txQueue, old, NULL, 0u);   /* 丢掉最旧的 */
        if (osMessageQueuePut(txQueue, item, 0u, 0u) != osOK)
        {
            wtx_dropped++;      /* 理论上不会走到这里 */
        }
    }
}

void wireless_tx_task(void *argument)
{
    uint8_t item[WTX_ITEM_LEN];
    uint8_t frame[PROTO_FRAME_LEN];
    uint8_t ret;
    uint8_t type;

    (void)argument;

    osDelay(500u);          /* 等 dongle 上电 + 进接收模式 */

    for (;;)
    {
        /* 等一帧（最多 5ms） */
        if (osMessageQueueGet(txQueue, item, NULL, WTX_GET_TIMEOUT) != osOK)
        {
            continue;
        }

        type = item[PROTO_REPORT_LEN];

        /* 组帧 + 发送（心跳帧与报告帧走同一条路，只是帧类型不同） */
        proto_pack_ex(item, txSeq, PROTO_DEV_ID_UNPAIRED, type, frame);
        ret = nrf24_send(frame, PROTO_FRAME_LEN, 50u);

        wtx_sent++;
        if (type == PROTO_TYPE_HEARTBEAT) { wtx_heartbeat++; }

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
        else
        {
            /* TIMEOUT：发送状态机根本没动，比 MAX_RT 更严重（模块没接好 / SPI 出错）。
               ⚠️ 第 6 步这里是空的 —— 若当时 link_up 已经是 1，就会永远卡在 1，
               而第 7 步要拿链路状态当判据，所以必须计入（Q3 的修复）。 */
            wtx_timeout++;
            wtx_link_up = proto_link_on_maxrt();
        }

        wtx_last_seq = txSeq;
        txSeq++;
    }
}
