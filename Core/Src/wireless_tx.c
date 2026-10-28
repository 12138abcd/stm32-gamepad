/**
 * @file    wireless_tx.c
 * @brief   手柄端无线发送：队列 + 独立任务（第 6 步，第 7 步加心跳节奏）
 *
 * 1ms 任务把报告塞进队列（深度 2，满则丢最旧）；独立任务取出 -> 组帧 -> nrf24_send。
 * 有线模式每 20ms 只发心跳帧（保活，dongle 收到不写报告）；无线模式每拍发报告帧。
 *
 * @note 为什么用队列 + 独立任务：nrf24_send 阻塞 1~3ms，塞进 1ms 任务会打崩节拍。
 *       心跳为什么 20ms、队列为什么"丢旧保新" —— 详见《固件代码导读》11-§11.3、§11.7。
 */

#include "wireless_tx.h"
#include "proto.h"
#include "nrf24.h"
#include "main.h"
#include "report.h"
#include "app.h"            /* app_is_wired() */
#include "latency_probe.h"  /* 无线探针的终点 */
#include "cmsis_os.h"
#include <string.h>

#define WTX_QUEUE_DEPTH   2u      /* 深度 2 足够：1ms 生产、约 3ms 消费 */
#define WTX_ITEM_LEN      (PROTO_REPORT_LEN + 1u)  /* 9 字节报告 + 1 字节帧类型 */
#define WTX_GET_TIMEOUT   5u      /* 取队列超时：越短，切通道响应越快 */
#define WTX_HEARTBEAT_MS  20u     /* 心跳间隔：50 帧/秒，远小于 dongle 的 100ms 超时 */
#define WTX_PAIR_WINDOW_MS 2000u  /* 第 9 步：上电后前 2 秒发配对帧（让 dongle 记下 dev_id）*/

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
        /* 无线 / LOST：每拍入队。LOST 也要发 —— 不发就永远连不回来。 */
        hb_acc = 0u;
        type   = PROTO_TYPE_REPORT;
    }

    /* 第 9 步：上电后前 2 秒，帧类型改成【配对帧】（让 dongle 记住这个 dev_id）。
       有线模式也发 —— 配对跟走哪条通道无关。 */
    if (HAL_GetTick() < WTX_PAIR_WINDOW_MS)
    {
        type = PROTO_TYPE_PAIR;
    }

    memcpy(item, gamepad_report, PROTO_REPORT_LEN);
    item[PROTO_REPORT_LEN] = type;

    /* 满则丢最旧、保最新：队列几乎总是满的，若"丢新"，
       发出去的就全是几毫秒前的旧报告，按键变化永远传不过去。 */
    if (osMessageQueuePut(txQueue, item, 0u, 0u) != osOK)
    {
        (void)osMessageQueueGet(txQueue, old, NULL, 0u);   /* 丢掉最旧的 */
        if (osMessageQueuePut(txQueue, item, 0u, 0u) != osOK)
        {
            wtx_dropped++;      /* 理论上到不了这里 */
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
        if (osMessageQueueGet(txQueue, item, NULL, WTX_GET_TIMEOUT) != osOK)
        {
            continue;
        }

        type = item[PROTO_REPORT_LEN];

        /* t1 只能"晚一帧"带出去：帧要先组好，而 t1 要等发送完才知道。
           第 9 步：dev_id 换成自己配对到的 ID（不再是固定的 0）。 */
        proto_pack_ex(item, txSeq, app_dev_id(), type, wprobe_t1_out, frame);
        ret = nrf24_send(frame, PROTO_FRAME_LEN, 50u);

        /* 探针终点：TX_OK = 收到 ACK = dongle 已收到，所以空中时间已含在这一段里。
           有线模式下探针没武装，这个调用直接返回。 */
        LatencyProbe_OnWirelessTx(ret);

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
            /* TIMEOUT 比 MAX_RT 更严重（发送状态机根本没动：模块没接好 / SPI 出错）。
               必须按失败计入 —— 否则 SPI 挂掉时 link_up 会永远卡在 1。 */
            wtx_timeout++;
            wtx_link_up = proto_link_on_maxrt();
        }

        wtx_last_seq = txSeq;
        txSeq++;
    }
}
