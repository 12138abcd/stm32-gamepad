/**
  ******************************************************************************
  * @file    proto.c
  * @brief   手柄 <-> dongle 的无线协议实现（无线 A 路线 · 第 5 步）
  *
  * 只做四件事：
  *   ① 组帧（proto_pack）  ② 解帧 + 校验（proto_unpack）
  *   ③ 校验和（proto_checksum）  ④ 断线判据状态机
  *
  * 不碰 nRF24、不碰 USB —— 纯数据变换，方便单测和复用。
  ******************************************************************************
  */

#include "proto.h"
#include <string.h>

/* ============================ 校验和 ============================ */

uint16_t proto_checksum(const uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0u;
    uint8_t  i;

    for (i = 0u; i < len; i++)
    {
        sum = (uint16_t)(sum + buf[i]);
    }
    return sum;
}

/* ============================ 组帧 ============================ */

void proto_pack(const uint8_t report[PROTO_REPORT_LEN], uint16_t seq,
                uint8_t dev_id, uint8_t out[PROTO_FRAME_LEN])
{
    proto_pack_ex(report, seq, dev_id, PROTO_TYPE_REPORT, 0u, out);
}

void proto_pack_ex(const uint8_t report[PROTO_REPORT_LEN], uint16_t seq,
                   uint8_t dev_id, uint8_t type, uint16_t t1_us,
                   uint8_t out[PROTO_FRAME_LEN])
{
    uint16_t sum;

    memset(out, 0, PROTO_FRAME_LEN);

    /* 0-8：报告原样放最前面 */
    memcpy(out, report, PROTO_REPORT_LEN);

    /* 9-10：序号（小端） */
    out[9]  = (uint8_t)(seq & 0xFFu);
    out[10] = (uint8_t)(seq >> 8);

    /* 11：帧类型 */
    out[11] = type;

    /* 12：设备 ID */
    out[12] = dev_id;

    /* 13-14：校验和（对 0~12 累加）—— 【不含 t1】，见 proto.h 的说明 */
    sum = proto_checksum(out, PROTO_SUM_LEN);
    out[13] = (uint8_t)(sum & 0xFFu);
    out[14] = (uint8_t)(sum >> 8);

    /* 15-16：t1（手柄段延迟，第 8 步，单位 µs，小端）*/
    out[PROTO_T1_OFS]      = (uint8_t)(t1_us & 0xFFu);
    out[PROTO_T1_OFS + 1u] = (uint8_t)(t1_us >> 8);

    /* 17-31：已经是 0（memset） */
}

/* ============================ 校验 ============================ */

uint8_t proto_verify(const uint8_t in[PROTO_FRAME_LEN])
{
    uint16_t sum;

    /* 类型必须是已知类型（报告帧 / 心跳帧），否则不是我们的包 */
    if ((in[11] != PROTO_TYPE_REPORT) && (in[11] != PROTO_TYPE_HEARTBEAT))
    {
        return 0u;
    }

    sum = proto_checksum(in, PROTO_SUM_LEN);
    if ((in[13] != (uint8_t)(sum & 0xFFu)) ||
        (in[14] != (uint8_t)(sum >> 8)))
    {
        return 0u;
    }

    return 1u;
}

/* ============================ 解帧 ============================ */

uint8_t proto_unpack(const uint8_t in[PROTO_FRAME_LEN],
                     uint8_t report[PROTO_REPORT_LEN],
                     uint16_t *seq, uint8_t *dev_id)
{
    /* 本函数只认【报告帧】。心跳帧是第 7 步加的保活帧，
       由调用者先用 proto_verify() 校验、再看 in[11] 分流处理。 */
    if (in[11] != PROTO_TYPE_REPORT)
    {
        return 0u;
    }

    /* 校验和 —— 与 proto_verify() 共用同一份逻辑，避免两处判定漂移 */
    if (proto_verify(in) == 0u)
    {
        return 0u;
    }

    /* 校验通过 → 拆字段 */
    memcpy(report, in, PROTO_REPORT_LEN);
    *seq    = (uint16_t)in[9] | ((uint16_t)in[10] << 8);
    *dev_id = in[12];

    return 1u;
}

/* ============================ 断线状态机 ============================ */

static uint8_t  link_up        = 0u;   /* 当前链路状态：1 = 通 */
static uint32_t link_last_ok   = 0u;   /* 最后一次成功的时间（ms） */
static uint8_t  link_maxrt_cnt = 0u;   /* 连续 MAX_RT 次数（手柄侧用） */

void proto_link_reset(void)
{
    link_up        = 0u;
    link_last_ok   = 0u;
    link_maxrt_cnt = 0u;
}

/* 收到 / 发出一个合法包 → 链路恢复 */
uint8_t proto_link_on_ok(uint32_t now_ms)
{
    link_last_ok   = now_ms;
    link_maxrt_cnt = 0u;
    link_up        = 1u;
    return link_up;
}

/* 手柄侧：一次 MAX_RT（没收到硬件 ACK） */
uint8_t proto_link_on_maxrt(void)
{
    if (link_maxrt_cnt < 255u)
    {
        link_maxrt_cnt++;
    }

    if (link_maxrt_cnt >= PROTO_LINK_MAXRT_COUNT)
    {
        link_up = 0u;      /* 连续 10 次没 ACK → 判定断线 */
    }
    return link_up;
}

/* dongle 侧：轮询检查超时（100ms 没收到合法包 → 断线） */
uint8_t proto_link_on_timeout(uint32_t now_ms)
{
    if (link_up && ((uint32_t)(now_ms - link_last_ok) >= PROTO_LINK_TIMEOUT_MS))
    {
        link_up = 0u;
    }
    return link_up;
}

uint8_t proto_link_is_up(void)
{
    return link_up;
}
