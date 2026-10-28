/**
  ******************************************************************************
  * @file    proto.h
  * @brief   手柄 <-> dongle 的无线协议（无线 A 路线 · 第 5 步）
  *
  * ============================ 帧格式（32 字节）============================
  *   偏移   长度   内容
  *   0-8    9     HID 报告（原样，dongle 收到后直接 memcpy 发 USB）
  *   9-10   2     序号 seq（uint16，每包 +1）
  *   11     1     帧类型（0x01 = 报告帧 / 0x02 = 心跳帧；心跳是第 7 步加的）
  *   12     1     设备 ID（0 = 未配对；1~255 = 手柄 ID，为第 6 步配对预留）
  *   13-14  2     校验和（对 0~12 逐字节累加，小端）
  *   15-31  17    补 0
  *
  * ⚠️ 帧长必须 = nrf24_init() 里的 RX_PW_P0（32），
  *    Enhanced ShockBurst 要求收发双方长度一致。
  *
  * ============================ 断线判据（§5.8.3）============================
  *   dongle 侧：100ms 没收到合法包 → 断线
  *   手柄  侧：连续 10 次 nrf24_send 返回 MAX_RT → 断线
  *   恢复    ：收到 / 成功发一包 → 立刻恢复
  ******************************************************************************
  */
#ifndef __PROTO_H
#define __PROTO_H

#include <stdint.h>

/* ============================ 常量 ============================ */
#define PROTO_FRAME_LEN       32u    /* 帧长，必须 = RX_PW_P0 */
#define PROTO_REPORT_LEN      9u     /* HID 报告长度 */

#define PROTO_TYPE_REPORT     0x01u  /* 帧类型：报告帧（dongle 写 gamepad_report + 发 USB） */
#define PROTO_TYPE_HEARTBEAT  0x02u  /* 帧类型：心跳（第 7 步；有线模式下每 20ms 一帧保活，
                                         dongle 只更新链路状态，【不】写报告、【不】发 USB） */
#define PROTO_TYPE_PAIR       0x03u  /* 帧类型：配对帧（第 9 步；手柄上电后前 2 秒发，
                                         dongle 在 10 秒学习窗口内记住它的 dev_id） */
#define PROTO_DEV_ID_UNPAIRED 0x00u  /* 设备 ID：0 = 未配对（dongle 侧表示"学习模式"）*/

/* 校验和覆盖范围：前 13 字节（0~12） */
#define PROTO_SUM_LEN         13u

/* ---- 第 8 步：t1 字段（手柄段延迟）----
   偏移 15~16，uint16 小端，单位 µs（最大 65.5ms，够用）。
   放在原来的预留区里，**不扩大校验和范围** ——
   完整性由 nRF24 的 2 字节 CRC 兜底；改校验和范围会动到已经验收过的第 5 步协议。 */
#define PROTO_T1_OFS          15u

/* ============================ 断线判据 ============================ */
#define PROTO_LINK_TIMEOUT_MS   100u  /* dongle：100ms 没收到合法包 → 断线 */
#define PROTO_LINK_MAXRT_COUNT  10u   /* 手柄：连续 10 次 MAX_RT → 断线 */

/* ============================ 组帧 / 解帧 ============================ */

/**
 * 手柄侧：把 9 字节 HID 报告打成 32 字节协议帧（帧类型固定为 REPORT）。
 * @param report  9 字节报告
 * @param seq     序号
 * @param dev_id  设备 ID（0 = 未配对）
 * @param out     输出的 32 字节帧
 */
void proto_pack(const uint8_t report[PROTO_REPORT_LEN], uint16_t seq,
                uint8_t dev_id, uint8_t out[PROTO_FRAME_LEN]);

/**
 * 同 proto_pack，但可以指定帧类型（第 7 步）和 t1（第 8 步）。
 * proto_pack() 就是 type = REPORT、t1 = 0 的特例。
 *
 * @param t1_us  手柄段延迟（输入变化 → TX_OK），单位 µs；不带这个信息时传 0。
 *               放帧的偏移 15~16，不参与校验和（靠 nRF24 的 CRC 兜底）。
 */
void proto_pack_ex(const uint8_t report[PROTO_REPORT_LEN], uint16_t seq,
                   uint8_t dev_id, uint8_t type, uint16_t t1_us,
                   uint8_t out[PROTO_FRAME_LEN]);

/**
 * 只校验、不拆字段：帧类型是已知类型 + 校验和正确 → 返回 1。
 * 给第 7 步的 dongle 用：先 verify，再按 frame[11] 分流（报告帧 / 心跳帧）。
 * 这样 proto_unpack() 可以保持"只认真报告帧"的原语义，老测试代码不用动。
 */
uint8_t proto_verify(const uint8_t in[PROTO_FRAME_LEN]);

/**
 * dongle 侧：解帧 + 校验（只接受 REPORT 帧，保持第 5 步原语义）。
 * @return 1 = 合法；0 = 非法（类型不对或校验和错）
 */
uint8_t proto_unpack(const uint8_t in[PROTO_FRAME_LEN],
                     uint8_t report[PROTO_REPORT_LEN],
                     uint16_t *seq, uint8_t *dev_id);

/** 校验和：对 buf[0..len-1] 逐字节累加 */
uint16_t proto_checksum(const uint8_t *buf, uint8_t len);

/* ============================ 断线状态机 ============================ */

void    proto_link_reset(void);                    /* 重新开始统计 */
uint8_t proto_link_on_ok(uint32_t now_ms);         /* 收到/发出合法包 → 调用；返回链路状态 */
uint8_t proto_link_on_maxrt(void);                 /* 手柄侧：一次 MAX_RT；返回链路状态 */
uint8_t proto_link_on_timeout(uint32_t now_ms);    /* dongle 侧：轮询检查；返回链路状态 */
uint8_t proto_link_is_up(void);                    /* 只读当前状态 */

#endif /* __PROTO_H */
