/**
  ******************************************************************************
  * @file    latency_probe.h
  * @brief   USB 输入延迟测量模块（可选的调试 / 测量装置，不是产品功能）。
  *
  * 打开 LATENCY_PROBE_ENABLE 才有实际代码；关掉时所有函数变成空操作，
  * 一行机器码都不占 —— 所以「产品版」和「测量版」共用同一份源码，不会分叉。
  ******************************************************************************
  */
#ifndef __LATENCY_PROBE_H
#define __LATENCY_PROBE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ============================== 总开关 ==============================
   0 = 产品版：测量代码完全不编译进去，main.c 里的调用点变成空操作
   1 = 测量版：打开全套测量装置（打点脚 + DWT 计时 + 直方图 + 分位数）

   打开后，用 Keil 的 Watch 窗口读这些统计量：
     probe_count       有效样本数（只统计"按下"）
     probe_p50/p95/p99 分位数（单位 µs）
     probe_max_us      最长延迟            probe_min_us   最短延迟
     probe_hist[]      直方图（每桶 10µs，下标 = 延迟 / 10µs）
     probe_hist_over   超出量程（>5000µs）的次数，应该恒为 0
     probe_timeout     武装超时次数，应该恒为 0
     probe_count_skip  被跳过的"松开"样本数（应该 ≈ probe_count）
     probe_fall/rise   打点脚翻转次数（链路自检用）
     probe_clear       写成 1 可在线清零 —— 两轮参数对比实验不用重烧固件
   ****************************************************************** */
#define LATENCY_PROBE_ENABLE   1

/* ---- 第 8 步：手柄侧"上一帧算出的 t1"（µs）----
   下一帧组帧时带出去（proto_pack_ex 的 t1_us 参数）。
   ⚠️ 放在总开关【外面】：关掉探针时它仍然存在（恒 0），
      这样 wireless_tx.c 不用加条件编译。 */
extern volatile uint16_t wprobe_t1_out;

#if LATENCY_PROBE_ENABLE

/* 打点脚：PB10。选它的理由：不占 ADC 通道、不占 SPI1（PA4~PA7 留给无线）、
   不占 USART1（PA9/PA10 留给串口）、不是 JTAG 脚（PA15/PB3/PB4）、不是板载 LED。
   空闲电平 = 高；"输入变化"时拉低、"报告发上总线"时拉高 —— 逻辑分析仪上
   看到的就是一个负脉冲。详细接法与读法见 latency_probe.c 顶部的说明。 */
#define LATENCY_PROBE_PORT   GPIOB
#define LATENCY_PROBE_PIN    GPIO_PIN_10

void LatencyProbe_Init(void);                  /* 在 main() 的 USER CODE 2 里调一次 */
void LatencyProbe_OnTick(void);                /* 主循环里每拍调一次 */
void LatencyProbe_DataInHook(uint8_t epnum);   /* 由 usbd_conf.c 里的宏调用 */

/* ---- 第 8 步：无线延迟测量（两段相加）----
   手柄侧 : 输入原始电平变化 → nrf24_send() 返回 TX_OK（空中时间已含在 ACK 里）
   dongle : 读到 RX_DR（收到包） → USB DataIn 完成
   两段相加 = 端到端；中间只差 ACK 回传（~130µs）。
   统计量是独立一套：wprobe_*（不碰有线那套 probe_*）。 */
void LatencyProbe_OnWirelessTx(uint8_t ret);   /* 手柄侧：一次无线发送完成，ret = NRF24_TX_* */
void LatencyProbe_OnRxPacket(void);            /* dongle 侧：读 FIFO 之前，只记时刻 */
void LatencyProbe_OnRxFrame(uint16_t seq, uint16_t t1_prev_us);
                                               /* dongle 侧：解得报告帧之后，配对 + 开测量 */

#else  /* -------------------- 关闭时：全部退化成空操作 -------------------- */

#define LatencyProbe_Init()               ((void)0)
#define LatencyProbe_OnTick()             ((void)0)
#define LatencyProbe_DataInHook(epnum)    ((void)0)
#define LatencyProbe_OnWirelessTx(ret)    ((void)0)
#define LatencyProbe_OnRxPacket()         ((void)0)
#define LatencyProbe_OnRxFrame(seq, t1)   ((void)0)

#endif /* LATENCY_PROBE_ENABLE */

#endif /* __LATENCY_PROBE_H */
