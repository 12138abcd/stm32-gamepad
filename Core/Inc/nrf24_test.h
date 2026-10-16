/**
  ******************************************************************************
  * @file    nrf24_test.h
  * @brief   nRF24L01 最小验证 —— 无线 A 路线 · 第 2 步（只证明 SPI 通了）
  *
  * 目的：读到 nRF24 的 STATUS 寄存器，值不是 0x00 也不是 0xFF。
  *       过了这一关，才谈得上收发。
  *
  * 引脚（2026-10-13 定稿）：
  *     SCK  = PA5   ┐
  *     MISO = PA6   ├ 硬件 SPI1
  *     MOSI = PA7   ┘
  *     CSN  = PA4      软件片选
  *     CE   = PB3      （JTAG 已由 CubeMX 的 __HAL_AFIO_REMAP_SWJ_NOJTAG() 禁用，
  *                      所以 PB3 可当普通 GPIO；SWD 下载调试不受影响）
  *     VCC  = 3.3V ⚠️ 不是 5V，且就近并 10µF + 100nF
  *     GND  = GND
  *     IRQ  = 悬空（用轮询读 STATUS 就够）
  *
  * ⚠️ 本文件是【验证用】：
  *     不依赖 CubeMX 的 SPI 配置（自己初始化 GPIO + SPI1），
  *     所以不需要重新 GENERATE CODE，也就不会触发那套"端点宏被冲 + 注释乱码"的修复流程。
  *     跑通之后再决定要不要正规化进 CubeMX。
  ******************************************************************************
  */
#ifndef __NRF24_TEST_H
#define __NRF24_TEST_H

#include <stdint.h>

/* 上电调一次：配置 GPIO（PA4/PA5/PA6/PA7、PB3）+ SPI1 */
void nrf24_test_init(void);

/* 在 1ms 任务里调；内部做了 100ms 节流，不会占用关键路径 */
void nrf24_test_poll(void);

/* ------- Watch 窗口观察这几个（都在 .c 里定义） ------- */
extern volatile uint32_t nrf24_status;      /* 最近一次读到的 STATUS（期望 ≈0x0E，不能是 0x00/0xFF）*/
extern volatile uint8_t  nrf24_status_b;    /* 紧接着再读一次，用于交叉验证（两次应相同）*/
extern volatile uint32_t nrf24_reads;       /* 一共读了多少次 */
extern volatile uint32_t nrf24_err;         /* SPI 传输超时/出错的次数（正常应为 0）*/

#endif /* __NRF24_TEST_H */
