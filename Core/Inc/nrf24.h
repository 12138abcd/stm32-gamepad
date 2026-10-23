/**
  ******************************************************************************
  * @file    nrf24.h
  * @brief   nRF24L01+ 底层驱动（无线 A 路线 · 第 3 步）
  *
  * 这一层只做最基础的事：读 / 写寄存器、读写 FIFO、切收发模式、发一包。
  * 【不含】协议（帧格式 / 序号 / 重传策略 / 配对）—— 那是第 5 步的事。
  *
  * ============================ 硬件与引脚 ============================
  * （2026-10-13 定稿；SPI 链路已于第 2 步验证通过：STATUS = 0x0E）
  *     SCK  = PA5  ┐
  *     MISO = PA6  ├ 硬件 SPI1，8 位、MSB First、Mode 0（CPOL Low + CPHA 1Edge）
  *     MOSI = PA7  ┘ 预分频 64 ≈ 1.125 MBits/s
  *     CSN  = PA4      软件片选（普通 GPIO）
  *     CE   = PB3      （JTAG 已被 __HAL_AFIO_REMAP_SWJ_NOJTAG 禁用，SWD 下载不受影响）
  *     VCC  = 3.3V  ⛔ 不是 5V，且就近并 10µF + 100nF
  *     GND  = 与蓝板共地
  *     IRQ  = 悬空（本驱动用轮询读 STATUS，不依赖 IRQ 引脚）
  *
  * ⚠️ 本驱动直接用 CubeMX 生成的 hspi1 与 CSN_Pin / CE_Pin，
  *    自己【不】初始化 SPI 和 GPIO —— 那些都在 PS.ioc 里，
  *    重复初始化只会让代码与 .ioc 不一致（第 2 步的注释踩过这个坑，已纠正）。
  *
  * ⚠️ nRF24 是【纯收发器、板载无 MCU】，单块模块无法完成"收到自己发的字节"
  *    这种自环测试（发送时接收机不工作）。所以"收发一帧、收发一致"必须两块板，
  *    属第 4 步；第 3 步单板能证明的是——
  *      ① 寄存器读写回环正确（SPI 命令格式对）
  *      ② FIFO 通路正确（命令集对）
  *      ③ 发送状态机真的在跑（CE 脉冲触发了发射，重传/完成标志会动）
  *      ④ 能切进接收模式（RX_P_NO / RX FIFO 状态对）
  *    自检代码在 nrf24_test.c，与驱动分开。
  ******************************************************************************
  */
#ifndef __NRF24_H
#define __NRF24_H

#include <stdint.h>

/* ============================ SPI 命令 ============================
   命令字节 = bit7~5 命令 + bit4~0 寄存器地址 */
#define NRF24_CMD_R_REGISTER     0x00u
#define NRF24_CMD_W_REGISTER     0x20u
#define NRF24_CMD_R_RX_PAYLOAD   0x61u
#define NRF24_CMD_W_TX_PAYLOAD   0xA0u
#define NRF24_CMD_FLUSH_TX       0xE1u
#define NRF24_CMD_FLUSH_RX       0xE2u
#define NRF24_CMD_REUSE_TX_PL    0xE3u
#define NRF24_CMD_NOP            0xFFu   /* 空操作；当占位字节用，它同时把 STATUS 回送到 MISO */

/* ============================ 寄存器地址 ============================ */
#define NRF24_REG_CONFIG        0x00u   /* 配置（PWR_UP / PRIM_RX / CRC / 中断屏蔽） */
#define NRF24_REG_EN_AA         0x01u   /* 各 Pipe 的自动应答使能 */
#define NRF24_REG_EN_RXADDR     0x02u   /* 各 Pipe 的接收使能 */
#define NRF24_REG_SETUP_AW      0x03u   /* 地址宽度 */
#define NRF24_REG_SETUP_RETR    0x04u   /* 自动重传：间隔 + 次数 */
#define NRF24_REG_RF_CH         0x05u   /* 射频信道 */
#define NRF24_REG_RF_SETUP      0x06u   /* 速率 + 发射功率 */
#define NRF24_REG_STATUS        0x07u   /* 状态（写 1 清标志） */
#define NRF24_REG_OBSERVE_TX    0x08u   /* 发送观测：丢包计数 / 重传计数 */
#define NRF24_REG_RPD           0x09u   /* 接收功率检测（bit0：收到 > -64dBm 的载波） */
#define NRF24_REG_RX_ADDR_P0    0x0Au   /* Pipe0 接收地址（5 字节） */
#define NRF24_REG_TX_ADDR       0x10u   /* 发送地址（5 字节） */
#define NRF24_REG_RX_PW_P0      0x11u   /* Pipe0 有效载荷长度 */
#define NRF24_REG_FIFO_STATUS   0x17u   /* FIFO 状态 */

/* ============================ 位定义 ============================ */
/* STATUS（0x07）—— 这三位写 1 清除 */
#define NRF24_STATUS_TX_FULL    0x01u   /* TX FIFO 满 */
#define NRF24_STATUS_MAX_RT     0x10u   /* 重传次数用尽（对端没应答） */
#define NRF24_STATUS_TX_DS      0x20u   /* 发送完成（收到 ACK） */
#define NRF24_STATUS_RX_DR      0x40u   /* 收到数据 */
#define NRF24_STATUS_IRQ_MASK   0x70u   /* RX_DR | TX_DS | MAX_RT */
#define NRF24_STATUS_RX_P_NO    0x0Eu   /* bit3~1：RX FIFO 里当前包的 Pipe 号；111 = FIFO 空 */

/* CONFIG（0x00） */
#define NRF24_CONFIG_PRIM_RX    0x01u   /* 1 = 接收模式，0 = 发送模式 */
#define NRF24_CONFIG_PWR_UP     0x02u   /* 1 = 上电 */
#define NRF24_CONFIG_CRCO       0x04u   /* 1 = 2 字节 CRC，0 = 1 字节 */
#define NRF24_CONFIG_EN_CRC     0x08u   /* 1 = 使能 CRC */
/* 上电 + 2 字节 CRC + 不屏蔽中断（我们用轮询读 STATUS，屏蔽与否不影响） */
#define NRF24_CONFIG_STANDBY    0x0Eu

/* FIFO_STATUS（0x17） */
#define NRF24_FIFO_RX_EMPTY     0x01u
#define NRF24_FIFO_RX_FULL      0x02u
#define NRF24_FIFO_TX_EMPTY     0x10u
#define NRF24_FIFO_TX_FULL      0x20u

/* ============================ 参数 ============================ */
#define NRF24_ADDR_WIDTH        5u      /* SETUP_AW = 0x03 */
#define NRF24_PAYLOAD_LEN       32u     /* nRF24 单包上限 32 字节 */
#define NRF24_RF_CH_DEFAULT     0x4Cu   /* 信道 76 → 2.476 GHz（避开 WiFi 常用的 1 / 6 / 11 段） */
#define NRF24_RF_SETUP_1M_0DBM  0x06u   /* RF_PWR=11（0dBm）、1 Mbps */

/* nrf24_send() 的返回值 */
#define NRF24_TX_OK             0u      /* TX_DS：收到 ACK —— 说明对端在（第 4 步的期望结果） */
#define NRF24_TX_MAXRT          1u      /* MAX_RT：重传耗尽 —— 单板无对端时的【正常】结果 */
#define NRF24_TX_TIMEOUT        2u      /* 超时：状态机根本没动，属异常 */

/* ============================ 对本文件外可见的运行期变量（Keil Watch 观察） ============================ */
extern volatile uint32_t nrf24_err;          /* SPI 传输出错的次数，正常恒 0 */
extern volatile uint8_t  nrf24_last_status;  /* 最近一次 SPI 事务里模块回送的 STATUS */

/* ---- 初始化自诊断（2026-10-23 加，治"下载后第一次连不上无线"）----
   判读：
     nrf24_boot_retry  = 0  → 一次就成功（正常）
     nrf24_boot_retry  > 0  → 重试过，但最终起来了
     nrf24_boot_retry  = NRF24_BOOT_RETRY_MAX → 模块一直没应答，没配置上
     nrf24_boot_config = 0x0E → 正常；0x00 / 0xFF → SPI 根本没通 */
extern volatile uint8_t  nrf24_boot_retry;
extern volatile uint8_t  nrf24_boot_config;

/* 软复位最多重试几次（每轮 = 写 0x08 + 2ms + 写 0x0E + 2ms + 读回校验） */
#define NRF24_BOOT_RETRY_MAX   3u

/* 初始化前先等多久（ms）让模块完成上电复位。
   为什么需要：下载代码时 ST-Link 的 Flash 编程电流会把 3.3V 拉一下，
   nRF24 跟着掉电；重新上电后它要 ~10ms 才响应 SPI，这段时间写的寄存器全被忽略。
   现取 20ms（留一倍裕量）。 */
#define NRF24_BOOT_POR_MS      20u

/* CE 脉冲宽度（µs）的默认值：200。
   数据手册上"CE ≥10µs"是【PLL 已锁定】的前提；而模块从 standby 真正进入 TX
   模式还要 Tstby2a = 130µs —— 所以默认取 200，留出裕量。 */
#define NRF24_CE_PULSE_DEFAULT   200u

/* CE 脉冲宽度（µs）—— 发送参数。正常情况用默认值；
   自检会临时改它来做"宽度扫描"（见 nrf24_test.c 的 ④）。 */
extern volatile uint16_t nrf24_ce_pulse_us;

/* 本项目的 Pipe0 收发地址（手柄与 dongle 共用同一份，第 4 步点对点直接用） */
extern const uint8_t nrf24_addr_p0[NRF24_ADDR_WIDTH];

/* ============================ API ============================ */
void    nrf24_init(void);                    /* 上电初始化；在 main() 的 USER CODE 2 里调一次 */

uint8_t nrf24_read_reg(uint8_t reg);
void    nrf24_write_reg(uint8_t reg, uint8_t val);
void    nrf24_read_reg_multi(uint8_t reg, uint8_t *buf, uint8_t len);        /* 多字节（地址类，最长 5） */
void    nrf24_write_reg_multi(uint8_t reg, const uint8_t *buf, uint8_t len);

uint8_t nrf24_get_status(void);
void    nrf24_clear_irq(uint8_t flags);      /* 写 1 清 STATUS 里的标志位 */
void    nrf24_flush_tx(void);
void    nrf24_flush_rx(void);

void    nrf24_write_tx_payload(const uint8_t *buf, uint8_t len);   /* 填 TX FIFO（CE 低时调用） */
void    nrf24_read_rx_payload(uint8_t *buf, uint8_t len);          /* 取 RX FIFO 里的一包 */

void    nrf24_set_mode_tx(void);             /* 待机（发送模式），CE 保持低 */
void    nrf24_set_mode_rx(void);             /* 接收模式，内部会把 CE 拉高 */

/* 发一包并等结果（阻塞，最长 timeout_ms）。返回值见上面的 NRF24_TX_*。
   ⚠️ 无对端时它会等满重传（ARD=500µs × 3 次 ≈ 2ms），所以【不要】放进 1ms 关键路径，
      第 6 步集成时要挪到独立任务里。 */
uint8_t nrf24_send(const uint8_t *buf, uint8_t len, uint32_t timeout_ms);

void    nrf24_ce_low(void);
void    nrf24_ce_high(void);
void    nrf24_delay_us(uint32_t us);         /* 基于 DWT 的微秒级忙等（CE 脉冲要 ≥10µs） */

#endif /* __NRF24_H */
