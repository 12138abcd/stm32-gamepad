/**
  ******************************************************************************
  * @file    nrf24.c
  * @brief   nRF24L01+ 底层驱动实现（无线 A 路线 · 第 3 步）。接口与判据见 nrf24.h。
  ******************************************************************************
  */

#include "nrf24.h"
#include "main.h"          /* hspi1 / CSN_Pin / CE_Pin / 端口宏 / DWT / HAL_Delay */
#include <string.h>        /* memcpy */

extern SPI_HandleTypeDef hspi1;   /* 定义在 main.c（CubeMX 生成），不重复初始化 */

/* ============================ 运行期统计（Watch 可见） ============================ */
volatile uint32_t nrf24_err         = 0u;   /* SPI 传输出错次数，正常恒 0 */
volatile uint8_t  nrf24_last_status = 0u;   /* 最近一次事务里模块回送的 STATUS */

/* 初始化自诊断（2026-10-23）：下载后第一次启动无线不通时，先看这两个 */
volatile uint8_t  nrf24_boot_retry  = 0u;   /* 软复位重试次数；0 = 一次成功 */
volatile uint8_t  nrf24_boot_config = 0u;   /* 初始化后读回的 CONFIG；正常 = 0x0E */

/* 本项目的 Pipe0 收发地址：手柄与 dongle 用同一份，代码整份复用 */
const uint8_t nrf24_addr_p0[NRF24_ADDR_WIDTH] = { 'P', 'S', 'P', 'A', 'D' };

/* ============================ 内部：SPI 事务缓冲 ============================
   放 static 而不是局部数组 —— 免得每次调用都吃掉调用者几百字节的栈
   （最大的包 32+1 字节；本驱动单线程访问，不存在重入）。 */
static uint8_t nrf24_tx_buf[NRF24_PAYLOAD_LEN + 1u];
static uint8_t nrf24_rx_buf[NRF24_PAYLOAD_LEN + 1u];

/* SPI 超时：全是几字节的小事务，10ms 已经极宽松（正常一次 ≈ 20µs） */
#define NRF24_SPI_TIMEOUT_MS   10u

/* ============================ 内部工具 ============================ */

static void nrf24_csn_low(void)  { HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET); }
static void nrf24_csn_high(void) { HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET); }

/* 一次完整 SPI 事务：CSN 拉低 → 全双工收发 len 字节 → CSN 拉高。
   ⚠️ 命令字节和数据必须在【同一个 CSN 低电平周期】里发完 ——
      中途拉高会被模块当成"这次事务结束"，多字节寄存器 / 负载就只能读到半截。 */
static void nrf24_transfer(uint16_t len)
{
    nrf24_csn_low();
    if (HAL_SPI_TransmitReceive(&hspi1, nrf24_tx_buf, nrf24_rx_buf, len, NRF24_SPI_TIMEOUT_MS) != HAL_OK)
    {
        nrf24_err++;
    }
    nrf24_csn_high();

    /* nRF24 的特性：任何命令字节期间它都会把 STATUS 从 MISO 送回来，
       所以 rx_buf[0] 就是这次事务开始时的 STATUS（白拿的，不额外花时钟）。 */
    nrf24_last_status = nrf24_rx_buf[0];
}

/* DWT 周期计数器：用来做微秒级延时（CE 脉冲要求 ≥10µs）。
   ⚠️ 不依赖 latency_probe 的开关 —— 产品版关掉延迟探针时，无线照样要能发。
      两个模块各自使能一次 DWT 是幂等的，互不影响（都只是读这个自由运行计数器）。 */
static void nrf24_dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void nrf24_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);   /* 72MHz → 72 周期/µs */

    while ((DWT->CYCCNT - start) < ticks) { /* 忙等：这段时间只有几微秒，不值得让出 CPU */ }
}

/* ============================ 寄存器访问 ============================ */

uint8_t nrf24_read_reg(uint8_t reg)
{
    nrf24_tx_buf[0] = (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1Fu));
    nrf24_tx_buf[1] = NRF24_CMD_NOP;     /* 占位字节：模块在这个字节的时钟上输出寄存器值 */
    nrf24_transfer(2u);
    return nrf24_rx_buf[1];
}

void nrf24_write_reg(uint8_t reg, uint8_t val)
{
    nrf24_tx_buf[0] = (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu));
    nrf24_tx_buf[1] = val;
    nrf24_transfer(2u);
}

void nrf24_read_reg_multi(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (len > NRF24_ADDR_WIDTH) { len = NRF24_ADDR_WIDTH; }   /* 多字节寄存器只有地址类，最长 5 */

    nrf24_tx_buf[0] = (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1Fu));
    for (i = 1u; i <= len; i++) { nrf24_tx_buf[i] = NRF24_CMD_NOP; }

    nrf24_transfer((uint16_t)(len + 1u));

    memcpy(buf, &nrf24_rx_buf[1], len);
}

void nrf24_write_reg_multi(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (len > NRF24_ADDR_WIDTH) { len = NRF24_ADDR_WIDTH; }

    nrf24_tx_buf[0] = (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu));
    for (i = 0u; i < len; i++) { nrf24_tx_buf[i + 1u] = buf[i]; }

    nrf24_transfer((uint16_t)(len + 1u));
}

uint8_t nrf24_get_status(void)
{
    return nrf24_read_reg(NRF24_REG_STATUS);
}

void nrf24_clear_irq(uint8_t flags)
{
    /* STATUS 的标志位是"写 1 清除"，写 0 的位不受影响 */
    nrf24_write_reg(NRF24_REG_STATUS, (uint8_t)(flags & NRF24_STATUS_IRQ_MASK));
}

void nrf24_flush_tx(void)
{
    nrf24_tx_buf[0] = NRF24_CMD_FLUSH_TX;
    nrf24_transfer(1u);
}

void nrf24_flush_rx(void)
{
    nrf24_tx_buf[0] = NRF24_CMD_FLUSH_RX;
    nrf24_transfer(1u);
}

/* ============================ FIFO 负载 ============================ */

void nrf24_write_tx_payload(const uint8_t *buf, uint8_t len)
{
    if (len > NRF24_PAYLOAD_LEN) { len = NRF24_PAYLOAD_LEN; }

    nrf24_tx_buf[0] = NRF24_CMD_W_TX_PAYLOAD;
    memcpy(&nrf24_tx_buf[1], buf, len);
    nrf24_transfer((uint16_t)(len + 1u));
}

void nrf24_read_rx_payload(uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (len > NRF24_PAYLOAD_LEN) { len = NRF24_PAYLOAD_LEN; }

    nrf24_tx_buf[0] = NRF24_CMD_R_RX_PAYLOAD;
    for (i = 1u; i <= len; i++) { nrf24_tx_buf[i] = NRF24_CMD_NOP; }

    nrf24_transfer((uint16_t)(len + 1u));

    memcpy(buf, &nrf24_rx_buf[1], len);
}

/* ============================ 模式切换 ============================ */

void nrf24_ce_low(void)
{
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
}

void nrf24_ce_high(void)
{
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
}

/* 模式设置：直接写常量，【不】用"读 CONFIG → 改一位 → 写回"的写法。
   理由：读改写把整个 CONFIG 的最终值押在"读回值一定正确"这个假设上 ——
   万一某次读回异常（例如 0x00 / 0x08），写回就会把 PWR_UP 一起清掉、模块直接掉电，
   现象是"CE 怎么拉都不发送"，而 SPI 读寄存器一切正常（极难查）。
   我们只需要 PRIM_RX 这一位，其余位（PWR_UP / EN_CRC / CRCO）本就是固定的 ——
   写常量既明确，又不会把自己搞坏。

   （2026-10-16 备注：曾经怀疑这个隐患就是当时"④发送超时"的根因，为此加过一个
     nrf24_cfg_legacy_mode 开关做复现实验；结果 legacy=1（老写法）也能通过，
     证明老写法无罪 —— 那个临时开关已按"结论出来就清理"的约定删除。） */
void nrf24_set_mode_tx(void)
{
    nrf24_ce_low();                                    /* 改模式必须在 CE 低时做 */
    nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_STANDBY);   /* 0x0E：PWR_UP=1、PRIM_RX=0 */
    nrf24_delay_us(150u);                              /* Tstby2a 规格 130µs，留点裕量 */
}

void nrf24_set_mode_rx(void)
{
    nrf24_ce_low();
    nrf24_write_reg(NRF24_REG_CONFIG,
                    (uint8_t)(NRF24_CONFIG_STANDBY | NRF24_CONFIG_PRIM_RX));  /* 0x0F */
    nrf24_delay_us(150u);
    nrf24_ce_high();                                   /* 接收模式：CE 拉高才开始收 */
}

/* ============================ 发一包 ============================ */

/* CE 脉冲宽度（µs）—— 发送参数，默认 NRF24_CE_PULSE_DEFAULT(200)。
   ⚠️ 不指望人工在线改它：Keil 的 Watch 在程序全速运行时改不了值，
      所以"宽度扫描"直接做进自检里（见 nrf24_test.c 的 ④b），一次烧写跑完一遍。

   背景：数据手册上"CE ≥10µs"是【模块已在 standby、PLL 已锁定】的前提；
   而从 standby 真正进入 TX 模式还要 Tstby2a = 130µs，所以理论上脉冲太短会让
   "发送启动"落在临界状态。但 2026-10-16 的扫描数据【否定了宽度是主因】：
   20µs 那次是 3.3ms（快），反倒是 200µs 的首包是 52.4ms（慢）。
   → 嫌疑转到"上电后第一包"上，见 nrf24_test.c 的 ④a。 */
volatile uint16_t nrf24_ce_pulse_us = NRF24_CE_PULSE_DEFAULT;

uint8_t nrf24_send(const uint8_t *buf, uint8_t len, uint32_t timeout_ms)
{
    uint8_t  st;
    uint32_t t0;

    nrf24_set_mode_tx();                    /* PRIM_RX = 0，CE 保持低 */

    nrf24_flush_tx();                       /* 清掉 FIFO 里的旧包，免得先发了旧的 */
    nrf24_clear_irq(NRF24_STATUS_IRQ_MASK); /* 清掉上一次的 TX_DS / MAX_RT */
    nrf24_write_tx_payload(buf, len);

    /* 发送流程：CE 拉高触发发射，宽度 = nrf24_ce_pulse_us，然后拉低 */
    nrf24_ce_high();
    nrf24_delay_us(nrf24_ce_pulse_us);
    nrf24_ce_low();

    /* 轮询等 TX_DS（收到 ACK）或 MAX_RT（重传耗尽）。
       ⚠️ 按 SETUP_RETR = 0x13（ARD 500µs、ARC 3 次），无对端时这里应该 ~2ms 就出结果；
          但 2026-10-16 实测都在 ~50ms 量级（见 nrf24_test.c 里记录的 elapsed）——
          这正是当前要查的异常，头号嫌疑就是上面那个 CE 脉冲宽度。 */
    t0 = HAL_GetTick();
    for (;;)
    {
        st = nrf24_get_status();
        if ((st & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) != 0u) { break; }

        if ((HAL_GetTick() - t0) >= timeout_ms)
        {
            nrf24_clear_irq(NRF24_STATUS_IRQ_MASK);
            return NRF24_TX_TIMEOUT;
        }
    }

    nrf24_clear_irq(NRF24_STATUS_IRQ_MASK);

    return ((st & NRF24_STATUS_TX_DS) != 0u) ? NRF24_TX_OK : NRF24_TX_MAXRT;
}

/* ============================ 初始化 ============================ */

void nrf24_init(void)
{
    uint8_t rd    = 0u;
    uint8_t retry;

    nrf24_ce_low();          /* 配置期间 CE 必须低（高电平时模块在做收发，SPI 行为会变）*/
    nrf24_dwt_enable();

    /* ---- 0) 先等模块上电复位完成（2026-10-23 加）----
       ST-Link 下载时代 Flash 编程电流会把 3.3V 拉一下 → nRF24 跟着掉电；
       重新上电后它要 ~10ms 才响应 SPI，这段时间写的寄存器【全部被忽略】。
       症状：下载完程序在跑、wtx_sent 在涨，但 wtx_maxrt 一起涨（模块根本没配置上）；
             过一会儿按一下 MCU 复位键就好 —— 那时模块早已稳定。
       ⚠️ 这一步不能靠下面的软复位替代：软复位本身就要用 SPI 去写寄存器。 */
    HAL_Delay(NRF24_BOOT_POR_MS);

    /* ---- 1) 软复位：先 Power Down 再上电，带【读回校验 + 重试】----
       读回校验的作用：把"模块没应答"和"配置写进去了"分开。
       只写不读的话，模块还在 POR 里时写 0x08 和写 0x0E 都是白写，表面看一切正常。 */
    for (retry = 0u; retry < NRF24_BOOT_RETRY_MAX; retry++)
    {
        nrf24_write_reg(NRF24_REG_CONFIG, 0x08u);   /* PWR_UP=0，EN_CRC=1，CRCO=1 → Power Down */
        HAL_Delay(2u);

        /* 上电：PWR_UP=1 + EN_CRC=1 + CRCO=1（2 字节 CRC），PRIM_RX=0 */
        nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_STANDBY);
        HAL_Delay(2u);           /* Tpd2stby = 1.5ms（上电 → 待机），精度不敏感，用 HAL 时基即可 */

        rd = nrf24_read_reg(NRF24_REG_CONFIG);
        if ((rd & 0x0Eu) == 0x0Eu) { break; }   /* PWR_UP / EN_CRC / CRCO 都置上了 → 真的起来了 */
    }
    nrf24_boot_retry  = retry;   /* 0 = 一次成功；= NRF24_BOOT_RETRY_MAX → 一直没应答 */
    nrf24_boot_config = rd;      /* 正常 0x0E；0x00 / 0xFF → SPI 没通 */

    /* ---- 2) 基本参数 ---- */
    nrf24_write_reg(NRF24_REG_SETUP_AW,    0x03u);                   /* 5 字节地址 */
    nrf24_write_reg(NRF24_REG_SETUP_RETR,  0x13u);                   /* ARD = 500µs、ARC = 3 次重传 */
    nrf24_write_reg(NRF24_REG_EN_AA,       0x01u);                   /* Pipe0 开自动应答 */
    nrf24_write_reg(NRF24_REG_EN_RXADDR,   0x01u);                   /* 只用 Pipe0 */
    nrf24_write_reg(NRF24_REG_RF_CH,       NRF24_RF_CH_DEFAULT);     /* 2.476 GHz */
    nrf24_write_reg(NRF24_REG_RF_SETUP,    NRF24_RF_SETUP_1M_0DBM);  /* 1 Mbps / 0 dBm */
    nrf24_write_reg(NRF24_REG_RX_PW_P0,    NRF24_PAYLOAD_LEN);       /* Pipe0 固定包长 */

    /* ---- 3) 地址：增强型 ShockBurst 要求 TX_ADDR 与 RX_ADDR_P0 相同，否则收不到 ACK ---- */
    nrf24_write_reg_multi(NRF24_REG_TX_ADDR,    nrf24_addr_p0, NRF24_ADDR_WIDTH);
    nrf24_write_reg_multi(NRF24_REG_RX_ADDR_P0, nrf24_addr_p0, NRF24_ADDR_WIDTH);

    /* ---- 4) 清 FIFO 和中断标志，交出一个干净的待机态 ---- */
    nrf24_flush_tx();
    nrf24_flush_rx();
    nrf24_clear_irq(NRF24_STATUS_IRQ_MASK);
}
