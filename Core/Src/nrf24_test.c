/**
  ******************************************************************************
  * @file    nrf24_test.c
  * @brief   nRF24L01 最小验证：读 STATUS 寄存器，证明 SPI 通了。
  *          详见 nrf24_test.h 的说明。
  ******************************************************************************
  */

#include "nrf24_test.h"
#include "main.h"

/* ⚠️ 本文件【不】自己初始化 SPI 和 GPIO —— 那些已经由 CubeMX 生成好了：
       MX_SPI1_Init()  → hspi1（PA5/PA6/PA7，8 位，Mode0，预分频 64 ≈ 1.125 MBits/s）
       MX_GPIO_Init()  → CSN(PA4，初值 High) / CE(PB3，初值 Low)
     这里直接使用它们即可；重复初始化只会让代码变脏，而且与 .ioc 不一致。 */

extern SPI_HandleTypeDef hspi1;      /* 定义在 main.c（CubeMX 生成） */

/* ============================ 统计量（Watch 观察） ============================ */
volatile uint32_t nrf24_status = 0u;    /* 最近一次读到的 STATUS（期望 ≈0x0E）*/
volatile uint8_t  nrf24_status_b = 0u;  /* 紧接着再读一次，应与之相同（交叉验证）*/
volatile uint32_t nrf24_reads = 0u;     /* 一共读了多少次 */
volatile uint32_t nrf24_err = 0u;       /* SPI 传输出错的次数（正常应为 0）*/

/* ============================ 初始化 ============================
   空函数：CubeMX 已经把 SPI1 与 CSN/CE 都配好了。
   保留它是为了让 main.c 的调用点稳定（将来要加东西就放这里 —— 例如
   写 CONFIG、设置地址、进入 RX 模式等）。 */
void nrf24_test_init(void)
{
    /* 目前无需做任何事 */
}

/* ============================ 读寄存器 ============================
   nRF24 的 SPI 命令格式：bit7~5 = 命令，bit4~0 = 寄存器地址
     R_REGISTER = 000  → 先发 (000 | 地址)，再发一个字节（内容任意），
                        返回的那个字节就是寄存器值。
   ⚠️ 整个过程 CSN 必须保持拉低。 */
static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t cmd = (uint8_t)(reg & 0x1Fu);   /* R_REGISTER(000) | 地址 */
    uint8_t val = 0u;

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);      /* 拉低：选中 */

    if (HAL_SPI_Transmit(&hspi1, &cmd, 1u, 10u) != HAL_OK) { nrf24_err++; }
    if (HAL_SPI_Receive(&hspi1, &val, 1u, 10u)  != HAL_OK) { nrf24_err++; }

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);        /* 拉高：取消选中 */

    return val;
}

/* ============================ 周期验证 ============================ */

void nrf24_test_poll(void)
{
    static uint32_t t_last = 0u;

    /* 100ms 节流：它是低频任务，不该占用 1ms 关键路径 */
    if ((HAL_GetTick() - t_last) < 100u) { return; }
    t_last = HAL_GetTick();

    /* 连读两次同一个寄存器：正常应完全相同（交叉验证，也能暴露读数不稳定） */
    nrf24_status   = nrf_read_reg(0x07u);   /* STATUS */
    nrf24_status_b = nrf_read_reg(0x07u);
    nrf24_reads++;
}

/* ============================ 寄存器地址备查 ============================
   0x00 CONFIG      上电默认 0x08（EN_CRC=1、PWR_UP=0）
   0x07 STATUS      上电默认 0x0E（RX_P_NO = 111 = RX FIFO 空）
   → 第一次就读到 0x0E 属于"教科书式正常"。
   → 读到 0x00 / 0xFF 基本等于 SPI 完全没通（MISO 一直是低 / 一直是高）。
   ⚠️ 读寄存器时 CE 必须为低 —— CubeMX 已把 PB3 初值设为 Low，正合适。 */
