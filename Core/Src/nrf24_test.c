/**
  ******************************************************************************
  * @file    nrf24_test.c
  * @brief   nRF24 驱动自检实现（无线 A 路线 · 第 3 步）。判据与用法见 nrf24_test.h。
  ******************************************************************************
  */

#include "nrf24_test.h"
#include "nrf24.h"          /* 驱动 */
#include "main.h"           /* DWT / HAL_GetTick */
#include <string.h>         /* memcmp */

/* ============================ Watch 观察项 ============================ */
volatile uint8_t  nrf24_test_mask = 0u;
volatile uint8_t  nrf24_test_pass = 0u;
volatile uint8_t  nrf24_test_done = 0u;
volatile uint8_t  nrf24_test_rerun = 0u;

volatile uint8_t  nrf24_chk_reg_rw  = 0u;
volatile uint8_t  nrf24_chk_addr_rw = 0u;
volatile uint8_t  nrf24_chk_fifo    = 0u;
volatile uint8_t  nrf24_chk_tx      = 0u;
volatile uint8_t  nrf24_chk_rx      = 0u;

volatile uint32_t nrf24_fail_code = 0u;
volatile uint8_t  nrf24_addr_readback[5] = { 0u, 0u, 0u, 0u, 0u };
volatile uint8_t  nrf24_fifo_probe[3]    = { 0u, 0u, 0u };

volatile uint32_t nrf24_tx_result     = 0xFFu;
volatile uint32_t nrf24_observe_tx    = 0u;
volatile uint32_t nrf24_tx_elapsed_us = 0u;

/* ④的诊断量（见 nrf24_test.h）：把"CE 没触发"与"配置没生效"分开 */
volatile uint8_t  nrf24_config_readback = 0u;
volatile uint8_t  nrf24_config_txmode   = 0u;
volatile uint8_t  nrf24_cfg_before_send = 0u;
volatile uint8_t  nrf24_fifo_after_tx   = 0u;

/* ④a：连发 3 包（盯着"上电后第一包"，见 nrf24_test.h） */
volatile uint32_t nrf24_tx_result_repeat[NRF24_REPEAT_COUNT]  = { 0u };
volatile uint32_t nrf24_tx_elapsed_repeat[NRF24_REPEAT_COUNT] = { 0u };
volatile uint32_t nrf24_observe_repeat[NRF24_REPEAT_COUNT]    = { 0u };

volatile uint8_t  nrf24_rx_status_raw = 0u;
volatile uint8_t  nrf24_rx_fifo_raw   = 0u;
volatile uint8_t  nrf24_rpd           = 0u;
volatile uint8_t  nrf24_ce_readback   = 0u;   /* CE(PB3) 引脚回环：1 = 写进去能读回来 */

volatile uint32_t nrf24_status   = 0u;
volatile uint8_t  nrf24_status_b = 0u;
volatile uint32_t nrf24_reads    = 0u;

/* ============================ 自检用常量 ============================ */
/* ②的回环地址：故意用一个和项目地址完全不同的值，避免"本来就是这个值"的假通过 */
static const uint8_t nrf24_test_addr[NRF24_ADDR_WIDTH] = { 'T', 'E', 'S', 'T', '9' };

/* ④的测试载荷：4 字节，内容随意（第 4 步才验内容一致性） */
static const uint8_t nrf24_test_payload[4] = { 0xA5u, 0x5Au, 0x01u, 0xFEu };

#define NRF24_SELFTEST_TX_TIMEOUT_MS   50u   /* 无对端时约 2ms 就该有结果，50ms 是兜底 */

/* ============================ 记忆失败原因 ============================ */
static void nrf24_record_fail(uint32_t item, uint32_t value)
{
    if (nrf24_fail_code == 0u)
    {
        nrf24_fail_code = ((item & 0xFFu) << 24) | (value & 0xFFFFFFu);
    }
}

/* ============================ 自检主流程 ============================ */
void nrf24_test_run(void)
{
    uint8_t rd;

    /* 每次都从干净状态开始 */
    nrf24_test_mask  = 0u;
    nrf24_test_pass  = 0u;
    nrf24_fail_code  = 0u;

    nrf24_chk_reg_rw  = 0u;
    nrf24_chk_addr_rw = 0u;
    nrf24_chk_fifo    = 0u;
    nrf24_chk_tx      = 0u;
    nrf24_chk_rx      = 0u;

    nrf24_addr_readback[0] = 0u;
    nrf24_addr_readback[1] = 0u;
    nrf24_addr_readback[2] = 0u;
    nrf24_addr_readback[3] = 0u;
    nrf24_addr_readback[4] = 0u;
    nrf24_fifo_probe[0] = 0u;
    nrf24_fifo_probe[1] = 0u;
    nrf24_fifo_probe[2] = 0u;

    nrf24_tx_result     = 0xFFu;
    nrf24_observe_tx    = 0u;
    nrf24_tx_elapsed_us = 0u;

    nrf24_config_readback = 0u;
    nrf24_config_txmode   = 0u;
    nrf24_cfg_before_send = 0u;
    nrf24_fifo_after_tx   = 0u;

    nrf24_rx_status_raw = 0u;
    nrf24_rx_fifo_raw   = 0u;
    nrf24_rpd           = 0u;
    nrf24_ce_readback   = 0u;

    nrf24_ce_low();        /* 整个自检期间自由改寄存器：CE 必须先拉低 */

    /* ============ ⓪ CE 引脚回环（不进 mask，纯诊断用） ============
       PB3 是 JTAG 的 JTDO 脚：如果 CubeMX 生成的 __HAL_AFIO_REMAP_SWJ_NOJTAG()
       没生效，PB3 会被 JTAG 抢占，CE 就永远动不了 —— 而那种情况下 SPI 读寄存器
       【全部正常】（①②③⑤全过），只表现为"④发送状态机没反应"。
       先写再读 GPIO 自身，把这条最容易静默失效的路堵上。 */
    {
        uint8_t ce_lo, ce_hi;

        nrf24_ce_low();
        nrf24_delay_us(5u);
        ce_lo = (HAL_GPIO_ReadPin(CE_GPIO_Port, CE_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

        nrf24_ce_high();
        nrf24_delay_us(5u);
        ce_hi = (HAL_GPIO_ReadPin(CE_GPIO_Port, CE_Pin) == GPIO_PIN_SET) ? 1u : 0u;

        nrf24_ce_readback = (uint8_t)(ce_lo & ce_hi);

        nrf24_ce_low();
    }

    /* 配置回读：init 里写完 CONFIG 之后到底生效没有（期望 0x0E = PWR_UP=1、PRIM_RX=0） */
    nrf24_config_readback = nrf24_read_reg(NRF24_REG_CONFIG);

    /* ================= ① 单字节寄存器写-读回环 =================
       用 RF_CH（信道，可自由读写）。写两个不同的值各读回一次 ——
       只写一次的话，万一寄存器本来就是那个值就成了假通过。 */
    nrf24_write_reg(NRF24_REG_RF_CH, 0x4Cu);
    rd = nrf24_read_reg(NRF24_REG_RF_CH);
    if (rd == 0x4Cu)
    {
        nrf24_write_reg(NRF24_REG_RF_CH, 0x25u);
        rd = nrf24_read_reg(NRF24_REG_RF_CH);
        if (rd == 0x25u) { nrf24_chk_reg_rw = 1u; }
        else             { nrf24_record_fail(1u, rd); }
    }
    else
    {
        nrf24_record_fail(1u, rd);
    }
    nrf24_write_reg(NRF24_REG_RF_CH, NRF24_RF_CH_DEFAULT);   /* 无论成败都恢复默认信道 */

    /* ================= ② 多字节地址写-读回环 =================
       地址是 5 字节寄存器，能验出"命令 + 数据是否在同一个 CSN 周期里发完"。 */
    nrf24_write_reg_multi(NRF24_REG_RX_ADDR_P0, nrf24_test_addr, NRF24_ADDR_WIDTH);
    nrf24_read_reg_multi(NRF24_REG_RX_ADDR_P0, (uint8_t *)nrf24_addr_readback, NRF24_ADDR_WIDTH);

    if (memcmp((const void *)nrf24_addr_readback, nrf24_test_addr, NRF24_ADDR_WIDTH) == 0)
    {
        nrf24_chk_addr_rw = 1u;
    }
    else
    {
        nrf24_record_fail(2u, nrf24_addr_readback[0]);
    }
    /* 恢复项目地址（第 4 步点对点要用它） */
    nrf24_write_reg_multi(NRF24_REG_RX_ADDR_P0, nrf24_addr_p0, NRF24_ADDR_WIDTH);

    /* ================= ③ TX FIFO 通路 =================
       三步：清空后应为空 → 写一包后应变为非空 → 再清空又应为空。
       中间"变成非空"这一步是关键：它证明 W_TX_PAYLOAD 真的写进去了。 */
    nrf24_flush_tx();
    nrf24_fifo_probe[0] = nrf24_read_reg(NRF24_REG_FIFO_STATUS);

    nrf24_write_tx_payload(nrf24_test_payload, 4u);
    nrf24_fifo_probe[1] = nrf24_read_reg(NRF24_REG_FIFO_STATUS);

    nrf24_flush_tx();
    nrf24_fifo_probe[2] = nrf24_read_reg(NRF24_REG_FIFO_STATUS);

    if (((nrf24_fifo_probe[0] & NRF24_FIFO_TX_EMPTY) != 0u) &&
        ((nrf24_fifo_probe[1] & NRF24_FIFO_TX_EMPTY) == 0u) &&
        ((nrf24_fifo_probe[2] & NRF24_FIFO_TX_EMPTY) != 0u))
    {
        nrf24_chk_fifo = 1u;
    }
    else
    {
        nrf24_record_fail(3u, nrf24_fifo_probe[1]);
    }

    /* ================= ④ 发送状态机有响应（含两组回归实验） =================
       CE 脉冲后能等到 TX_DS 或 MAX_RT，就说明"发射真的发生了"。
       ⚠️ 单板无对端时，正确答案是 MAX_RT（重传耗尽）——
          它恰恰证明包发出去了、重传也按 SETUP_RETR 跑了。
          只有"超时"（状态位一直不动）才算失败。

       这一节顺带跑两组实验，用来定位那个 52.4ms 的异常：
         ④a 连发 3 包 —— 看"上电后第一包"是不是特别慢
         ④b 宽度扫描 —— 复验 CE 脉冲宽度到底有没有影响 */
    {
        uint32_t t0;
        uint8_t  i;

        /* 发送【前】的 CONFIG（期望 0x0E） */
        nrf24_cfg_before_send = nrf24_read_reg(NRF24_REG_CONFIG);

        /* ---- ④a：连发 NRF24_REPEAT_COUNT 包（固定用默认宽度）----
           这里的 [0] 就是"上电后自检里的第一次射频发送"。
           ⭐ 判读：[0] 若 ≈52000 而 [1]/[2] ≈3300 → "首包慢"成立。 */
        for (i = 0u; i < NRF24_REPEAT_COUNT; i++)
        {
            nrf24_ce_pulse_us = NRF24_CE_PULSE_DEFAULT;

            t0 = DWT->CYCCNT;
            nrf24_tx_result_repeat[i]  = nrf24_send(nrf24_test_payload, 4u, NRF24_SELFTEST_TX_TIMEOUT_MS);
            nrf24_tx_elapsed_repeat[i] = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
            nrf24_observe_repeat[i]    = nrf24_read_reg(NRF24_REG_OBSERVE_TX);
        }

        nrf24_ce_pulse_us = NRF24_CE_PULSE_DEFAULT;   /* 固定用默认宽度（宽度结论已定：无影响）*/

        /* ---- ④ 的通过判据：用 ④a 的最后一包（此时模块肯定是"热"的）---- */
        nrf24_tx_result     = nrf24_tx_result_repeat[NRF24_REPEAT_COUNT - 1u];
        nrf24_tx_elapsed_us = nrf24_tx_elapsed_repeat[NRF24_REPEAT_COUNT - 1u];
        nrf24_observe_tx    = nrf24_observe_repeat[NRF24_REPEAT_COUNT - 1u];

        nrf24_config_txmode = nrf24_read_reg(NRF24_REG_CONFIG);
        nrf24_fifo_after_tx = nrf24_read_reg(NRF24_REG_FIFO_STATUS);

        if (nrf24_tx_result != NRF24_TX_TIMEOUT)
        {
            nrf24_chk_tx = 1u;
        }
        else
        {
            nrf24_record_fail(4u, (uint32_t)nrf24_last_status);
        }
    }

    /* ================= ⑤ 能切进接收模式 =================
       进 RX 后：STATUS 的 RX_P_NO（bit3~1）应为 111（RX FIFO 空，没收到东西），
       且 FIFO_STATUS 的 RX_EMPTY 应为 1。 */
    nrf24_flush_rx();
    nrf24_set_mode_rx();          /* 内部会把 CE 拉高 */

    nrf24_delay_us(500u);         /* 给接收机稳定时间：Tstby2a 规格 130µs，留足裕量 */

    /* ⚠️ 读 STATUS 时 CE 保持高是对的 —— 接收模式下本来就是这样读的
          （真收到包时也是 CE 高着读 STATUS + 取 payload），这里不额外拉低。 */
    nrf24_rx_status_raw = nrf24_read_reg(NRF24_REG_STATUS);
    nrf24_rx_fifo_raw   = nrf24_read_reg(NRF24_REG_FIFO_STATUS);
    nrf24_rpd           = (uint8_t)(nrf24_read_reg(NRF24_REG_RPD) & 0x01u);

    if (((nrf24_rx_status_raw & NRF24_STATUS_RX_P_NO) == NRF24_STATUS_RX_P_NO) &&
        ((nrf24_rx_fifo_raw & NRF24_FIFO_RX_EMPTY) != 0u))
    {
        nrf24_chk_rx = 1u;
    }
    else
    {
        nrf24_record_fail(5u, nrf24_rx_status_raw);
    }

    /* 收工：CE 拉低、回到待机发送态，把模块交回给后续使用 */
    nrf24_ce_low();
    nrf24_set_mode_tx();

    /* ================= 汇总 ================= */
    nrf24_test_mask = (uint8_t)((nrf24_chk_reg_rw  << 0) |
                                (nrf24_chk_addr_rw << 1) |
                                (nrf24_chk_fifo    << 2) |
                                (nrf24_chk_tx      << 3) |
                                (nrf24_chk_rx      << 4));

    nrf24_test_pass = (nrf24_test_mask == 0x1Fu) ? 1u : 0u;
}

/* ============================ 对外接口 ============================ */

void nrf24_test_init(void)
{
    nrf24_init();     /* 硬件初始化交给驱动；main.c 的调用点因此不用改 */
}

void nrf24_test_poll(void)
{
    static uint32_t t_last = 0u;

    /* 在线重跑：Watch 里把 nrf24_test_rerun 写成 1（下一拍生效，不用重烧固件） */
    if (nrf24_test_rerun != 0u)
    {
        nrf24_test_rerun = 0u;
        nrf24_test_done  = 0u;
    }

    /* 自检只跑一遍。⚠️ 这一拍会占用 1ms 任务约 2~3ms（④要等重传），
          属调试期的一次性动作；跑完就不再占用关键路径。
          若此时正在测延迟，直方图里可能多出 1~2 个尖峰样本 ——
          先跑完自检，再把 probe_clear 写 1 清一次统计即可。 */
    if (nrf24_test_done == 0u)
    {
        nrf24_test_run();
        nrf24_test_done = 1u;

        nrf24_status   = nrf24_read_reg(NRF24_REG_STATUS);
        nrf24_status_b = (uint8_t)nrf24_status;
        t_last = HAL_GetTick();
        return;
    }

    /* 之后每 100ms 读一次 STATUS 做在线监视（连读两次交叉验证），与第 2 步一致 */
    if ((HAL_GetTick() - t_last) < 100u) { return; }
    t_last = HAL_GetTick();

    nrf24_status   = nrf24_read_reg(NRF24_REG_STATUS);
    nrf24_status_b = nrf24_read_reg(NRF24_REG_STATUS);
    nrf24_reads++;
}
