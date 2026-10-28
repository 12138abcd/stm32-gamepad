/**
 * @file    analog_input.c
 * @brief   模拟输入映射：ADC 原始值 -> HID 报告轴字段
 *
 * 两类被测量的规律完全不同：
 *   摇杆（双极性）：中心 128，两侧各自到 0 / 255   —— axis_map()
 *   扳机（单极性）：0 = 松开、255 = 到底           —— trig_map()
 *
 * @note 摇杆校准常量已移到 Flash（`param_store.c` 里的默认值 = 2026-10-07 实测值），
 *       这里从 param_get() 读。⚠️ 换摇杆 / 换板子 / 拆装之后必须重新标定。
 */

#include "analog_input.h"
#include "report.h"        /* gamepad_report[] */
#include "param_store.h"   /* 摇杆校准常量 */

/* 死区（8 位报告值）：5 ≈ 满量程 2%。实测静置噪声峰峰 3~7 counts（映射后 < 1 LSB），
   这个值偏保守；想更灵敏可以试 3。左右摇杆共用。 */
#define JS_DEADZONE      5

/* ADC 结果缓冲区（定义在 main.c 的 USER CODE 区），下标 = CubeMX 的 Rank 顺序：
   [0] PA0/IN0 左X   [1] PA1/IN1 左Y   [2] PA2/IN2 右X
   [3] PA3/IN3 右Y   [4] PB0/IN8 左扳机 [5] PB1/IN9 右扳机 */
extern uint16_t adc_buf[6];

/**
 * @brief  摇杆映射（双极性）：12 位 -> 8 位，中心 128
 * @param  half_neg 中心 -> 负极端的行程
 * @param  half_pos 中心 -> 正极端的行程
 * @note   正方向用 127、负方向用 128。用 128 时 128+128=256 会溢出成 0，
 *         现象是"推到最右，光标瞬间跳到最左"。
 */
static uint8_t axis_map(uint16_t raw, int32_t center, uint16_t half_neg, uint16_t half_pos)
{
    int32_t d = (int32_t)raw - center;
    int32_t v;

    if (d >= 0)
        v = 128 + (d * 127) / (int32_t)half_pos;
    else
        v = 128 - ((-d) * 128) / (int32_t)half_neg;

    if (v < 0)   v = 0;          /* 钳位保险 */
    if (v > 255) v = 255;

    if (v > 128 - JS_DEADZONE && v < 128 + JS_DEADZONE) v = 128;   /* 死区 */

    return (uint8_t)v;
}

/* 扳机端点（12 位）。实测：松开 0~10、按到底 4093~4095，上限取下限以保证两路都能到 255。
   ⚠️ 扳机没有中心点，绝不能复用 axis_map()（那是中心 128 的双极性映射）。
   松手抖动最大 10 counts，经整数除法自然归零，故无需死区。 */
#define TRIG_MIN_RAW   0
#define TRIG_MAX_RAW   4093

static uint8_t trig_map(uint16_t raw)
{
    if (raw <= TRIG_MIN_RAW) return 0;
    if (raw >= TRIG_MAX_RAW) return 255;
    return (uint8_t)(((uint32_t)(raw - TRIG_MIN_RAW) * 255u) / (TRIG_MAX_RAW - TRIG_MIN_RAW));
}

void analog_update(void)
{
    const pad_params_t *p = param_get();   /* 校准常量来自 Flash */

    /* 摇杆（双极性）：byte3 = X、byte4 = Y、byte5 = Rx、byte6 = Ry */
    gamepad_report[3] = axis_map(adc_buf[0], p->js_lx_center, p->js_lx_half_l, p->js_lx_half_r);
    gamepad_report[4] = axis_map(adc_buf[1], p->js_ly_center, p->js_ly_half_u, p->js_ly_half_d);
    gamepad_report[5] = axis_map(adc_buf[2], p->js_rx_center, p->js_rx_half_l, p->js_rx_half_r);
    gamepad_report[6] = axis_map(adc_buf[3], p->js_ry_center, p->js_ry_half_u, p->js_ry_half_d);

    /* 扳机（单极性）：byte7 = Z = 左扳机、byte8 = Rz = 右扳机 */
    gamepad_report[7] = trig_map(adc_buf[4]);
    gamepad_report[8] = trig_map(adc_buf[5]);
}
