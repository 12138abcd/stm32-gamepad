/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   模拟摇杆：把 ADC 的原始读数校准映射成 HID 报告的轴字段。
  ******************************************************************************
  */

#include "joystick.h"
#include "report.h"        /* gamepad_report[] */

/* ============================ 校准常量 ============================
   2026-10-05 实测：用 Keil 的 Watch 窗口读 adc_buf 在"静置 / 推到底"时的值。
   理论中位是 2048，但电位器有机械公差，实测中心是 2001 / 2000。
   换摇杆或换板子之后这几个数要重新测，别照抄。                     */
#define JS_X_CENTER    2001     /* X 轴静置中心（12 位原始值） */
#define JS_Y_CENTER    2000     /* Y 轴静置中心 */
#define JS_X_HALF_L    2001     /* 中心 → 最左的行程 */
#define JS_X_HALF_R    2032     /* 中心 → 最右的行程 */
#define JS_Y_HALF_U    2000     /* 中心 → 最上的行程 */
#define JS_Y_HALF_D    2030     /* 中心 → 最下的行程 */

/* 死区：离中心太近就当作回中，防止松手后光标自己抖。
   单位是【8 位报告值】，5 ≈ 满量程的 2%。
   实测静置噪声峰峰值只有 14 counts（映射后 < 1 LSB），所以这个值偏保守，
   想要更灵敏的手感可以试 3。 */
#define JS_DEADZONE      5

/* adc_buf 定义在 main.c 的 USER CODE 区，是 ADC + DMA 的转换结果缓冲区：
   [0] = PA0 = VRx（X 轴，左右）    [1] = PA1 = VRy（Y 轴，上下） */
extern uint16_t adc_buf[2];

/* ============================ 轴映射 ============================
   12 位原始值（0~4095）→ 8 位报告值（0~255），以实测中心为 128。
   左右半行程分别拉伸：电位器天生不对称，分开算才能让"推到底 = 255"。
   纯函数：没有状态，输入一样输出就一样。 */
static uint8_t axis_map(uint16_t raw, int32_t center, uint16_t half_neg, uint16_t half_pos)
{
    int32_t d = (int32_t)raw - center;
    int32_t v;

    /* ⚠️ 正向必须用 127、负向用 128：
          d == half_pos 时 128 + 128 = 256，存进 uint8_t 会溢出成 0，
          表现为"推到最右，光标瞬间跳到最左"。用 127 恰好得 255。 */
    if (d >= 0)
        v = 128 + (d * 127) / (int32_t)half_pos;
    else
        v = 128 - ((-d) * 128) / (int32_t)half_neg;

    if (v < 0)   v = 0;          /* 钳位保险，绝不让它越界 */
    if (v > 255) v = 255;

    if (v > 128 - JS_DEADZONE && v < 128 + JS_DEADZONE) v = 128;   /* 死区 */

    return (uint8_t)v;
}

/* ============================ 更新报告里的轴字段 ============================ */
void joystick_update(void)
{
    gamepad_report[3] = axis_map(adc_buf[0], JS_X_CENTER, JS_X_HALF_L, JS_X_HALF_R);   /* X 轴 */
    gamepad_report[4] = axis_map(adc_buf[1], JS_Y_CENTER, JS_Y_HALF_U, JS_Y_HALF_D);   /* Y 轴 */
}
