/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   模拟摇杆：把 ADC 的原始读数校准映射成 HID 报告的轴字段。
  ******************************************************************************
  */

#include "joystick.h"
#include "report.h"        /* gamepad_report[] */

/* ============================ 校准常量 ============================
   【左摇杆 LX / LY】2026-10-05 实测：用 Keil 的 Watch 窗口读 adc_buf 在
   "静置 / 推到底"时的值。理论中位是 2048，但电位器有机械公差，实测中心是 2001 / 2000。
   换摇杆或换板子之后这几个数要重新测，别照抄。                     */
#define JS_LX_CENTER   2001     /* 左摇杆 X 静置中心（12 位原始值） */
#define JS_LY_CENTER   2000     /* 左摇杆 Y 静置中心 */
#define JS_LX_HALF_L   2001     /* 中心 → 最左的行程 */
#define JS_LX_HALF_R   2032     /* 中心 → 最右的行程 */
#define JS_LY_HALF_U   2000     /* 中心 → 最上的行程 */
#define JS_LY_HALF_D   2030     /* 中心 → 最下的行程 */

/* 【右摇杆 RX / RY】2026-10-07 实测（同款摇杆模块、同一套 3.3V 供电）。
   原始读数记录在 `手柄项目\_项目状态.md` 第七节"右摇杆标定"，这里只放结论。
   ⚠️ 换摇杆、换板子，或者拆下来重装之后，这几个数必须重测 —— 别照抄。 */
#define JS_RX_CENTER   1984     /* 右摇杆 X 静置中心（实测 1980~1989） */
#define JS_RX_HALF_L   1984     /* 中心 → 最左（最左实测 0） */
#define JS_RX_HALF_R   2062     /* 中心 → 最右（最右实测 4039~4053） */
#define JS_RY_CENTER   2019     /* 右摇杆 Y 静置中心（实测 2014~2024） */
#define JS_RY_HALF_U   2019     /* 中心 → 最上（最上实测 0） */
#define JS_RY_HALF_D   2013     /* 中心 → 最下（最下实测 4032） */

/* 死区：离中心太近就当作回中，防止松手后光标自己抖。
   单位是【8 位报告值】，5 ≈ 满量程的 2%。
   实测静置噪声峰峰值只有 14 counts（映射后 < 1 LSB），所以这个值偏保守，
   想要更灵敏的手感可以试 3。左右摇杆共用同一个死区。 */
#define JS_DEADZONE      5

/* adc_buf 定义在 main.c 的 USER CODE 区，是 ADC + DMA 的转换结果缓冲区
   （下标顺序 = CubeMX 里的 Rank 顺序，一一对应，不用查表）：
     [0] = PA0 = IN0 = 左摇杆 VRx → 报告 byte3 = X
     [1] = PA1 = IN1 = 左摇杆 VRy → 报告 byte4 = Y
     [2] = PA2 = IN2 = 右摇杆 VRx → 报告 byte5 = Rx
     [3] = PA3 = IN3 = 右摇杆 VRy → 报告 byte6 = Ry
     [4] = PB0 = IN8 = 左扳机 LT  → 报告 byte7 = Z
     [5] = PB1 = IN9 = 右扳机 RT  → 报告 byte8 = Rz
   ⚠️ [4]/[5] 的固件映射还没写（扳机器件也未定），先留在缓冲区里不参与上报。 */
extern uint16_t adc_buf[6];

/* ============================ 轴映射 ============================
   12 位原始值（0~4095）→ 8 位报告值（0~255），以实测中心为 128。
   左右半行程分别拉伸：电位器天生不对称，分开算才能让"推到底 = 255"。
   纯函数：没有状态，输入一样输出就一样。左右摇杆共用这一份逻辑。 */
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
    /* ---- 左摇杆：byte3 = X，byte4 = Y（常量是 2026-10-05 实测值） ---- */
    gamepad_report[3] = axis_map(adc_buf[0], JS_LX_CENTER, JS_LX_HALF_L, JS_LX_HALF_R);
    gamepad_report[4] = axis_map(adc_buf[1], JS_LY_CENTER, JS_LY_HALF_U, JS_LY_HALF_D);

    /* ---- 右摇杆：byte5 = Rx，byte6 = Ry（常量是 2026-10-07 实测值） ---- */
    gamepad_report[5] = axis_map(adc_buf[2], JS_RX_CENTER, JS_RX_HALF_L, JS_RX_HALF_R);
    gamepad_report[6] = axis_map(adc_buf[3], JS_RY_CENTER, JS_RY_HALF_U, JS_RY_HALF_D);
}
