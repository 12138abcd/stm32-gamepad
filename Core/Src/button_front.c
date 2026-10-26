/**
 * @file    button_front.c
 * @brief   12 个按键 + 两个摇杆按下 -> 消抖 -> 写 HID 报告的按键位
 *
 * 非阻塞消抖：每 1ms 调一次，靠"连续 N 拍与当前判断相反"才改判，不等待、不阻塞。
 * 逐拍流程见《固件代码导读》02-§2.2。
 */

#include "button_front.h"
#include "main.h"
#include "report.h"
#include "usbd_customhid.h"

/* 消抖参数（单位：拍，1 拍 = 1ms）。
   按下 1 拍、松开 4 拍：人对按下的延迟敏感，松手晚几毫秒无人感知，且能防误松开。

   ⚠️ 故意写成 volatile 变量而非宏：做参数对比实验时可直接在 Watch 里改值，
      不用重新编译烧写，两轮实验条件完全一致。

   ⚠️ 1 拍是实测选的（各按 100 次）：2 拍 -> P50 1590µs；1 拍 -> P50 520µs，
      min 差 1005µs ≈ 正好 1 个 1ms 节拍。当前布线 100 次零误触发；
      若以后换长线 / 强干扰环境，可退回 2 拍。 */
volatile uint8_t db_press_ticks   = 1u;   /* 按下：连续几拍一致就认 */
volatile uint8_t db_release_ticks = 4u;   /* 松开：连续几拍一致就认 */

/* 按键是上拉输入：按下时引脚被拉到地，读到 0 */
#define BTN_DOWN(port, pin)   (HAL_GPIO_ReadPin((port), (pin)) == GPIO_PIN_RESET)

/* 消抖状态机：每键一份。stable = 当前判断，count = 连续反对票数 */
typedef struct {
    uint8_t stable;
    uint8_t count;
} deb_t;

static deb_t db_up, db_down, db_left, db_right;   /* 十字键 4 触点 */
static deb_t db_a,  db_b,    db_x,    db_y;       /* 面键 */
static deb_t db_l3, db_r3;                        /* 摇杆按下：PB9 / PA8 */
static deb_t db_lb, db_rb, db_view, db_menu;      /* PA15 / PB4 / PA9 / PA10 */

/* 调试计数器（volatile：否则会被优化掉，调试器里永远读到 0）*/
volatile uint32_t count_a   = 0;
volatile uint32_t count_pov = 0;
volatile uint32_t count_l3  = 0;
volatile uint32_t count_r3  = 0;
volatile uint32_t count_lb  = 0;
volatile uint32_t count_rb  = 0;

/**
 * @brief  非阻塞消抖：只有"与当前判断相反的读数"连续够多拍才改判
 * @note   threshold 每拍动态选：想改成"按下"用 1 拍、想改成"松开"用 4 拍。
 *         只要有一拍与判断一致，反对票立刻清零。
 */
static uint8_t deb_update(deb_t *d, uint8_t raw_pressed)
{
    uint8_t threshold = raw_pressed ? db_press_ticks : db_release_ticks;

    if (raw_pressed == d->stable)
    {
        d->count = 0;
    }
    else if (++d->count >= threshold)
    {
        d->stable = raw_pressed;
        d->count  = 0;
    }

    return d->stable;
}

/**
 * @brief  十字键编码成 Hat 值（0~7，8 = 中立）
 * @note   相反方向同按必须特判回中立，否则斜向判断会先命中，出现"同时按上下却报右上"。
 */
static uint8_t hat_encode(uint8_t up, uint8_t down, uint8_t left, uint8_t right)
{
    if ((up && down) || (left && right)) return 8;
    if (up    && right) return 1;
    if (down  && right) return 3;
    if (down  && left)  return 5;
    if (up    && left)  return 7;
    if (up)             return 0;
    if (right)          return 2;
    if (down)           return 4;
    if (left)           return 6;
    return 8;
}

void scan_buttons(void)
{
    /* 十字键：4 个触点各自消抖 -> 编码成 Hat -> byte2 */
    uint8_t up    = deb_update(&db_up,    BTN_DOWN(up_GPIO_Port,    up_Pin));
    uint8_t down  = deb_update(&db_down,  BTN_DOWN(down_GPIO_Port,  down_Pin));
    uint8_t left  = deb_update(&db_left,  BTN_DOWN(left_GPIO_Port,  left_Pin));
    uint8_t right = deb_update(&db_right, BTN_DOWN(right_GPIO_Port, right_Pin));

    uint8_t hat_new = hat_encode(up, down, left, right);
    if (hat_new != gamepad_report[2]) count_pov++;
    gamepad_report[2] = hat_new;

    /* 面键 A/B/X/Y -> byte0 低 4 位。用 |= / &= ~，不影响同字节其他位 */
    uint8_t a_old = db_a.stable;                                       /* 更新【前】*/
    uint8_t a_new = deb_update(&db_a, BTN_DOWN(A_GPIO_Port, A_Pin));   /* 更新【后】*/

    if (a_new && !a_old) count_a++;     /* 只在"抬起 -> 按下"这一拍计数 */
    if (a_new) gamepad_report[0] |=  BTN_A;
    else       gamepad_report[0] &= ~BTN_A;

    if (deb_update(&db_b, BTN_DOWN(B_GPIO_Port, B_Pin))) gamepad_report[0] |=  BTN_B;
    else                                                 gamepad_report[0] &= ~BTN_B;

    if (deb_update(&db_x, BTN_DOWN(X_GPIO_Port, X_Pin))) gamepad_report[0] |=  BTN_X;
    else                                                 gamepad_report[0] &= ~BTN_X;

    if (deb_update(&db_y, BTN_DOWN(Y_GPIO_Port, Y_Pin))) gamepad_report[0] |=  BTN_Y;
    else                                                 gamepad_report[0] &= ~BTN_Y;

    /* 摇杆按下 L3 / R3 -> byte1 的 bit0 / bit1 */
    uint8_t l3_old = db_l3.stable;
    uint8_t l3_new = deb_update(&db_l3, BTN_DOWN(SW1_GPIO_Port, SW1_Pin));

    if (l3_new && !l3_old) count_l3++;
    if (l3_new) gamepad_report[1] |=  BTN_L3;
    else        gamepad_report[1] &= ~BTN_L3;

    uint8_t r3_old = db_r3.stable;
    uint8_t r3_new = deb_update(&db_r3, BTN_DOWN(SW2_GPIO_Port, SW2_Pin));

    if (r3_new && !r3_old) count_r3++;
    if (r3_new) gamepad_report[1] |=  BTN_R3;
    else        gamepad_report[1] &= ~BTN_R3;

    /* 肩键 LB / RB -> byte0 的 bit4 / bit5。
       引脚 PA15 / PB4 原本是 JTAG 脚，靠 __HAL_AFIO_REMAP_SWJ_NOJTAG() 释放。 */
    uint8_t lb_old = db_lb.stable;
    uint8_t lb_new = deb_update(&db_lb, BTN_DOWN(LB_GPIO_Port, LB_Pin));

    if (lb_new && !lb_old) count_lb++;
    if (lb_new) gamepad_report[0] |=  BTN_LB;
    else        gamepad_report[0] &= ~BTN_LB;

    uint8_t rb_old = db_rb.stable;
    uint8_t rb_new = deb_update(&db_rb, BTN_DOWN(RB_GPIO_Port, RB_Pin));

    if (rb_new && !rb_old) count_rb++;
    if (rb_new) gamepad_report[0] |=  BTN_RB;
    else        gamepad_report[0] &= ~BTN_RB;

    /* View / Menu -> byte0 的 bit6 / bit7。
       ⚠️ PA9 / PA10 原本是 USART1_TX/RX，所以串口现在不可用（将来要串口得换脚）。*/
    if (deb_update(&db_view, BTN_DOWN(VIEW_GPIO_Port, VIEW_Pin))) gamepad_report[0] |=  BTN_VIEW;
    else                                                          gamepad_report[0] &= ~BTN_VIEW;

    if (deb_update(&db_menu, BTN_DOWN(MENU_GPIO_Port, MENU_Pin))) gamepad_report[0] |=  BTN_MENU;
    else                                                          gamepad_report[0] &= ~BTN_MENU;
}
