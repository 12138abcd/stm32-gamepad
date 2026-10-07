/**
  ******************************************************************************
  * @file    button_front.c
  * @brief   扫描 8 个前部按键（十字键 + A/B/X/Y）和两个摇杆按下（SW1/SW2），更新 HID 报告。
  ******************************************************************************
  */

#include "button_front.h"
#include "main.h"
#include "report.h"
#include "usbd_customhid.h"

/* ============================ 消抖参数 ============================
   单位是"扫描拍数"，一拍 = 1ms（scan_buttons 每 1ms 被调用一次）。
   按下要快、松开可以慢，因为人对按下的延迟敏感得多。

   ⚠️ 这两个值【故意写成 volatile 变量而不是宏】，是为了做"消抖参数
   对延迟的影响"实验：直接用 Keil 的 Watch 窗口改值即可，**不用重新
   编译烧写**，两轮实验能在同一次运行里对比，测试条件完全一致。 */
volatile uint8_t db_press_ticks   = 1u;   /* 按下：连续几拍一致就认 —— 参数【定稿值】

   这个值是实测选出来的，不是拍脑袋定的。两轮对照实验（各按 100 次，
   只统计"按下"的延迟，其他条件完全一致）：

     db_press_ticks |  P50   |  P95   |  P99   | min   | 误触发
     ---------------+--------+--------+--------+-------+--------
          2 拍      | 1590µs | 1960µs | 2000µs | 1031µs|   0
          1 拍      |  520µs |  970µs | 1020µs |   26µs|   0

   关键证据是 min：26µs vs 1031µs，差值 1005µs ≈ 正好 1 个 1ms 节拍 ——
   说明"每多 1 拍消抖 = 固定多 1ms 延迟"，是量出来的，不是估的。
   两轮误触发都是 0，所以取延迟更低的 1 拍。

   ⚠️ 1 拍对抗【电气噪声】的裕度比 2 拍小。当前布线（杜邦线）实测
   100 次零误触发，裕度够；如果以后换成长线或强干扰环境，可以退回 2 拍。*/
volatile uint8_t db_release_ticks = 4u;   /* 松开：连续几拍一致才认（默认 4）*/

/* 按键是上拉输入：按下时引脚被拉到地，读到 0 */
#define BTN_DOWN(port, pin)   (HAL_GPIO_ReadPin((port), (pin)) == GPIO_PIN_RESET)

/* ============================ 消抖状态机 ============================
   每个按键一份：stable 是"目前的判断"，count 是"连续多少拍和判断不一致"。 */
typedef struct {
    uint8_t stable;   /* 消抖后的稳定状态：1 = 按下，0 = 松开 */
    uint8_t count;    /* 连续读到"和 stable 不一样"的拍数 */
} deb_t;

static deb_t db_up, db_down, db_left, db_right;   /* 十字键 4 个触点 */
static deb_t db_a,  db_b,    db_x,    db_y;       /* 4 个面键 */
static deb_t db_l3;                               /* 左摇杆按下（SW1 → PB9） */
static deb_t db_r3;                               /* 右摇杆按下（SW2 → PA8） */

/* ---- 调试用计数器：加 volatile，否则会被优化掉，调试器里永远是 0 ---- */
volatile uint32_t count_a   = 0;   /* A 键被按下的次数 */
volatile uint32_t count_pov = 0;   /* 十字键方向变化的次数 */
volatile uint32_t count_l3  = 0;   /* 左摇杆被按下的次数 */
volatile uint32_t count_r3  = 0;   /* 右摇杆被按下的次数 */

/* 非阻塞消抖：每拍调一次，不等待、不阻塞。
   只有"和当前判断矛盾的读数"连续出现够多拍，才改判。 */
static uint8_t deb_update(deb_t *d, uint8_t raw_pressed)
{
    /* 三目运算符：读到按下就用"按下阈值"，否则用"松开阈值" */
    uint8_t threshold = raw_pressed ? db_press_ticks : db_release_ticks;

    if (raw_pressed == d->stable)
    {
        d->count = 0;                        /* 和当前判断一致 → 账目清零 */
    }
    else if (++d->count >= threshold)
    {
        d->stable = raw_pressed;             /* 连续够多拍都反对 → 改判 */
        d->count  = 0;
    }

    return d->stable;
}

/* ============================ 十字键编码 ============================
   把"哪几个方向被按下"翻译成一个 Hat 数值（0~7，8 = 松开）。
   纯函数：没有状态，输入一样输出就一样。 */
static uint8_t hat_encode(uint8_t up, uint8_t down, uint8_t left, uint8_t right)
{
    if ((up && down) || (left && right)) return 8;   /* 相反方向同按 → 回中立 */
    if (up    && right) return 1;                    /* 右上 */
    if (down  && right) return 3;                    /* 右下 */
    if (down  && left)  return 5;                    /* 左下 */
    if (up    && left)  return 7;                    /* 左上 */
    if (up)             return 0;                    /* 上 */
    if (right)          return 2;                    /* 右 */
    if (down)           return 4;                    /* 下 */
    if (left)           return 6;                    /* 左 */
    return 8;                                        /* 什么都没按 */
}

/* ============================ 主扫描函数 ============================ */
void scan_buttons(void)
{
    /* ---------- 第一步：十字键四个方向各自消抖 ---------- */
    uint8_t up    = deb_update(&db_up,    BTN_DOWN(up_GPIO_Port,    up_Pin));
    uint8_t down  = deb_update(&db_down,  BTN_DOWN(down_GPIO_Port,  down_Pin));
    uint8_t left  = deb_update(&db_left,  BTN_DOWN(left_GPIO_Port,  left_Pin));
    uint8_t right = deb_update(&db_right, BTN_DOWN(right_GPIO_Port, right_Pin));

    /* ---------- 第二步：编码成 Hat 值，写进报告的 byte 2 ---------- */
    uint8_t hat_new = hat_encode(up, down, left, right);
    if (hat_new != gamepad_report[2]) count_pov++;   /* 方向变了就记一笔 */
    gamepad_report[2] = hat_new;

    /* ---------- 面键 A / B / X / Y → 报告 byte 0 的低 4 位 ----------
       置位用 |=，清位用 &= ~，这样不会影响同一个字节里的其他位。 */
    uint8_t a_old = db_a.stable;                                       /* 更新【前】的判断 */
    uint8_t a_new = deb_update(&db_a, BTN_DOWN(A_GPIO_Port, A_Pin));   /* 更新【后】的判断 */

    if (a_new && !a_old) count_a++;     /* 只在"抬起 → 按下"的这一拍计数 */
    if (a_new) gamepad_report[0] |=  BTN_A;
    else       gamepad_report[0] &= ~BTN_A;//使用A来调试`

    if (deb_update(&db_b, BTN_DOWN(B_GPIO_Port, B_Pin))) gamepad_report[0] |=  BTN_B;
    else                                                 gamepad_report[0] &= ~BTN_B;

    if (deb_update(&db_x, BTN_DOWN(X_GPIO_Port, X_Pin))) gamepad_report[0] |=  BTN_X;
    else                                                 gamepad_report[0] &= ~BTN_X;

    if (deb_update(&db_y, BTN_DOWN(Y_GPIO_Port, Y_Pin))) gamepad_report[0] |=  BTN_Y;
    else                                                 gamepad_report[0] &= ~BTN_Y;

    /* ---------- 左摇杆按下（SW1）→ 报告 byte 1 的 bit0，即 L3 / Button 9 ----------
       注意这时写的是 gamepad_report[1]，不是 [0] —— byte1 的 bit0 在 HID 里编号是"按钮 9"。
       接法跟普通按键完全一样：模块的 SW 脚接 PB9，内部上拉，按下读到 0。 */
    uint8_t l3_old = db_l3.stable;
    uint8_t l3_new = deb_update(&db_l3, BTN_DOWN(SW1_GPIO_Port, SW1_Pin));

    if (l3_new && !l3_old) count_l3++;
    if (l3_new) gamepad_report[1] |=  BTN_L3;
    else        gamepad_report[1] &= ~BTN_L3;

    /* ---------- 右摇杆按下（SW2）→ 报告 byte 1 的 bit1，即 R3 / Button 10 ----------
       ⚠️ 是 bit1（BTN_R3 = 0x02），不是 bit0 —— L3 占的是 bit0。
       接法与 L3 完全相同：模块的 SW 脚接 PA8，内部上拉，按下读到 0。
       GPIO 初始化在 main.c 的 MX_GPIO_Init() 里（PA8 = SW2 + PullUp）。 */
    uint8_t r3_old = db_r3.stable;
    uint8_t r3_new = deb_update(&db_r3, BTN_DOWN(SW2_GPIO_Port, SW2_Pin));

    if (r3_new && !r3_old) count_r3++;
    if (r3_new) gamepad_report[1] |=  BTN_R3;
    else        gamepad_report[1] &= ~BTN_R3;
}