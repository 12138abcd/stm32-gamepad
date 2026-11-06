/**
 * @file    wdt.h
 * @brief   双层看门狗：IWDG（防死锁）+ WWDG（防跑飞 / 丢节拍）
 *
 * ============================ 两层各管什么 ============================
 *
 *   IWDG  1.5s（LSI 40kHz，独立于主时钟）
 *     喂狗条件：1ms 任务里的监督逻辑确认【所有已武装任务的心跳都新鲜】。
 *     抓：任务卡死 / 优先级反转饿死 / HSE+PLL 挂了（LSI 照样跑）。
 *     ⚠️ 喂狗只在任务里，绝不放中断 —— 放中断里主循环死了照样喂，等于没有。
 *
 *   WWDG  58.25ms（PCLK1 36MHz，窗口 14.56ms）
 *     喂狗节奏：1ms 任务按 HAL_GetTick 判断，距上次 ≥20ms 喂一次（时间基）。
 *     抓：喂早了（<14.56ms，跑飞/误喂）→ 窗口复位；
 *         喂晚了（循环卡住 >58.25ms）→ 超时复位。
 *
 * ============================ 实测参数 ============================
 *   IWDG：PR=/32 → 1250Hz，RLR=1875 → 1.5s（LSI 30~60kHz → 1.0~2.0s）
 *   WWDG：WDGTB=/8 → 0.9102ms/拍
 *         0x7F→0x3F 共 64 拍 = 58.25ms 超时
 *         窗口 0x6F：至少过 16 拍（14.56ms）才允许喂
 *         20ms 喂一次：早 5.4ms / 晚 38ms 余量
 *
 * ============================ 为什么用时间基、不用循环计数 ============================
 *   F1 擦 Flash 时整个 CPU 都会 stall（不只是发起擦除的那个任务）——
 *   计数基会把“擦除的 40ms”当成“没到 20 拍”，恢复后节奏全乱，甚至直接超时。
 *   所以 WWDG 按 HAL_GetTick 判断“距上次喂 ≥20ms”。
 *   擦除前用 wdt_flash_begin() 等到窗口内并喂满，擦除后用
 *   wdt_flash_end() 立刻补喂 —— 40ms 擦除仍落在 58ms 预算里。
 *
 * ============================ 复位后能查到什么 ============================
 *   wdt_reset_cause   本次复位原因（WDT_CAUSE_*）
 *   wdt_reset_flags   RCC->CSR 原始值（IWDG/WWDG/POR/PIN/SFT 位都在）
 *   wdt_boot_count    上电以来第几次启动（BKP 备份寄存器，复位不清零）
 *   wdt_prev_cause    上一次复位原因（同上）
 *   wdt_prev_stall    上一次是哪个任务心跳停了（bit0=1ms 任务，bit1=appTask）
 *   wdt_stall_count   运行期检出心跳超时的次数
 *
 * ============================ 故障注入（验收用）============================
 *   WDT_FAULT_TEST = 0  正常
 *   WDT_FAULT_TEST = 1  跑到第 300 拍时死循环      → 预期 WWDG 复位（~58ms）
 *   WDT_FAULT_TEST = 2  每拍都喂 WWDG（早于窗口） → 预期 WWDG 复位（立刻）
 *   WDT_FAULT_TEST = 3  不开 WWDG、不喂 IWDG      → 预期 IWDG 复位（~1.5s）
 *
 *   三档都改宏、重编译、烧写；复位后看 wdt_boot_count +1、
 *   wdt_reset_cause = 5（WWDG）或 4（IWDG）。
 */

#ifndef WDT_H
#define WDT_H

#include <stdint.h>

/* ============================ 故障注入 ============================ */
#define WDT_FAULT_TEST   0

/* ============================ 任务编号 ============================ */
#define WDT_TASK_INPUT   0u   /* 1ms 输入任务（defaultTask） */
#define WDT_TASK_APP     1u   /* 应用任务（手柄 = 无线发送；dongle = 参数落盘） */
#define WDT_TASK_MAX     2u

/* ============================ 时间参数 ============================ */
#define WDT_IWDG_FEED_MS      100u    /* 健康时每 100ms 喂一次 IWDG */
#define WDT_HB_TIMEOUT_MS     200u    /* 某任务心跳超过 200ms 没动 = 卡住 */
#define WDT_WWDG_FEED_MS      20u     /* 距上次喂 ≥20ms 喂一次 WWDG（时间基） */
#define WDT_WWDG_WINDOW_MS    15u     /* 窗口内侧 14.56ms，取 15ms 保守 */

/* ============================ 复位原因 ============================ */
#define WDT_CAUSE_UNKNOWN  0u
#define WDT_CAUSE_POR      1u   /* 上电 */
#define WDT_CAUSE_PIN      2u   /* NRST 按键 / ST-Link 复位 */
#define WDT_CAUSE_SOFT     3u   /* NVIC_SystemReset */
#define WDT_CAUSE_IWDG     4u   /* 独立看门狗 */
#define WDT_CAUSE_WWDG     5u   /* 窗口看门狗 */
#define WDT_CAUSE_LPWR     6u   /* 低功耗 */

/* ============================ 诊断（Keil Watch 可见）============================ */
extern volatile uint32_t wdt_reset_flags;   /* RCC->CSR 原始值 */
extern volatile uint8_t  wdt_reset_cause;   /* WDT_CAUSE_* */
extern volatile uint8_t  wdt_prev_cause;    /* 上一次复位原因 */
extern volatile uint8_t  wdt_prev_stall;    /* 上一次停住的任务掩码 */
extern volatile uint32_t wdt_boot_count;    /* 启动次数（BKP，掉电才清零）*/
extern volatile uint8_t  wdt_stall_mask;    /* 当前停住的任务掩码 */
extern volatile uint32_t wdt_stall_count;   /* 检出心跳超时的累计次数 */
extern volatile uint32_t wdt_iwdg_feeds;    /* IWDG 喂狗次数 */
extern volatile uint32_t wdt_wwdg_feeds;    /* WWDG 喂狗次数 */
extern volatile uint32_t wdt_cycles;        /* 1ms 循环计数 */

/* ============================ API ============================ */

/**
 * @brief  上电尽早调用：读复位原因 + 记启动次数 + 开 IWDG
 * @note   放在 SystemClock_Config() 之后、外设初始化之前。
 *         IWDG 一旦启动就无法关闭（只能复位），所以从这行起 1s 内必须开始喂。
 */
void wdt_init(void);

/**
 * @brief  开 WWDG（第二层）
 * @note   必须在 USB 初始化之后、1ms 循环之前调用 —— USB 初始化可能几十 ms，
 *         早开会被 58ms 超时误复位。
 */
void wdt_wwdg_start(void);

/** @brief 标记一个任务进入主循环（未武装的任务不参与心跳检查） */
void wdt_task_arm(uint8_t id);

/** @brief 任务心跳（每个循环 +1，监督逻辑据此判断任务是否还在跑） */
void wdt_heartbeat(uint8_t id);

/**
 * @brief  1ms 循环末尾调用一次：心跳监督 + 喂两层狗
 * @note   必须在“本轮活全干完”之后调用；放在循环开头等于没检查。
 */
void wdt_loop_hook(void);

/**
 * @brief  Flash 擦除/编程【之前】调用：等到窗口内并喂满 WWDG
 * @note   擦除会 stall 整个 CPU 20~40ms；只有先喂满，58ms 预算才够。
 *         内部可能 HAL_Delay 最多 15ms（为了等窗口内侧），只在写 Flash 时调。
 */
void wdt_flash_begin(void);

/**
 * @brief  Flash 擦除/编程【之后】调用：立刻补喂一次
 * @note   此时距上次喂约 40ms，仍在窗口内（>14.56ms）且未超时（<58ms）。
 */
void wdt_flash_end(void);

#endif /* WDT_H */
