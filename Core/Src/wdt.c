/**
 * @file    wdt.c
 * @brief   双层看门狗实现（IWDG 1s + WWDG 58ms 窗口）
 *
 * 为什么直接写寄存器、不用 HAL 的 IWDG/WWDG 模块：
 *   这两个外设的 HAL 封装就是几行寄存器操作，但要多编译两个 .c 文件、
 *   改 hal_conf.h、改 Keil 工程 —— 收益不成比例。这里一次写清楚，
 *   面试时也能直接讲寄存器时序。
 *
 * 寄存器时序依据：STM32F10x 参考手册 RM0008
 *   IWDG：KR=0xCCCC 启动 → KR=0x5555 允许写 PR/RLR → 写 PR/RLR
 *         → 等 SR.PVU/RVU 清零 → KR=0xAAAA 重装
 *   WWDG：CFR 设 WDGTB 和窗口 W → CR 写 WDGA+计数值启动
 *         之后每次喂狗都是重写 CR = WDGA | 0x7F
 *
 * @note 参数换算、两层分工、故障注入怎么测 —— 详见《固件代码导读》16。
 */

#include "wdt.h"
#include "main.h"      /* stm32f1xx_hal.h：BKP / RCC / DBGMCU / HAL_GetTick */

/* ============================ IWDG 键值 ============================ */
#define IWDG_KEY_RELOAD   0x0000AAAAu
#define IWDG_KEY_ENABLE   0x0000CCCCu
#define IWDG_KEY_WRITE    0x00005555u

/* ============================ WWDG 参数 ============================
   PCLK1 = 36MHz，WDGTB = 11 (/8)：
     WWDG 时钟 = 36MHz / (4096 × 8) = 1098.6Hz → 0.9102ms/拍
     超时     = (0x7F − 0x3F) × 0.9102 = 58.25ms
     窗口 0x6F：至少过 16 拍（14.56ms）才允许喂
     每 20ms 喂一次：早 5.4ms / 晚 38ms 余量
   ⚠️ 改这几个数要同步改 wdt.h 注释和导读-16。 */
#define WWDG_CFR_PRESCALER_8   (3u << 7)   /* WDGTB[1:0] = 11 → /8 */
#define WWDG_CFR_WINDOW        0x6Fu
/* WDGA 直接用 CMSIS 的 WWDG_CR_WDGA（0x80），不重复定义 */
#define WWDG_CR_COUNTER        0x7Fu

/* ============================ BKP 备份寄存器分配 ============================
   DR1 = magic（判断备份域是不是第一次上电）
   DR2 = 启动次数
   DR3 = 上一次的复位原因
   DR4 = 上一次卡住的任务掩码
   ⚠️ BKP 在备份域，普通复位不清零；只有掉电（无 VBAT）才丢。 */
#define WDT_BKP_MAGIC   0x5754u   /* 'W''T' */

/* ============================ 诊断量 ============================ */
volatile uint32_t wdt_reset_flags = 0u;
volatile uint8_t  wdt_reset_cause = WDT_CAUSE_UNKNOWN;
volatile uint8_t  wdt_prev_cause  = WDT_CAUSE_UNKNOWN;
volatile uint8_t  wdt_prev_stall  = 0u;
volatile uint32_t wdt_boot_count  = 0u;
volatile uint8_t  wdt_stall_mask  = 0u;
volatile uint32_t wdt_stall_count = 0u;
volatile uint32_t wdt_iwdg_feeds  = 0u;
volatile uint32_t wdt_wwdg_feeds  = 0u;
volatile uint32_t wdt_cycles      = 0u;

/* ============================ 内部状态 ============================ */
typedef struct
{
    volatile uint32_t hb;      /* 心跳计数（任务每圈 +1） */
    volatile uint8_t  armed;   /* 1 = 该任务已进入主循环 */
} wdt_slot_t;

static wdt_slot_t s_slot[WDT_TASK_MAX];
static uint32_t   s_seen[WDT_TASK_MAX];      /* 上次看到的心跳值 */
static uint32_t   s_seen_ms[WDT_TASK_MAX];   /* 上次心跳变化的时间 */
static uint32_t   s_iwdg_last_ms = 0u;       /* 上次喂 IWDG 的时间 */
static uint32_t   s_wwdg_last_ms = 0u;       /* 上次喂 WWDG 的时间 */
static uint8_t    wdt_ready      = 0u;       /* IWDG 已启动 */
static uint8_t    wdt_wwdg_on    = 0u;       /* WWDG 已启动 */

/* ============================ 复位原因判定 ============================ */
static uint8_t reset_cause_from_flags(uint32_t flags)
{
    /* 顺序有讲究：POR 会连带置 PINRSTF，所以 POR 必须排在 PIN 前面；
       IWDG/WWDG/SFT 优先级最高（它们发生时就一定有明确原因）。 */
    if ((flags & RCC_CSR_IWDGRSTF) != 0u) { return WDT_CAUSE_IWDG; }
    if ((flags & RCC_CSR_WWDGRSTF) != 0u) { return WDT_CAUSE_WWDG; }
    if ((flags & RCC_CSR_SFTRSTF)  != 0u) { return WDT_CAUSE_SOFT; }
    if ((flags & RCC_CSR_PORRSTF)  != 0u) { return WDT_CAUSE_POR;  }
    if ((flags & RCC_CSR_PINRSTF)  != 0u) { return WDT_CAUSE_PIN;  }
    if ((flags & RCC_CSR_LPWRRSTF) != 0u) { return WDT_CAUSE_LPWR; }
    return WDT_CAUSE_UNKNOWN;
}

/* ============================ BKP 访问 ============================ */
static void bkp_enable(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
}

/* ============================ IWDG 启动 ============================ */
static void iwdg_start(void)
{
    uint32_t t0;

    IWDG->KR = IWDG_KEY_ENABLE;    /* 启动；LSI 自动打开 */

    /* LSI 起振需要时间（约 40µs）。这里最多等 10ms，超时就照常配置 ——
       1s 的超时足够兜住这点偏差。 */
    t0 = HAL_GetTick();
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0u)
    {
        if ((uint32_t)(HAL_GetTick() - t0) > 10u) { break; }
    }

    IWDG->KR = IWDG_KEY_WRITE;     /* 允许写 PR / RLR */
    IWDG->PR = 3u;                 /* 预分频 /32 → 1250Hz */
    IWDG->RLR = 1875u;             /* 1875 / 1250Hz = 1.5s（LSI 30~60kHz → 1.0~2.0s）*/

    /* PR/RLR 是同步到 LSI 域的，写完要等 SR 的两个 update 标志清零 */
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u) { }

    IWDG->KR = IWDG_KEY_RELOAD;    /* 立即重装，从现在开始算 1s */
}

/* ============================ 对外接口 ============================ */

void wdt_init(void)
{
    uint32_t flags;

    bkp_enable();

    /* 备份域第一次上电（magic 不对）：清掉旧内容，避免 BKP 里是随机值 */
    if (((uint16_t)BKP->DR1) != (uint16_t)WDT_BKP_MAGIC)
    {
        BKP->DR1 = (uint16_t)WDT_BKP_MAGIC;
        BKP->DR2 = 0u;
        BKP->DR3 = (uint16_t)WDT_CAUSE_UNKNOWN;
        BKP->DR4 = 0u;
    }

    wdt_boot_count = (uint32_t)BKP->DR2 + 1u;
    BKP->DR2 = (uint16_t)(wdt_boot_count & 0xFFFFu);

    /* 上一次启动留下的现场（本次启动前写入的） */
    wdt_prev_cause = (uint8_t)(BKP->DR3 & 0xFFu);
    wdt_prev_stall = (uint8_t)(BKP->DR4 & 0xFFu);

    /* 本次复位原因：先读、再清，否则下次就看不到了 */
    flags = RCC->CSR;
    wdt_reset_flags = flags;
    wdt_reset_cause = reset_cause_from_flags(flags);

    BKP->DR3 = (uint16_t)wdt_reset_cause;
    BKP->DR4 = 0u;
    RCC->CSR |= RCC_CSR_RMVF;      /* 清所有复位标志 */

    /* 调试时 halt 不让狗咬：三条都要，否则一开 Watch 就复位 */
    __HAL_DBGMCU_FREEZE_IWDG();
    __HAL_DBGMCU_FREEZE_WWDG();
    __HAL_DBGMCU_FREEZE_TIM4();    /* HAL 时基也停，避免恢复后心跳时间戳假性超时 */

    iwdg_start();

    s_iwdg_last_ms = HAL_GetTick();
    wdt_ready = 1u;
}

void wdt_wwdg_start(void)
{
#if WDT_FAULT_TEST == 3
    /* 只测 IWDG：不开 WWDG，否则 58ms 就先复位了 */
    return;
#else
    __HAL_RCC_WWDG_CLK_ENABLE();

    WWDG->CFR = WWDG_CFR_PRESCALER_8 | WWDG_CFR_WINDOW;
    WWDG->CR  = WWDG_CR_WDGA | WWDG_CR_COUNTER;

    s_wwdg_last_ms = HAL_GetTick();
    wdt_wwdg_on = 1u;
#endif
}

void wdt_task_arm(uint8_t id)
{
    if (id >= WDT_TASK_MAX) { return; }

    if (s_slot[id].armed == 0u)
    {
        s_slot[id].armed = 1u;
        s_seen[id]       = s_slot[id].hb;
        s_seen_ms[id]    = HAL_GetTick();
    }
}

void wdt_heartbeat(uint8_t id)
{
    if (id >= WDT_TASK_MAX) { return; }
    s_slot[id].hb++;
}

void wdt_loop_hook(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  stall = 0u;
    uint8_t  i;

    wdt_cycles++;

#if WDT_FAULT_TEST == 1
    /* 模拟死循环：跑到第 300 拍就停住。WWDG 应在 ~58ms 后复位。 */
    if (wdt_cycles == 300u)
    {
        __disable_irq();
        for (;;) { __NOP(); }
    }
#endif

    /* ---- 心跳监督：任何一个已武装任务超过 200ms 没心跳，就扣住 IWDG ---- */
    for (i = 0u; i < WDT_TASK_MAX; i++)
    {
        if (s_slot[i].armed == 0u) { continue; }

        if (s_slot[i].hb != s_seen[i])
        {
            s_seen[i]    = s_slot[i].hb;
            s_seen_ms[i] = now;
        }
        else if ((uint32_t)(now - s_seen_ms[i]) > WDT_HB_TIMEOUT_MS)
        {
            stall |= (uint8_t)(1u << i);
        }
    }

    if (stall != 0u)
    {
        /* 有任务卡死：不喂 IWDG（1s 内复位），但 WWDG 照常喂 ——
           这样复位原因能准确区分“死锁(IWDG)”和“循环卡住(WWDG)”。 */
        if (wdt_stall_mask != stall)
        {
            wdt_stall_mask = stall;
            wdt_stall_count++;
            HAL_PWR_EnableBkUpAccess();
            BKP->DR4 = (uint16_t)stall;   /* 复位后 wdt_prev_stall 能读到 */
        }
    }
    else
    {
        wdt_stall_mask = 0u;

#if WDT_FAULT_TEST != 3
        if ((wdt_ready != 0u) && ((uint32_t)(now - s_iwdg_last_ms) >= WDT_IWDG_FEED_MS))
        {
            s_iwdg_last_ms = now;
            IWDG->KR = IWDG_KEY_RELOAD;
            wdt_iwdg_feeds++;
        }
#endif
    }

#if WDT_FAULT_TEST == 2
    /* 故意每拍都喂：1ms < 窗口 14.56ms → WWDG 应立刻复位 */
    WWDG->CR = WWDG_CR_WDGA | WWDG_CR_COUNTER;
    wdt_wwdg_feeds++;
#else
    /* 时间基喂狗：擦 Flash 会让整个 CPU stall，计数基会把 40ms 擦除
       算成“没到 20 拍”，恢复后节奏全乱。用 tick 判断最稳。 */
    if ((wdt_wwdg_on != 0u) && ((uint32_t)(now - s_wwdg_last_ms) >= WDT_WWDG_FEED_MS))
    {
        WWDG->CR = WWDG_CR_WDGA | WWDG_CR_COUNTER;
        s_wwdg_last_ms = now;
        wdt_wwdg_feeds++;
    }
#endif
}

void wdt_flash_begin(void)
{
    uint32_t elapsed;

    if (wdt_wwdg_on == 0u) { return; }

    /* 距上次喂不足窗口（14.56ms）就等一等 —— 早喂会被窗口复位。
       等完喂满，把整整 58ms 预算留给即将到来的擦除。 */
    elapsed = (uint32_t)(HAL_GetTick() - s_wwdg_last_ms);
    if (elapsed < WDT_WWDG_WINDOW_MS)
    {
        HAL_Delay(WDT_WWDG_WINDOW_MS - elapsed + 1u);
    }

    WWDG->CR = WWDG_CR_WDGA | WWDG_CR_COUNTER;
    s_wwdg_last_ms = HAL_GetTick();
    wdt_wwdg_feeds++;
}

void wdt_flash_end(void)
{
    if (wdt_wwdg_on == 0u) { return; }

    /* 擦除/编程最多 40ms，这里补喂一次，绝不能让它超时 */
    WWDG->CR = WWDG_CR_WDGA | WWDG_CR_COUNTER;
    s_wwdg_last_ms = HAL_GetTick();
    wdt_wwdg_feeds++;
}
