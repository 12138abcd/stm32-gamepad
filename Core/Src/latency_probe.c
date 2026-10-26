/**
  ******************************************************************************
  * @file    latency_probe.c
  * @brief   USB 输入延迟测量 —— 在固件内部测「输入电平变化 → 报告发上总线」。
  *
  * ============================ 它测什么 ============================
  * 起点：固件【第一次读到】按键引脚的原始电平变化（未经消抖）
  * 终点：这一帧报告真正发上 USB 总线（端点 DataIn 回调触发的那一刻）
  *
  * 所以测出来的是完整的【设备侧 input latency】：
  *     轮询量化(0~1ms) + 消抖(按下 1 拍) + 节拍对齐 + 协议栈 + USB 帧等待(0~1ms)
  *
  * 只统计【按下】的样本 —— 松开阈值是 4 拍（防误松开），而松手晚几毫秒
  * 没人感知得到；业界测手柄 input lag 也只测按下。
  *
  * ============================ 靠什么测 ============================
  * Cortex-M3 的 DWT->CYCCNT —— 跟着内核时钟自由运行的周期计数器，
  * 72MHz 下分辨率 13.9ns（比 24MHz 逻辑分析仪的 41.7ns 还精确）。
  * 完全在固件内部完成，不需要任何外部仪器；结果按 10µs 分桶累加成直方图，
  * 并实时算出 P50 / P95 / P99，直接在 Keil 的 Watch 窗口里读。
  *
  * ======================== 逻辑分析仪接法（可选）====================
  * 想看波形的话：分析仪 CH1 → 按键引脚（如 PB5 = A 键）
  *              分析仪 CH2 → PB10（打点脚）
  *              分析仪 GND → 板子 GND
  * CH2 上是一个负脉冲：下降沿 = 报告变化已确认，上升沿 = 已发上总线。
  ******************************************************************************
  */

#include "latency_probe.h"
#include "main.h"              /* 按键引脚宏 + DWT/CoreDebug */
#include "report.h"            /* gamepad_report[] */
#include "usbd_customhid.h"    /* CUSTOM_HID_EPIN_ADDR */
#include "app.h"               /* DEVICE_ROLE + app_is_wired()（第 8 步）*/
#include "nrf24.h"             /* NRF24_TX_OK / NRF24_TX_MAXRT（第 8 步）*/
#include <string.h>            /* memcmp / memcpy */

/* 第 8 步：t1 的出口（手柄侧）。放总开关【外面】—— 关掉探针时它恒 0，
   wireless_tx.c 照样能把它传给 proto_pack_ex，不用加条件编译。 */
volatile uint16_t wprobe_t1_out = 0u;

#if LATENCY_PROBE_ENABLE

/* ============================ 参数 ============================ */

/* 直方图的桶设置。按下延迟主要在 0~2ms 这个量级，所以桶宽 10µs、量程 0~5000µs
   留足了余量（超量程会记进 probe_hist_over，正常应该恒为 0）。
   500 个桶 × 2 字节 = 1000 字节，F103 的 20KB SRAM 放得下。 */
#define PROBE_BUCKET_US   10u    /* 每个桶 10 微秒 */
#define PROBE_BUCKETS     500u   /* 0 ~ 5000 微秒 */

/* 一次测量完成后的"封锁期"：期间不再武装新的测量。
   机械按键按下时有 5~10ms 的抖动，会让原始电平反复跳变；不封锁的话
   一次按键会被当成好几次测量。20ms 比最长抖动还长，而人最快也就
   每秒按十几次（间隔 60ms+），所以不会漏掉真实操作。 */
#define PROBE_LOCKOUT_CYCLES  (SystemCoreClock / 50u)   /* 20ms 对应的周期数 */

/* 武装超时：一次测量武装后，若 50ms 内还没等到终点就放弃。
   正常延迟只有 0~2ms，所以 50ms 足够宽松；它的作用是兜底 ——
   防止"武装挂着没人消费"，在很久之后被下一次报告变化误消费掉，
   测出一个假的巨大值（实测踩过一次，测出了 139ms）。 */
#define PROBE_ARM_TIMEOUT_CYCLES  (SystemCoreClock / 20u)   /* 50ms 对应的周期数 */

/* ============================ 统计量 ============================ */

static uint8_t    probe_prev[9];                  /* 上一拍的报告内容 */
volatile uint32_t probe_fall        = 0;          /* 打点脚被拉低的次数 */
volatile uint32_t probe_rise        = 0;          /* 打点脚被拉高的次数 */
volatile uint32_t probe_datain_any  = 0;          /* DataIn 回调总调用次数 */
volatile uint32_t probe_datain_ep1  = 0;          /* 其中 epnum == 1 的次数 */

volatile uint16_t probe_hist[PROBE_BUCKETS];      /* 直方图（下标 = 延迟 / 10µs）*/
volatile uint16_t probe_hist_over = 0;            /* 超量程次数，应该 0 */
volatile uint32_t probe_count     = 0;            /* 有效样本数（只算按下）*/
volatile uint32_t probe_max_us    = 0;            /* 最长延迟 */
volatile uint32_t probe_min_us    = 0xFFFFFFFFu;  /* 最短延迟（无样本时是 0xFFFFFFFF）*/
volatile uint32_t probe_p50       = 0;            /* 分位数（每采到新样本后重算）*/
volatile uint32_t probe_p95       = 0;
volatile uint32_t probe_p99       = 0;
volatile uint32_t probe_timeout   = 0;            /* 武装超时次数，应该 0 */
volatile uint32_t probe_count_skip = 0;           /* 被跳过的"松开"样本数 */

/* 在线清零开关：Watch 里改成 1，下一拍就清空所有统计。
   初值写成 1 —— 上电后第一拍自动清一次，每次烧写完都是干净的。
   做参数对比实验时不用重烧固件，两轮可以接着测，条件完全一致。 */
volatile uint8_t  probe_clear     = 1;

static uint32_t probe_send_done    = 0;   /* 已完成的 HID 报告发送次数（帧序号）*/
static uint32_t probe_pending_done = 0;   /* 报告变化时"当时已完成"的发送数 */
static uint32_t probe_t0           = 0;   /* 【起点】原始电平第一次变化的时刻 */
static uint32_t probe_lockout      = 0;   /* 上次测量完成的时刻（封锁期用）*/
static uint8_t  probe_armed        = 0;   /* 是否已武装（有一次测量在进行）*/
static uint8_t  probe_changed      = 0;   /* 本次测量中报告内容是否已变化 */
static uint8_t  probe_raw_prev     = 0;   /* 上一拍的 8 个按键原始电平 */
static uint8_t  probe_is_press     = 2u;  /* 本次变化：1=按下 0=松开 2=其他(摇杆等)*/

/* ============================ 无线探针（第 8 步）============================
   有线模式的终点是 USB 的 DataIn 回调；无线没有这个回调，所以拆成两段：

     手柄侧：输入原始电平变化 → nrf24_send() 返回 TX_OK
             TX_OK = 收到 dongle 硬件回的 ACK，所以【空中时间已经包含在这一段里】，
             不需要再单独加"空中 ≈350µs"。
     dongle：读到 RX_DR（收到包） → USB DataIn 完成

   两段相加 = 端到端，中间只差一个 ACK 回传（~130µs）。

   ⚠️ 为什么另起一套变量、不和有线探针共用：
      有线那套已经验收过（P50 0.52ms），不该为了新功能去动它；
      两套分开，出问题时能一眼分清是哪一套坏了。
   ========================================================================== */

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD

/* ---- 手柄侧统计量（Watch 里看这些）---- */
volatile uint16_t wprobe_hist[PROBE_BUCKETS];
volatile uint16_t wprobe_hist_over = 0;            /* 超量程（>5000µs），应该 0 */
volatile uint32_t wprobe_count     = 0;            /* 有效样本数（只算按下）*/
volatile uint32_t wprobe_max_us    = 0;
volatile uint32_t wprobe_min_us    = 0xFFFFFFFFu;
volatile uint32_t wprobe_p50       = 0;
volatile uint32_t wprobe_p95       = 0;
volatile uint32_t wprobe_p99       = 0;
volatile uint32_t wprobe_timeout   = 0;            /* 武装超时次数，应该 0 */
volatile uint32_t wprobe_maxrt_cnt = 0;            /* 武装后遇到 MAX_RT 的帧数（链路差的证据）*/

/* ---- 对账用的三个计数器（2026-10-26 加）----
   背景：实测"按 112 次只统计到 100 次"，需要知道少掉的那 12 次去哪了。
   对账公式：wprobe_press_seen ≈ wprobe_count + wprobe_skip_press + wprobe_locked + wprobe_timeout */
volatile uint32_t wprobe_press_seen = 0;   /* 固件看到"按下方向"的次数（应 ≈ 你实际按的次数）*/
volatile uint32_t wprobe_locked     = 0;   /* 想武装、但被 20ms 封锁期挡掉的按下次数 */
volatile uint32_t wprobe_skip_press = 0;   /* 武装成功、但最终判成"非按下"而丢掉的次数 */

/* ⚠️ 下面 4 个是【跨任务】的：1ms 任务里写（WirelessProbe_OnTick），
   无线任务里读（LatencyProbe_OnWirelessTx）—— 必须 volatile，
   否则编译器可能把它缓存进寄存器，两个任务看到的值不一致。
   这是无线版和有线条最大的结构差别（有线版是"任务写、USB 中断读"）。 */
static volatile uint32_t wprobe_t0      = 0;   /* 起点：原始电平第一次变化 */
static volatile uint8_t  wprobe_armed   = 0;
static volatile uint8_t  wprobe_changed = 0;   /* 报告内容已反映这次变化 */
static volatile uint32_t wprobe_lockout = 0;   /* 上次完成的时刻（封锁期）*/

/* 下面这几个只在 1ms 任务里用，不需要 volatile */
static uint8_t  wprobe_is_press  = 2u;
static uint8_t  wprobe_raw_prev  = 0;
static uint8_t  wprobe_prev[9];         /* 上一拍的报告内容（判按下/松开用）*/

#else  /* ---------------- DONGLE ---------------- */

/* ---- dongle 侧统计量 ---- */
volatile uint16_t wprobe_hist[PROBE_BUCKETS];
volatile uint16_t wprobe_hist_over = 0;
volatile uint32_t wprobe_count     = 0;
volatile uint32_t wprobe_max_us    = 0;
volatile uint32_t wprobe_min_us    = 0xFFFFFFFFu;
volatile uint32_t wprobe_p50       = 0;
volatile uint32_t wprobe_p95       = 0;
volatile uint32_t wprobe_p99       = 0;
volatile uint32_t wprobe_timeout   = 0;
volatile uint32_t wprobe_maxrt_cnt = 0;   /* dongle 侧不用，占位保持变量名一致 */

/* ⚠️ 这两个也是跨上下文的：1ms 任务写（OnRxPacket）、USB 中断读（DataInHook），
   所以同样要 volatile。 */
static volatile uint32_t wprobe_t0      = 0;   /* 起点：发现 RX_DR 的时刻 */
static volatile uint8_t  wprobe_pending = 0;   /* 1 = 有一次测量在等 USB 完成 */

/* ---- 第 8 步：严格配对的缓存 ----
   帧里带的是【上一帧】的 t1（帧在发送前就组好了，而 t1 要等发完才知道），
   所以收到帧 N 时拿到的是 t1(N-1)，要用【上一帧缓存的 t2(N-1)】去配对，
   得到 t1(N-1) + t2(N-1) = 同一个按键事件的完整端到端。 */
static volatile uint16_t wprobe_seq        = 0;   /* 当前帧的 seq */
static volatile uint16_t wprobe_pend_t2    = 0;   /* 上一帧的 t2（µs）*/
static volatile uint16_t wprobe_pend_seq   = 0;   /* 上一帧的 seq */
static volatile uint8_t  wprobe_pend_valid = 0;   /* 上一帧的 t2 是否有效 */
static volatile uint32_t wprobe_t0_raw     = 0;   /* 读 FIFO 之前记下的时刻（先存着，
                                                     确认是报告帧才启用 —— 心跳帧不发 USB，
                                                     如果直接开测量，pending 会一直挂着）*/

#endif

/* ============================ 内部工具 ============================ */

/* 数一个字节里有几个 1 —— 用来判断按钮位变多了（按下）还是变少了（松开）。 */
static uint8_t popcount8(uint8_t v)
{
    uint8_t n = 0u;
    while (v) { n += (v & 1u); v >>= 1; }
    return n;
}

/* 从直方图重算 P50 / P95 / P99。
   每采到一个新样本后调一次（500 次循环，72MHz 下约 70µs），而且只在输入变化时
   触发，对实时性没有影响。

   两个稳健性处理（都为了不让调试器读到假值）：
   ① 样本少于 20 个就不更新 —— 没有统计意义，而且早期样本少时算出来的分位数
      很容易出现"比 min 还小"这种不可能的值；
   ② 三个值先算到局部变量，最后一次性赋值 —— 避免调试器恰好在
      "清完 0、还没写新值"的瞬间读到中间状态。 */
static void LatencyProbe_UpdateStats(void)
{
    uint32_t total = 0u, cum = 0u, i;
    uint32_t t50, t95, t99;
    uint32_t r50 = 0u, r95 = 0u, r99 = 0u;

    for (i = 0u; i < PROBE_BUCKETS; i++) { total += probe_hist[i]; }
    if (total < 20u) { return; }

    t50 = (total * 50u) / 100u;
    t95 = (total * 95u) / 100u;
    t99 = (total * 99u) / 100u;

    for (i = 0u; i < PROBE_BUCKETS; i++)
    {
        cum += probe_hist[i];
        if ((r50 == 0u) && (cum >= t50)) { r50 = (i + 1u) * PROBE_BUCKET_US; }
        if ((r95 == 0u) && (cum >= t95)) { r95 = (i + 1u) * PROBE_BUCKET_US; }
        if ((r99 == 0u) && (cum >= t99)) { r99 = (i + 1u) * PROBE_BUCKET_US; }
    }

    probe_p50 = r50;
    probe_p95 = r95;
    probe_p99 = r99;
}

/* ---- 无线探针的统计更新：逻辑和有线那套一样，只是换成 wprobe_* ---- */
static void WirelessProbe_UpdateStats(void)
{
    uint32_t total = 0u, cum = 0u, i;
    uint32_t t50, t95, t99;
    uint32_t r50 = 0u, r95 = 0u, r99 = 0u;

    for (i = 0u; i < PROBE_BUCKETS; i++) { total += wprobe_hist[i]; }
    if (total < 20u) { return; }

    t50 = (total * 50u) / 100u;
    t95 = (total * 95u) / 100u;
    t99 = (total * 99u) / 100u;

    for (i = 0u; i < PROBE_BUCKETS; i++)
    {
        cum += wprobe_hist[i];
        if ((r50 == 0u) && (cum >= t50)) { r50 = (i + 1u) * PROBE_BUCKET_US; }
        if ((r95 == 0u) && (cum >= t95)) { r95 = (i + 1u) * PROBE_BUCKET_US; }
        if ((r99 == 0u) && (cum >= t99)) { r99 = (i + 1u) * PROBE_BUCKET_US; }
    }

    wprobe_p50 = r50;
    wprobe_p95 = r95;
    wprobe_p99 = r99;
}

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
/* 无线模式下的每拍处理：武装 + 判按下/松开 + 超时兜底。
   和有线版几乎一样，唯一区别是【终点不在这里】—— 在 LatencyProbe_OnWirelessTx() 里。 */
static void WirelessProbe_OnTick(void)
{
    uint8_t raw = 0u;

    if (HAL_GPIO_ReadPin(A_GPIO_Port,     A_Pin)     == GPIO_PIN_RESET) { raw |= 0x01u; }
    if (HAL_GPIO_ReadPin(B_GPIO_Port,     B_Pin)     == GPIO_PIN_RESET) { raw |= 0x02u; }
    if (HAL_GPIO_ReadPin(X_GPIO_Port,     X_Pin)     == GPIO_PIN_RESET) { raw |= 0x04u; }
    if (HAL_GPIO_ReadPin(Y_GPIO_Port,     Y_Pin)     == GPIO_PIN_RESET) { raw |= 0x08u; }
    if (HAL_GPIO_ReadPin(up_GPIO_Port,    up_Pin)    == GPIO_PIN_RESET) { raw |= 0x10u; }
    if (HAL_GPIO_ReadPin(down_GPIO_Port,  down_Pin)  == GPIO_PIN_RESET) { raw |= 0x20u; }
    if (HAL_GPIO_ReadPin(left_GPIO_Port,  left_Pin)  == GPIO_PIN_RESET) { raw |= 0x40u; }
    if (HAL_GPIO_ReadPin(right_GPIO_Port, right_Pin) == GPIO_PIN_RESET) { raw |= 0x80u; }

    /* ① 原始电平变化 → 武装（起点必须最早：消抖本身也是延迟的一部分）*/
    if (raw != wprobe_raw_prev)
    {
        wprobe_raw_prev = raw;

        if ((wprobe_armed == 0u) && ((DWT->CYCCNT - wprobe_lockout) > PROBE_LOCKOUT_CYCLES))
        {
            wprobe_armed   = 1u;
            wprobe_changed = 0u;
            wprobe_t0      = DWT->CYCCNT;
        }
    }

    /* ② 报告变化 → 判按下/松开 + 对账计数 + 标记"内容已反映这次变化"
       ⚠️ 2026-10-26 修正：对账计数【全部基于消抖后的报告】，不再用原始电平。
          原因：原始电平有机械抖动，一次按下会被数成 2~3 次
          （实测：按 100 次，原始电平的"按下方向跳变"有 148 次）。 */
    if (memcmp(wprobe_prev, gamepad_report, 9) != 0)
    {
        uint8_t old_bits = popcount8(wprobe_prev[0])    + popcount8(wprobe_prev[1]);
        uint8_t new_bits = popcount8(gamepad_report[0]) + popcount8(gamepad_report[1]);

        wprobe_is_press = (new_bits > old_bits) ? 1u : ((new_bits < old_bits) ? 0u : 2u);

        if (new_bits > old_bits)          /* 按下沿 */
        {
            wprobe_press_seen++;          /* 账本：消抖后确认了一次按下（应 = 你按的次数）*/

            if (wprobe_armed == 0u)
            {
                /* 这次按下发生时没有武装 → 被 20ms 封锁期挡住，或者上一次还没测完。
                   两种情况都属于"这次按下没被测量"，所以都记在 locked 里。 */
                wprobe_locked++;
            }
        }

        memcpy(wprobe_prev, gamepad_report, 9);

        if (wprobe_armed) { wprobe_changed = 1u; }
    }

    /* ③ 超时兜底（防止武装挂着、被很久之后的一次发送误消费 → 测出假的巨大值）*/
    if (wprobe_armed && ((DWT->CYCCNT - wprobe_t0) > PROBE_ARM_TIMEOUT_CYCLES))
    {
        wprobe_armed   = 0u;
        wprobe_changed = 0u;
        wprobe_timeout++;
    }
}
#endif /* DEVICE_ROLE_HANDHELD */

/* ============================ 对外接口 ============================ */

/* 初始化：打点脚（PB10 推挽输出，空闲高）+ 打开 DWT 周期计数器。
   在 main() 的 USER CODE 2 里调一次即可。 */
void LatencyProbe_Init(void)
{
    GPIO_InitTypeDef probe = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    probe.Pin   = LATENCY_PROBE_PIN;
    probe.Mode  = GPIO_MODE_OUTPUT_PP;
    probe.Pull  = GPIO_NOPULL;
    probe.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LATENCY_PROBE_PORT, &probe);
    HAL_GPIO_WritePin(LATENCY_PROBE_PORT, LATENCY_PROBE_PIN, GPIO_PIN_SET);  /* 空闲 = 高 */

    /* DWT->CYCCNT 三个步骤缺一不可：使能跟踪 → 计数器清零 → 使能计数 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 主循环里每 1ms 调一次（scan_buttons / analog_update 之后、send_report 之前）。
   干两件事：
     ① 报告内容变化 → 把打点脚拉低（给逻辑分析仪一个可见的边沿）
     ② 按键【原始电平】变化 → 武装一次测量，记下起点时间

   ② 为什么读原始电平、而不是等报告变化：
   消抖本身就是延迟里的一块。如果从"报告变化"才开始计时，就把消抖整个漏掉了，
   测出来的数没有意义。所以起点必须是"固件第一次看到电平变化"。 */
void LatencyProbe_OnTick(void)
{
    uint8_t raw = 0u;

    /* ---- 在线清零：Watch 里把 probe_clear 改成 1 即可（下一拍生效）----
       只清统计量，不动 probe_send_done / probe_pending_done —— 那两个是
       "帧序号"，清零反而会让完成判据错乱。 */
    if (probe_clear)
    {
        uint16_t k;
        probe_clear = 0u;
        for (k = 0u; k < PROBE_BUCKETS; k++) { probe_hist[k] = 0u; }
        probe_hist_over   = 0u;
        probe_count       = 0u;
        probe_max_us      = 0u;
        probe_min_us      = 0xFFFFFFFFu;
        probe_p50 = 0u; probe_p95 = 0u; probe_p99 = 0u;
        probe_fall        = 0u;
        probe_rise        = 0u;
        probe_datain_any  = 0u;
        probe_datain_ep1  = 0u;
        probe_timeout     = 0u;
        probe_count_skip  = 0u;
        probe_armed       = 0u;
        probe_changed     = 0u;
        probe_lockout     = DWT->CYCCNT;   /* 清完先封锁，避开清空瞬间的残留 */

        /* ---- 第 8 步：无线探针也一起清（两块板共用同一个 probe_clear 开关）---- */
        {
            uint16_t k2;
            for (k2 = 0u; k2 < PROBE_BUCKETS; k2++) { wprobe_hist[k2] = 0u; }
            wprobe_hist_over = 0u;
            wprobe_count     = 0u;
            wprobe_max_us    = 0u;
            wprobe_min_us    = 0xFFFFFFFFu;
            wprobe_p50 = 0u; wprobe_p95 = 0u; wprobe_p99 = 0u;
            wprobe_timeout   = 0u;
            wprobe_maxrt_cnt = 0u;
#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
            wprobe_press_seen = 0u;
            wprobe_locked     = 0u;
            wprobe_skip_press = 0u;
            wprobe_armed     = 0u;
            wprobe_changed   = 0u;
            wprobe_lockout   = DWT->CYCCNT;
#else
            wprobe_pending   = 0u;
#endif
        }
    }

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    /* ---- 第 8 步：无线模式走【独立那一套】----
       起点同样是"原始电平变化"，但终点是 nrf24_send() 返回 TX_OK，
       不是 USB 的 DataIn 回调（无线模式下这个回调根本不会来）。
       所以这里做完武装/判按下就返回，不往下走有线的逻辑。 */
    if (app_is_wired() == 0u)
    {
        WirelessProbe_OnTick();
        return;
    }
#endif

    /* ---- 读 8 个按键引脚的【原始电平】（未经消抖）---- */
    if (HAL_GPIO_ReadPin(A_GPIO_Port,     A_Pin)     == GPIO_PIN_RESET) { raw |= 0x01u; }
    if (HAL_GPIO_ReadPin(B_GPIO_Port,     B_Pin)     == GPIO_PIN_RESET) { raw |= 0x02u; }
    if (HAL_GPIO_ReadPin(X_GPIO_Port,     X_Pin)     == GPIO_PIN_RESET) { raw |= 0x04u; }
    if (HAL_GPIO_ReadPin(Y_GPIO_Port,     Y_Pin)     == GPIO_PIN_RESET) { raw |= 0x08u; }
    if (HAL_GPIO_ReadPin(up_GPIO_Port,    up_Pin)    == GPIO_PIN_RESET) { raw |= 0x10u; }
    if (HAL_GPIO_ReadPin(down_GPIO_Port,  down_Pin)  == GPIO_PIN_RESET) { raw |= 0x20u; }
    if (HAL_GPIO_ReadPin(left_GPIO_Port,  left_Pin)  == GPIO_PIN_RESET) { raw |= 0x40u; }
    if (HAL_GPIO_ReadPin(right_GPIO_Port, right_Pin) == GPIO_PIN_RESET) { raw |= 0x80u; }

    /* ---- ① 原始电平变了 → 【先】武装一次测量 ----
       ⚠️ 这一步必须排在"报告变化"【前面】，顺序不能换！

       原因：scan_buttons() 在主循环里已经先跑完了，所以当 db_press_ticks = 1 时，
       "报告变化"和"原始电平变化"会落在【同一拍】。如果先处理报告变化，那一刻
       probe_armed 还是 0，probe_changed 就永远置不上，这次武装会一直挂到下一次
       报告变化（也就是松开的时候），测出来的就成了"按住时长"（几百毫秒）。
       这是实测踩过的坑：db_press_ticks 从 2 改成 1 之后，max 直接跳到 139.7ms。 */
    if (raw != probe_raw_prev)
    {
        probe_raw_prev = raw;

        if ((probe_armed == 0u) && ((DWT->CYCCNT - probe_lockout) > PROBE_LOCKOUT_CYCLES))
        {
            probe_armed   = 1u;
            probe_changed = 0u;
            probe_t0      = DWT->CYCCNT;   /* ★起点：固件第一次看到电平变化的那一拍 */
        }
    }

    /* ---- ② 报告内容变了 → 拉低打点脚 + 记下"终点判据" + 判断按下/松开 ---- */
    if (memcmp(probe_prev, gamepad_report, 9) != 0)
    {
        /* 先判断这次变化是"按下"还是"松开"：比较两个按钮字节里置位的个数。

           ⚠️ 这一步决定了指标的【定义】，比看起来重要：
           按下阈值是 1 拍（≈0~1ms），松开阈值是 4 拍（≈4~5ms）。
           两者混在一起统计的话，一半样本被松开的 4ms 抬高，中位数会死死卡在
           2ms 附近下不来 —— 实测就是这样（P50 一直 1.98ms 降不下去）。
           而我们真正要衡量、用户也能感知的是【按下】，业界测手柄 input lag
           也只测按下。所以松开的样本单独计数、不进直方图。 */
        {
            uint8_t old_bits = popcount8(probe_prev[0])     + popcount8(probe_prev[1]);
            uint8_t new_bits = popcount8(gamepad_report[0]) + popcount8(gamepad_report[1]);
            probe_is_press = (new_bits > old_bits) ? 1u : ((new_bits < old_bits) ? 0u : 2u);
        }

        HAL_GPIO_WritePin(LATENCY_PROBE_PORT, LATENCY_PROBE_PIN, GPIO_PIN_RESET);
        probe_fall++;                                            /* 调试计数 */
        memcpy(probe_prev, gamepad_report, 9);

        /* 正在测量的话，记下"这一刻已完成多少帧"——变化之后发出去的第一帧就是终点 */
        if (probe_armed)
        {
            probe_pending_done = probe_send_done;
            probe_changed      = 1u;
        }
    }

    /* ---- ③ 武装超时兜底：挂着超过 50ms 就放弃，避免产生假的巨大值 ---- */
    if (probe_armed && ((DWT->CYCCNT - probe_t0) > PROBE_ARM_TIMEOUT_CYCLES))
    {
        probe_armed   = 0u;
        probe_changed = 0u;
        probe_timeout++;     /* 记一笔；正常应该一直是 0 */
    }
}

/* USB DataIn 打点钩子。
   它由 usbd_conf.c 里的宏在【原回调调用 USBD_LL_DataInStage() 的那一刻】调用，
   所以这里不需要自己再调 USBD_LL_DataInStage —— 原始调用照常执行。 */
void LatencyProbe_DataInHook(uint8_t epnum)
{
    probe_datain_any++;                        /* 钩子有没有被调用 */

    /* epnum 是【不含方向位】的端点号：CUSTOM_HID_EPIN_ADDR = 0x81 → epnum = 1 */
    if (epnum == (CUSTOM_HID_EPIN_ADDR & 0x7FU))
    {
        probe_datain_ep1++;                    /* 端点号判断有没有命中 */
        HAL_GPIO_WritePin(LATENCY_PROBE_PORT, LATENCY_PROBE_PIN, GPIO_PIN_SET);
        probe_rise++;                          /* 上升沿 = 已发上总线 */

        probe_send_done++;                     /* 又成功发出去了一帧报告 */

#if DEVICE_ROLE == DEVICE_ROLE_DONGLE
        /* ---- 第 8 步：dongle 侧无线探针的【终点】----
           起点是"发现 RX_DR"（wireless_rx.c 里调 LatencyProbe_OnRxPacket），
           这里是"报告真正发上 USB 总线"，所以这一段的含义是
           【dongle 收到包 → PC 能取到】。 */
        if (wprobe_pending)
        {
            uint32_t us_w = (DWT->CYCCNT - wprobe_t0) / (SystemCoreClock / 1000000u);

            /* 第 8 步：这里【只缓存 t2】，不立即统计。
               因为要等【下一帧】带来的 t1 才能和这一帧配对（见 OnRxFrame），
               单看这一段没有意义 —— 它只是"收包 → USB"的半段。 */
            wprobe_pend_t2    = (uint16_t)((us_w > 0xFFFFu) ? 0xFFFFu : us_w);
            wprobe_pend_seq   = wprobe_seq;
            wprobe_pend_valid = 1u;
            wprobe_pending    = 0u;
        }
#endif

        /* 测量完成要同时满足三个条件：
             armed        —— 有一次测量正在进行
             changed      —— 报告内容已经反映出这次变化（也就是消抖已完成）
             send_done >  —— 而且这一帧正是"变化之后发出去的第一帧"
           这样才能测到完整时间，既不会漏掉消抖，也不会被前一次还在途中的发送干扰。 */
        if (probe_armed && probe_changed && (probe_send_done > probe_pending_done))
        {
            uint32_t cycles = DWT->CYCCNT - probe_t0;   /* 无符号减法：计数器溢出也正确 */
            uint32_t us     = cycles / (SystemCoreClock / 1000000u);

            probe_armed   = 0u;
            probe_changed = 0u;
            probe_lockout = DWT->CYCCNT;                /* 进入封锁期，压掉机械抖动 */

            /* 只统计【按下】的延迟 —— 这才是用户能感知、也是业界定义的
               手柄 input latency。松开的样本单独计数。 */
            if (probe_is_press == 1u)
            {
                probe_count++;

                if (us < (PROBE_BUCKETS * PROBE_BUCKET_US)) { probe_hist[us / PROBE_BUCKET_US]++; }
                else                                        { probe_hist_over++; }

                if (us > probe_max_us) { probe_max_us = us; }
                if (us < probe_min_us) { probe_min_us = us; }

                LatencyProbe_UpdateStats();
            }
            else
            {
                probe_count_skip++;     /* 松开（0）或摇杆等其它变化（2），不计入 */
            }
        }
    }
}

/* ==================== 第 8 步：无线探针的两个端点 ==================== */

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
/* 手柄侧【终点】：一次无线发送完成（在 wireless_tx.c 里 nrf24_send 返回后调）。
   ret 为 NRF24_TX_OK 时说明收到了 dongle 的硬件 ACK —— 这一刻 dongle 已经收到包了，
   所以"空中时间"已经包含在这一段里，不需要另外再加。 */
void LatencyProbe_OnWirelessTx(uint8_t ret)
{
    if ((wprobe_armed == 0u) || (wprobe_changed == 0u)) { return; }

    if (ret == NRF24_TX_OK)
    {
        uint32_t cycles = DWT->CYCCNT - wprobe_t0;   /* 无符号减法：计数器溢出也正确 */
        uint32_t us     = cycles / (SystemCoreClock / 1000000u);

        wprobe_armed   = 0u;
        wprobe_changed = 0u;
        wprobe_lockout = DWT->CYCCNT;

        /* 只统计【按下】—— 和有线同一个理由：松手晚几毫秒没人感知得到 */
        if (wprobe_is_press == 1u)
        {
            /* 第 8 步：把这个 t1 存下来，下一帧组帧时带给 dongle（严格配对用）。
               帧必须先组好才能发，所以 t1 只能"晚一帧"带出去。
               超过 65535µs 就截顶（正常只有一两毫秒，不会发生）。 */
            wprobe_t1_out = (uint16_t)((us > 0xFFFFu) ? 0xFFFFu : us);

            wprobe_count++;

            if (us < (PROBE_BUCKETS * PROBE_BUCKET_US)) { wprobe_hist[us / PROBE_BUCKET_US]++; }
            else                                        { wprobe_hist_over++; }

            if (us > wprobe_max_us) { wprobe_max_us = us; }
            if (us < wprobe_min_us) { wprobe_min_us = us; }

            WirelessProbe_UpdateStats();
        }
        else
        {
            /* 松开或摇杆变化：不计入直方图，但记一笔 —— 这样对账才闭合
               （wprobe_press_seen ≈ wprobe_count + wprobe_skip_press + wprobe_locked）*/
            wprobe_skip_press++;
        }
    }
    else if (ret == NRF24_TX_MAXRT)
    {
        /* 这一帧没送到：【不结束测量】，等下一帧继续等。
           这样"链路差导致的重发"会真实地反映成更大的延迟 —— 这是对的，
           不能为了数字好看把它剔掉。 */
        wprobe_maxrt_cnt++;
    }
}
#else
void LatencyProbe_OnWirelessTx(uint8_t ret) { (void)ret; }   /* dongle 侧用不到 */
#endif

/* dongle 侧【起点】：刚发现 RX_DR（在 wireless_rx.c 里读到 RX_DR 后立刻调）。
   注意这是"1ms 轮询发现"的时刻，不是"空中到达"的时刻，
   所以含 0~1ms 的轮询量化误差 —— 和手柄侧用原始电平当起点是同一个道理。

   ⚠️ 这里【只打点】。配对要等解出 seq 才能做，见下面的 OnRxFrame()。
      为什么不在这一步就把 t0 和配对一起做完：
      读 FIFO + 解帧要几百 µs，如果把打点放到解帧之后，
      这段读包时间会被算进"收包 → USB"，把第二段算大。 */
void LatencyProbe_OnRxPacket(void)
{
#if DEVICE_ROLE == DEVICE_ROLE_DONGLE
    /* 只记时刻，【不开】测量 —— 因为这时候还不知道这包是报告帧还是心跳帧。
       心跳帧不发 USB，如果这里直接 pending = 1，就永远等不到 DataIn，
       下一次报告帧的 DataIn 会用错的起点。 */
    wprobe_t0_raw = DWT->CYCCNT;
#endif
}

/* dongle 侧【配对】：解出 seq 之后调（第 8 步的严格方案）。
   帧里带的是【上一帧】的 t1（帧在发送前组好，t1 要等发完才知道），
   所以收到 seq 这一帧时拿到的是 t1(seq-1)，要用缓存的 t2(seq-1) 去配对，
   得到 t1(seq-1) + t2(seq-1) = 同一个按键事件的完整端到端。

   两个前提，缺一就不配对：
     - 上一帧的 t2 有效（pend_valid）
     - seq 连续（不连续说明中间丢了帧，缓存对不上）
     - t1_prev != 0（手柄侧这一帧没带 t1，比如心跳帧或没武装） */
void LatencyProbe_OnRxFrame(uint16_t seq, uint16_t t1_prev_us)
{
#if DEVICE_ROLE == DEVICE_ROLE_DONGLE
    if (wprobe_pend_valid &&
        (seq == (uint16_t)(wprobe_pend_seq + 1u)) &&
        (t1_prev_us != 0u))
    {
        uint32_t total = (uint32_t)t1_prev_us + (uint32_t)wprobe_pend_t2;

        wprobe_count++;

        if (total < (PROBE_BUCKETS * PROBE_BUCKET_US)) { wprobe_hist[total / PROBE_BUCKET_US]++; }
        else                                          { wprobe_hist_over++; }

        if (total > wprobe_max_us) { wprobe_max_us = total; }
        if (total < wprobe_min_us) { wprobe_min_us = total; }

        WirelessProbe_UpdateStats();
    }

    wprobe_pend_valid = 0u;   /* 配对只用一次，避免同一份 t2 被下一个 t1 重复消费 */
    wprobe_seq        = seq;  /* DataIn 里要把 t2 挂到这一帧的 seq 上 */

    /* 开一次新的：t0 用"读 FIFO 之前"记下的那个时刻，
       这样读包那几百 µs 不会被算进第二段（dongle 段 = 发现 RX_DR → USB 完成）。 */
    wprobe_t0      = wprobe_t0_raw;
    wprobe_pending = 1u;
#else
    (void)seq;
    (void)t1_prev_us;
#endif
}

#endif /* LATENCY_PROBE_ENABLE */
