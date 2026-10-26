/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层总入口实现（手柄 / dongle 双角色）+ 通道状态机（第 7 步）
  *
  * 分工：
  *   手柄（DEVICE_ROLE_HANDHELD）
  *     1ms 任务：scan_buttons → analog_update → channel_on_tick
  *               → 有线时才 LatencyProbe_OnTick + send_report
  *               → wireless_tx_on_tick（按通道：报告帧 or 20ms 心跳帧）
  *     独立任务：wireless_tx_task（组帧 → nrf24_send → 统计）
  *
  *   dongle（DEVICE_ROLE_DONGLE）
  *     1ms 任务：wireless_rx_poll（收无线 → 写 gamepad_report → USB 发；
  *                                断线时发空闲报告，防"卡键"）
  *     独立任务：不需要（收包在 1ms 里完成，非阻塞）
  *
  * ============================ 第 7 步：通道切换 ============================
  * 判据有两条，各管一头，不能互相替代：
  *   ① USB 通不通 = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED
  *   ② 无线通不通 = proto_link_is_up()
  *
  * ⚠️ 为什么不能只用 proto_link_is_up() 当切换判据：
  *    它回答的是"无线链路健康吗"，回答不了"我该走哪条"。
  *    有线模式下无线是【降频心跳】，链路本来就该是通的 ——
  *    若拿它当判据，插着线时也会永远判成"走无线"。
  *
  * ⚠️ 为什么有线时不停发无线，而是发 20ms 心跳：
  *    停发 → dongle 100ms 超时 → 拔线后还要等它重新 link，
  *    空窗 100~200ms，游戏里能感觉到。心跳让 dongle 始终"热备"，
  *    拔线瞬间无线已经在发，切换没有空窗。
  *
  * ⚠️ 第 6 步的"双发"在这里被收掉：
  *    那时手柄和 dongle 都在往 PC 发，PC 上会看到两个 HID 设备同时在动。
  *    现在：有线模式只发 USB；无线只发心跳，dongle 收到心跳【不】写报告、不碰 USB。
  ******************************************************************************
  */

#include "app.h"
#include "main.h"
#include "cmsis_os.h"
#include "report.h"

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
  #include "button_front.h"
  #include "analog_input.h"
  #include "latency_probe.h"
  #include "wireless_tx.h"
  #include "proto.h"          /* proto_link_is_up()：无线侧判据 */
  #include "usb_device.h"     /* hUsbDeviceFS / usbd_def.h：USB 侧判据 */
#else
  #include "wireless_rx.h"
#endif

/* ==================== 时基对拍（2026-10-23 加）==================== */

/* app_tick 每 1ms 跑一次，所以 tick_probe_app 应 ≈ tick_probe_hal。
   两者差多少，就是 RTOS tick（SysTick）与 HAL tick（TIM4）的相对偏差。
   用来验 Q6 那个"52.4ms 是不是 tick 偏慢 4.86% 造成的"疑点。 */
volatile uint32_t tick_probe_app = 0u;
volatile uint32_t tick_probe_hal = 0u;

/* ==================== 手柄侧：通道状态机（第 7 步）==================== */

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD

#define CH_DEBOUNCE_TICKS   5u    /* 连续 5 拍（5ms）一致才承认切换，防枚举抖动 */

volatile uint8_t  app_ch_mode     = APP_CH_LOST;
volatile uint32_t app_ch_switches = 0u;
volatile uint32_t app_usb_lost    = 0u;
volatile uint32_t app_usb_back    = 0u;
volatile uint8_t  app_dev_state   = 0u;   /* 直接观察 USB 状态机的当前状态 */

static uint8_t usb_stable  = 0u;  /* 去抖后的结果：1 = USB 已配置 */
static uint8_t usb_pending = 0u;  /* 最近一次采到的原始值 */
static uint8_t usb_cnt     = 0u;  /* 与 pending 连续一致的拍数 */

/* 无线任务的句柄（只有手柄侧用） */
static osThreadId_t appTaskHandle = NULL;

/* 1ms 调用：去抖 + 判模式。必须在 send_report 之前调。
   为什么用 dev_state 而不用 send_report() 的返回值：
   拔线瞬间 USB 中断就把状态改掉了，比"发一次发现 FAIL"更早；
   而且它不依赖发送节奏（两种通道节奏不同，用返回值判定会和节奏耦合）。 */
static void channel_on_tick(void)
{
    uint8_t raw;
    uint8_t m;

    app_dev_state = hUsbDeviceFS.dev_state;   /* 诊断：先存下来，Watch 里直接看它 */
    raw = (app_dev_state == USBD_STATE_CONFIGURED) ? 1u : 0u;

    if (raw != usb_pending)
    {
        usb_pending = raw;
        usb_cnt     = 0u;
    }
    else if (usb_cnt < CH_DEBOUNCE_TICKS)
    {
        usb_cnt++;
    }

    if ((usb_cnt >= CH_DEBOUNCE_TICKS) && (raw != usb_stable))
    {
        usb_stable = raw;
        if (raw) { app_usb_back++; } else { app_usb_lost++; }
    }

    if (usb_stable)              { m = APP_CH_WIRED;    }
    else if (proto_link_is_up()) { m = APP_CH_WIRELESS; }
    else                         { m = APP_CH_LOST;     }

    if (m != app_ch_mode)
    {
        app_ch_mode = m;
        app_ch_switches++;
    }
}

uint8_t app_channel_mode(void) { return app_ch_mode; }
uint8_t app_is_wired(void)     { return usb_stable;    }

#else  /* dongle：不做通道切换，它本身就是唯一的 USB 输出口 */

uint8_t app_channel_mode(void) { return APP_CH_WIRED; }
uint8_t app_is_wired(void)     { return 1u;           }

#endif /* DEVICE_ROLE_HANDHELD */

/* ============================ 对外 API ============================ */

void app_init(void)
{
#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    static const osThreadAttr_t attr = {
        .name       = "appTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t) osPriorityBelowNormal,
    };

    wireless_tx_init();                       /* nrf24_init + 建队列 */
    appTaskHandle = osThreadNew(app_task, NULL, &attr);
#else
    wireless_rx_init();                       /* nrf24_init + 进接收模式 */
#endif
}

void app_tick(void)
{
#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    scan_buttons();          /* 按键：读 GPIO + 消抖 */
    analog_update();         /* 摇杆/扳机：读 ADC + 映射 */
    channel_on_tick();       /* 第 7 步：先判通道 */

    /* 第 8 步起：探针【两种模式都调】——
       有线版靠 USB 的 DataIn 回调收尾，无线版靠 nrf24_send 返回 TX_OK 收尾；
       具体走哪一套，由 LatencyProbe_OnTick() 内部按 app_is_wired() 判断。 */
    LatencyProbe_OnTick();

    if (app_is_wired())
    {
        send_report();           /* USB 上报（无线模式下没枚举，调了只会一直累加 tx_fail） */
    }

    wireless_tx_on_tick();   /* 有线 → 20ms 心跳；无线 → 每拍塞报告帧 */
#else
    wireless_rx_poll();      /* 收包 + 断线发空闲报告 */
#endif

    /* 时基对拍：每拍 +1，并把此刻的 HAL tick 记下来 */
    tick_probe_app++;
    tick_probe_hal = HAL_GetTick();
}

void app_task(void *argument)
{
#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    wireless_tx_task(argument);
#else
    (void)argument;
    for (;;) { osDelay(1000u); }   /* dongle 不需要独立任务，空转保底 */
#endif
}
