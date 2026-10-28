/**
 * @file    app.c
 * @brief   应用层总入口：按角色调度 + 通道状态机
 *
 * 手柄 : 1ms 任务做输入/上报，独立任务发无线；dongle: 1ms 任务收包 -> USB。
 *
 * @note 通道判据为什么看 USB 而不是无线、心跳为什么取 20ms、USB 中断优先级那个坑
 *       —— 详见《固件代码导读》11-§11.7。
 */

#include "app.h"
#include "main.h"
#include "cmsis_os.h"
#include "report.h"
#include "param_store.h"    /* Flash 参数（校准常量 / 配对 ID）*/

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
  #include "button_front.h"
  #include "analog_input.h"
  #include "latency_probe.h"
  #include "wireless_tx.h"
  #include "proto.h"          /* proto_link_is_up() */
  #include "usb_device.h"     /* hUsbDeviceFS / dev_state */
#else
  #include "wireless_rx.h"
#endif

/* 时基对拍：app_tick 每 1ms 跑一次，两者应接近；差值就是 SysTick 与 TIM4 的相对偏差。 */
volatile uint32_t tick_probe_app = 0u;
volatile uint32_t tick_probe_hal = 0u;

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD

/* ---- 第 9 步：配对 ID ---- */
static uint8_t device_id = PROTO_DEV_ID_UNPAIRED;

/**
 * @brief 确保手柄有 dev_id（首次上电用芯片唯一 ID 生成并存 Flash）
 * @note  0 保留给"未配对"，所以生成结果如果碰巧是 0 就改成 1。
 */
static void device_id_init(void)
{
    pad_params_t p = *param_get();
    uint8_t id = (uint8_t)(p.dev_id & 0xFFu);

    if (id == 0u)
    {
        /* STM32F103 的 96 位唯一 ID（0x1FFFF7E8 起 12 字节），取低字节 */
        uint32_t uid = *(volatile const uint32_t *)0x1FFFF7E8u;
        id = (uint8_t)(uid & 0xFFu);
        if (id == 0u) { id = 1u; }

        p.dev_id = (uint16_t)id;
        (void)param_set(&p);   /* 存 Flash，之后每次上电都用同一个 */
    }

    device_id = id;
}

#define CH_DEBOUNCE_TICKS   5u    /* 连续 5 拍一致才认，防 USB 枚举过程中的状态抖动 */

volatile uint8_t  app_ch_mode     = APP_CH_LOST;
volatile uint32_t app_ch_switches = 0u;
volatile uint32_t app_usb_lost    = 0u;
volatile uint32_t app_usb_back    = 0u;
volatile uint8_t  app_dev_state   = 0u;   /* 诊断：USB 状态机的当前值 */

static uint8_t usb_stable  = 0u;  /* 去抖后：1 = 已配置 */
static uint8_t usb_pending = 0u;  /* 最近一次采到的原始值 */
static uint8_t usb_cnt     = 0u;  /* 与 pending 连续一致的拍数 */

static osThreadId_t appTaskHandle = NULL;

/**
 * @brief  判通道（1ms 调一次，必须在 send_report 之前）
 * @note   用 dev_state 而不是 send_report() 的返回值：拔线瞬间 USB 中断就改了状态，
 *         比"发一次发现 FAIL"更早，也不与发送节奏耦合。
 */
static void channel_on_tick(void)
{
    uint8_t raw;
    uint8_t m;

    app_dev_state = hUsbDeviceFS.dev_state;
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

#else  /* dongle 不做通道切换：它就是唯一的 USB 输出口 */

uint8_t app_channel_mode(void) { return APP_CH_WIRED; }
uint8_t app_is_wired(void)     { return 1u;           }

#endif /* DEVICE_ROLE_HANDHELD */

/* 第 9 步：设备 ID。两个角色语义不同：
     手柄  = 自己的 ID（1~255）
     dongle = 它配对的对方（0 = 学习模式，接受并记住第一个）*/
uint8_t app_dev_id(void)
{
#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    return device_id;
#else
    return (uint8_t)(param_get()->dev_id & 0xFFu);
#endif
}

void app_init(void)
{
    (void)param_store_init();   /* 加载 Flash 参数（校准常量 / 配对 ID）*/

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    static const osThreadAttr_t attr = {
        .name       = "appTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t) osPriorityBelowNormal,
    };

    device_id_init();     /* 第 9 步：确保有 dev_id（首次上电生成并存 Flash）*/
    wireless_tx_init();   /* nrf24_init + 建队列 */
    appTaskHandle = osThreadNew(app_task, NULL, &attr);
#else
    wireless_rx_init();   /* nrf24_init + 进接收模式 */
#endif
}

void app_tick(void)
{
#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    scan_buttons();       /* 读 GPIO + 消抖 */
    analog_update();      /* 读 ADC + 映射 */
    channel_on_tick();    /* 判通道 */

    /* 有线 / 无线两套探针，内部按 app_is_wired() 自选（第 8 步） */
    LatencyProbe_OnTick();

    if (app_is_wired())
    {
        send_report();    /* 无线模式下没枚举，调了只会一直累加 tx_fail */
    }

    wireless_tx_on_tick();   /* 有线 → 20ms 心跳；无线 → 每拍报告帧 */
#else
    wireless_rx_poll();      /* 收包 + 断线发空闲报告 */
#endif

    tick_probe_app++;
    tick_probe_hal = HAL_GetTick();
}

void app_task(void *argument)
{
#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
    wireless_tx_task(argument);
#else
    (void)argument;
    for (;;) { osDelay(1000u); }   /* dongle 不需要独立任务 */
#endif
}
