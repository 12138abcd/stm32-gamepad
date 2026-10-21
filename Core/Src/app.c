/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层总入口实现（手柄 / dongle 双角色）
  *
  * 分工：
  *   手柄（DEVICE_ROLE_HANDHELD）
  *     1ms 任务：scan_buttons → analog_update → LatencyProbe_OnTick
  *               → send_report（USB）→ wireless_tx_on_tick（塞队列）
  *     独立任务：wireless_tx_task（2ms 一帧发给 dongle）
  *
  *   dongle（DEVICE_ROLE_DONGLE）
  *     1ms 任务：wireless_rx_poll（收无线 → 写 gamepad_report → USB 发）
  *     独立任务：不需要（收包在 1ms 里完成，非阻塞）
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
#else
  #include "wireless_rx.h"
#endif

#if DEVICE_ROLE == DEVICE_ROLE_HANDHELD
/* 无线任务的句柄（只有手柄侧用） */
static osThreadId_t appTaskHandle = NULL;
#endif

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
    LatencyProbe_OnTick();   /* 延迟测量：输入有变化就打点 */
    send_report();           /* USB 上报（有线模式） */
    wireless_tx_on_tick();   /* 1ms：把最新报告塞进发送队列（不阻塞） */
#else
    wireless_rx_poll();      /* 1ms：收无线包 → 写 gamepad_report → USB 发 */
#endif
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
