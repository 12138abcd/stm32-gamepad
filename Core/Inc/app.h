/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层总入口（手柄 / dongle 双角色）
  *
  * 这是整个固件的"抓总"文件：
  *   - 只负责"按角色选模块"和"1ms 节拍里调什么"
  *   - 具体功能都在各模块里（button_front / analog_input / report / wireless_*）
  *
  * ============================ 角色 ============================
  *   DEVICE_ROLE_HANDHELD：手柄 —— 本地按键/摇杆 → 报告 → 无线发（USB 也发）
  *   DEVICE_ROLE_DONGLE  ：dongle —— 无线收 → 报告 → USB 发
  *
  * ⚠️ 两块板烧不同的 DEVICE_ROLE！
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#include <stdint.h>

/* ============================ 角色 ============================ */
#define DEVICE_ROLE_HANDHELD  0u   /* 手柄：本地输入 + 无线发 */
#define DEVICE_ROLE_DONGLE    1u   /* dongle：无线收 + USB 发 */

/* ⚠️⚠️ 角色开关就在这一行 —— 两块板必须烧不同的值！ ⚠️⚠️
   0 = 手柄（本地按键/摇杆 → 无线发）
   1 = dongle（无线收 → USB 发）
   ⚠️ 必须定义在 app.h（而不是 main.c）—— 因为 app.c 只看得到这个头文件。 */
#define DEVICE_ROLE  DEVICE_ROLE_HANDHELD   /* ← 只改这个宏的值（0 = 手柄 / 1 = dongle），别烧反 */

/* ============================ 通道（第 7 步）============================
   手柄侧才真正"切"通道；dongle 侧固定走 USB（它就是唯一的输出口）。

   APP_CH_WIRED    USB 已配置 → 走 USB 上报；无线每 20ms 发一帧【心跳】保活，
                   让 dongle 的 100ms 超时不触发 —— 拔线瞬间无线已经在发，切换没有空窗。
   APP_CH_WIRELESS USB 未配置 → 无线满速上报（1ms 生产、约 2ms 一帧）。
   APP_CH_LOST     两条都不通 → 仍按满速尝试，因为不发就永远连不回来。 */
#define APP_CH_WIRED     0u
#define APP_CH_WIRELESS  1u
#define APP_CH_LOST      2u

/** 当前通道（APP_CH_*），1ms 刷新一次 */
uint8_t app_channel_mode(void);

/** 1 = 当前走有线（app_tick 据此决定发不发 USB；wireless_tx 据此决定心跳节奏） */
uint8_t app_is_wired(void);

/* 通道计数器（Keil Watch 观察；都是运行期累计量，不受 debugger halt 影响） */
extern volatile uint8_t  app_ch_mode;       /* 当前模式 */
extern volatile uint32_t app_ch_switches;   /* 模式变化次数 */
extern volatile uint32_t app_usb_lost;      /* 检测到 USB 掉线次数 */
extern volatile uint32_t app_usb_back;      /* USB 恢复次数 */
extern volatile uint8_t  app_dev_state;     /* 最近一次读到的 hUsbDeviceFS.dev_state
                                               （3 = CONFIGURED；拔线后应是 4 = SUSPENDED 或 1 = DEFAULT） */

/* 时基对拍计数器（2026-10-23 加，验 Q6 的"tick 是否偏慢 4.86%"）
   判读：app_tick 每 1ms 跑一次，所以 tick_probe_app 应 ≈ tick_probe_hal。
         两者差多少，就是 RTOS tick 与 HAL tick 的相对偏差。 */
extern volatile uint32_t tick_probe_app;    /* app_tick 执行次数（RTOS tick） */
extern volatile uint32_t tick_probe_hal;    /* 同一时刻的 HAL_GetTick()（TIM4 时基） */

/* ============================ API ============================ */

/** 上电调一次：按角色初始化模块（手柄建无线任务 / dongle 进接收模式） */
void app_init(void);

/** 1ms 任务里调：手柄 = 扫描+映射+判通道+(有线才)上报 + 喂无线；dongle = 收包+USB转发 */
void app_tick(void);

/** 独立任务的入口（手柄侧：无线发送任务） */
void app_task(void *argument);

#endif /* __APP_H */
