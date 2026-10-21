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
#define DEVICE_ROLE  DEVICE_ROLE_HANDHELD   /* ← 当前烧 dongle；手柄板改成 DEVICE_ROLE_HANDHELD */

/* ============================ API ============================ */

/** 上电调一次：按角色初始化模块（手柄建无线任务 / dongle 进接收模式） */
void app_init(void);

/** 1ms 任务里调：手柄 = 扫描+映射+打点+USB上报+塞队列；dongle = 收包+USB转发 */
void app_tick(void);

/** 独立任务的入口（手柄侧：无线发送任务） */
void app_task(void *argument);

#endif /* __APP_H */
