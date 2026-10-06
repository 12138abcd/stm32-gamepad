/**
  ******************************************************************************
  * @file    report.c
  * @brief   HID report buffer and transfer helpers.
  ******************************************************************************
  */

#include "report.h"
#include "usb_device.h"         /* hUsbDeviceFS */
#include "usbd_customhid.h"     /* USBD_CUSTOM_HID_SendReport */

/* Idle baseline: no buttons, hat released (8), sticks centred (128),
   triggers released (0).
   报告的填充由各输入模块负责：
     button_front.c → byte0（按键）和 byte2（十字键 Hat）
     joystick.c     → byte3 / byte4（左摇杆 X / Y）                        */
uint8_t gamepad_report[9] = { 0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };

volatile uint32_t tx_dropped = 0;   /* 调试用：端点忙、这一帧被丢掉 */
volatile uint32_t tx_fail    = 0;   /* 调试用：发送失败（设备未配置 / 挂起）—— 这是关键指标 */
volatile uint32_t tx_ok      = 0;   /* 调试用：成功交给 USB 中间件的次数 */

void send_report(void)
{
    /* 把返回值分开统计。三个数一起看就能判断 USB 处于什么状态，
       而且它们记录的是【运行期间的历史】，不受调试器 halt 影响
       （dev_state 只是"当前瞬间"的值，halt 时读到的可能是假象）。 */
    uint8_t r = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, gamepad_report, 9);

    if (r == USBD_BUSY)      { tx_dropped++; }   /* normally ~0 */
    else if (r == USBD_FAIL) { tx_fail++;    }   /* 设备没配置 / 挂起 */
    else                     { tx_ok++;      }   /* USBD_OK */
}
