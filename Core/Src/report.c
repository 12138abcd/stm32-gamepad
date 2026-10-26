/**
 * @file    report.c
 * @brief   HID 报告缓冲区 + USB 发送 + 端点参数的编译期断言
 *
 * @note 为什么要有断言：CubeMX GENERATE 会把端点参数刷回 2 字节 / 5ms，
 *       而【功能完全正常】，只有延迟被悄悄量化到 5ms（2026-10-05、10-07 各栽过一次）。
 *       改成编译期错误，才不会被静默放过。详见《固件代码导读》04-§4.3、06-§8.2。
 */

#include "report.h"
#include "usb_device.h"         /* hUsbDeviceFS */
#include "usbd_customhid.h"     /* USBD_CUSTOM_HID_SendReport（链式 include 已带入三个端点宏）*/

/* 端点参数必须正确，否则直接编译失败。
   报错信息用英文：Keil 的 Build Output 按 ANSI 显示，中文会是乱码。 */
#if CUSTOM_HID_EPIN_SIZE != 0x09U
#error "CUSTOM_HID_EPIN_SIZE must be 9 (got 2?). 9-byte report would be split. Run tools/fix-generated-code.mjs"
#endif

#if CUSTOM_HID_FS_BINTERVAL != 0x01U
#error "CUSTOM_HID_FS_BINTERVAL must be 1 ms (got 5?). Host would poll every 5 ms. Run tools/fix-generated-code.mjs"
#endif

#if USBD_CUSTOM_HID_REPORT_DESC_SIZE != 78
#error "USBD_CUSTOM_HID_REPORT_DESC_SIZE must be 78. Report descriptor length wrong. Run tools/fix-generated-code.mjs"
#endif

/* 空闲基线：按键全 0、Hat 中立(8)、四轴居中(0x80)、扳机松开(0)。
   byte0/2 由 button_front.c 写，byte3~8 由 analog_input.c 写。 */
uint8_t gamepad_report[9] = { 0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };

volatile uint32_t tx_dropped = 0;   /* 端点忙，这一帧被丢 */
volatile uint32_t tx_fail    = 0;   /* 设备未配置 / 挂起 —— 关键指标，应恒 0 */
volatile uint32_t tx_ok      = 0;   /* 成功交给 USB 中间件 */

void send_report(void)
{
    /* 三个计数器都是【运行期累计量】，不受调试器 halt 影响（dev_state 只是瞬时值）。 */
    uint8_t r = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, gamepad_report, 9);

    if (r == USBD_BUSY)      { tx_dropped++; }
    else if (r == USBD_FAIL) { tx_fail++;    }
    else                     { tx_ok++;      }
}
