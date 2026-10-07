/**
  ******************************************************************************
  * @file    report.c
  * @brief   HID report buffer and transfer helpers.
  ******************************************************************************
  */

#include "report.h"
#include "usb_device.h"         /* hUsbDeviceFS */
#include "usbd_customhid.h"     /* USBD_CUSTOM_HID_SendReport */
/* ↑ 它会链式包含 usbd_ioreq.h → usbd_def.h → usbd_conf.h，
   所以下面三个 USB 端点宏在这里都已经定义好了（不用额外 include）。 */

/* ================= 编译期断言：防止 USB 端点参数被静默刷回 =================
   CubeMX 每次 GENERATE CODE 都会把 `USB_DEVICE/Target/usbd_conf.h` 和中间件里的
   `usbd_customhid.h` 按模板重写，手改的端点参数会被冲回默认值：
       CUSTOM_HID_EPIN_SIZE             9 → 2 字节   （9 字节报告被拆成多个 USB 事务）
       CUSTOM_HID_FS_BINTERVAL          1 → 5 (ms)   （主机每 5ms 才取一次报告）
       USBD_CUSTOM_HID_REPORT_DESC_SIZE 78 → 2       （描述符被截断，设备带感叹号 Code 10）

   ⚠️ 最阴的地方是【功能完全正常】：设备认得出、按键能点、摇杆能动，
      只有端到端延迟被悄悄量化到 5ms —— 2026-10-05 和 2026-10-07 各栽过一次。

   所以在这里把它变成【编译期错误】：一旦被刷回，编译直接失败，不许静默退化。
   位置是故意选的 —— report.c 是本项目自研文件，CubeMX 不会碰它；
   写在生成区文件里的"加固"已经被证明不可靠（会被整体重写掉）。
   报错信息用英文，因为 Keil 的 Build Output 按 ANSI 显示，中文会是乱码。 */
#if CUSTOM_HID_EPIN_SIZE != 0x09U
#error "CUSTOM_HID_EPIN_SIZE must be 9 (got 2?). 9-byte report would be split. Run tools/fix-generated-code.mjs"
#endif

#if CUSTOM_HID_FS_BINTERVAL != 0x01U
#error "CUSTOM_HID_FS_BINTERVAL must be 1 ms (got 5?). Host would poll every 5 ms. Run tools/fix-generated-code.mjs"
#endif

#if USBD_CUSTOM_HID_REPORT_DESC_SIZE != 78
#error "USBD_CUSTOM_HID_REPORT_DESC_SIZE must be 78. Report descriptor length wrong. Run tools/fix-generated-code.mjs"
#endif

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
