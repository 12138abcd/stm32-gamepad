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
   triggers released (0). */
uint8_t gamepad_report[9] = { 0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };

/* adc_buf 定义在 main.c 的 USER CODE 区，是 ADC + DMA 的转换结果缓冲区：
   [0] = PA0 = VRx（X 轴，左右）    [1] = PA1 = VRy（Y 轴，上下） */
extern uint16_t adc_buf[2];

void build_report(void)
{
    /* ADC 是 12 位（0~4095），HID 报告要 8 位（0~255）—— 右移 4 位即可。
       注：电位器有机械公差，回中时读到的不是精确的 2048，所以光标可能偏一点；
       上下方向也可能反。这两件事第 5 步（中心校准 + 方向）专门处理，
       阶段 4 只要"四个方向都能动"就算通过。 */
    gamepad_report[3] = (uint8_t)(adc_buf[0] >> 4);   /* X 轴（左摇杆左右） */
    gamepad_report[4] = (uint8_t)(adc_buf[1] >> 4);   /* Y 轴（左摇杆上下） */
}

volatile uint32_t tx_dropped = 0;   /* 调试用：被丢掉的帧数 */

void send_report(void)
{
    if (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, gamepad_report, 9) == USBD_BUSY)
    {
        tx_dropped++;      /* 正常运行这个数应该接近 0；一直涨说明发得太快 */
    }
}
