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

void build_report(void)
{
    /* TODO: assemble gamepad_report[] from the scanned inputs */
	/*暂且无用*/
}

void send_report(void)
{
    /* TODO: USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, gamepad_report, 9);
       It is asynchronous - on USBD_BUSY skip this tick instead of retrying. */
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, gamepad_report, 9);
}
