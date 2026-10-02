/**
  ******************************************************************************
  * @file    report.h
  * @brief   HID report buffer and transfer helpers.
  ******************************************************************************
  */
#ifndef __REPORT_H
#define __REPORT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
/* byte 0 */
#define BTN_A       (1u << 0)   /* 0x01 */
#define BTN_B       (1u << 1)   /* 0x02 */
#define BTN_X       (1u << 2)   /* 0x04 */
#define BTN_Y       (1u << 3)   /* 0x08 */
#define BTN_LB      (1u << 4)   /* 0x10 */
#define BTN_RB      (1u << 5)   /* 0x20 */
#define BTN_VIEW    (1u << 6)   /* 0x40 */
#define BTN_MENU    (1u << 7)   /* 0x80 */

/* byte 1 */
#define BTN_L3      (1u << 0)   /* 0x01 */
#define BTN_R3      (1u << 1)   /* 0x02 */
#define BTN_GUIDE   (1u << 2)   /* 0x04 */
#define BTN_SHARE   (1u << 3)   /* 0x08 */

/* 9-byte HID report layout:
   [0] buttons 1-8    [1] buttons 9-16    [2] hat switch
   [3] X   [4] Y   [5] Rx   [6] Ry   [7] Z   [8] Rz          */
extern uint8_t gamepad_report[9];

void build_report(void);
void send_report(void);

#endif /* __REPORT_H */
