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

/* 9-byte HID report layout:
   [0] buttons 1-8    [1] buttons 9-16    [2] hat switch
   [3] X   [4] Y   [5] Rx   [6] Ry   [7] Z   [8] Rz          */
extern uint8_t gamepad_report[9];

void build_report(void);
void send_report(void);

#endif /* __REPORT_H */
