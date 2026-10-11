/**
  ******************************************************************************
  * @file    analog_input.h
  * @brief   模拟输入映射：摇杆（双极性，中心 128）+ 扳机（单极性，0~255）。
  ******************************************************************************
  */
#ifndef __ANALOG_INPUT_H
#define __ANALOG_INPUT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 主循环里每 1ms 调一次：读 adc_buf[0..5]，填报告 byte3 ~ byte8 */
void analog_update(void);

#endif /* __ANALOG_INPUT_H */
