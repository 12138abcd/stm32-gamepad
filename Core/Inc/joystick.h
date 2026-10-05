/**
  ******************************************************************************
  * @file    joystick.h
  * @brief   模拟摇杆：ADC 采样 + 中心校准 + 行程拉伸 + 死区。
  ******************************************************************************
  */
#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void joystick_update(void);

#endif /* __JOYSTICK_H */
