/**
  ******************************************************************************
  * @file    button_front.h
  * @brief   Face buttons (A/B/X/Y) and D-pad scanning.
  ******************************************************************************
  */
#ifndef __BUTTON_FRONT_H
#define __BUTTON_FRONT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void scan_buttons(void);

#endif /* __BUTTON_FRONT_H */
