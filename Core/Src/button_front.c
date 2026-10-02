/**
  ******************************************************************************
  * @file    button_front.c
  * @brief   Read the 8 buttons (D-pad + A/B/X/Y) and update the HID report.
  ******************************************************************************
  */

#include "button_front.h"
#include "main.h"       /* up_Pin / down_Pin / left_Pin / right_Pin / A_Pin ... */
#include "report.h"     /* gamepad_report[] */

void scan_buttons(void)
{
    /* TODO: read the GPIOs, debounce, write the result into gamepad_report[] */
}
