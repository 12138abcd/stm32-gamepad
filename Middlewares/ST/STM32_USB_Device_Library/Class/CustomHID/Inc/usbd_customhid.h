/**
  ******************************************************************************
  * @file    usbd_customhid.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_customhid.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CUSTOMHID_H
#define __USB_CUSTOMHID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CUSTOM_HID
  * @brief This file is the Header file for USBD_customhid.c
  * @{
  */


/** @defgroup USBD_CUSTOM_HID_Exported_Defines
  * @{
  */
/* ⚠️ 本工程的 HID 报告是 9 字节，wMaxPacketSize 必须跟报告长度对齐，
   否则一个报告会被拆成多个 USB 事务（9 字节 = 2+2+2+2+1，共 5 个事务）。
   ST 原来的默认值 0x02U 对应它自带的 2 字节示例报告，对本工程是错的。
   这里加上 #ifndef 保护 —— 原版没有保护，导致在 usbd_conf.h 里定义或命令行
   -D 覆盖都无效，只能改这个文件。加保护之后就能从 usbd_conf.h 管了。 */
#ifndef CUSTOM_HID_EPIN_ADDR
#define CUSTOM_HID_EPIN_ADDR                 0x81U
#endif
#ifndef CUSTOM_HID_EPIN_SIZE
#define CUSTOM_HID_EPIN_SIZE                 0x09U
#endif

#ifndef CUSTOM_HID_EPOUT_ADDR
#define CUSTOM_HID_EPOUT_ADDR                0x01U
#endif
#ifndef CUSTOM_HID_EPOUT_SIZE
#define CUSTOM_HID_EPOUT_SIZE                0x02U   /* OUT 用于主机下发（以后做振动），暂保持 2 */
#endif

#define USB_CUSTOM_HID_CONFIG_DESC_SIZ       41U
#define USB_CUSTOM_HID_DESC_SIZ              9U

#ifndef CUSTOM_HID_HS_BINTERVAL
#define CUSTOM_HID_HS_BINTERVAL            0x05U
#endif /* CUSTOM_HID_HS_BINTERVAL */

/* ⚠️ 默认改成 0x01U（1ms 轮询）。ST 原值 0x05U 意味着主机每 5ms 才来取一次报告，
   报告更新率上限只有 200Hz、延迟被量化到 5ms —— 直接卡死本项目 P50 ≤ 5ms 的目标。
   同样加 #ifndef 保护，usbd_conf.h 里也显式定义一份（双保险，防止被刷回）。 */
#ifndef CUSTOM_HID_FS_BINTERVAL
#define CUSTOM_HID_FS_BINTERVAL            0x01U
#endif /* CUSTOM_HID_FS_BINTERVAL */

#ifndef USBD_CUSTOMHID_OUTREPORT_BUF_SIZE
#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE  0x02U
#endif /* USBD_CUSTOMHID_OUTREPORT_BUF_SIZE */
#ifndef USBD_CUSTOM_HID_REPORT_DESC_SIZE
#define USBD_CUSTOM_HID_REPORT_DESC_SIZE   163U
#endif /* USBD_CUSTOM_HID_REPORT_DESC_SIZE */

#define CUSTOM_HID_DESCRIPTOR_TYPE           0x21U
#define CUSTOM_HID_REPORT_DESC               0x22U

#define CUSTOM_HID_REQ_SET_PROTOCOL          0x0BU
#define CUSTOM_HID_REQ_GET_PROTOCOL          0x03U

#define CUSTOM_HID_REQ_SET_IDLE              0x0AU
#define CUSTOM_HID_REQ_GET_IDLE              0x02U

#define CUSTOM_HID_REQ_SET_REPORT            0x09U
#define CUSTOM_HID_REQ_GET_REPORT            0x01U
/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  CUSTOM_HID_IDLE = 0U,
  CUSTOM_HID_BUSY,
}
CUSTOM_HID_StateTypeDef;

typedef struct _USBD_CUSTOM_HID_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* OutEvent)(uint8_t event_idx, uint8_t state);

} USBD_CUSTOM_HID_ItfTypeDef;

typedef struct
{
  uint8_t              Report_buf[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  uint32_t             IsReportAvailable;
  CUSTOM_HID_StateTypeDef     state;
}
USBD_CUSTOM_HID_HandleTypeDef;
/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_CUSTOM_HID;
#define USBD_CUSTOM_HID_CLASS    &USBD_CUSTOM_HID
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev,
                                   uint8_t *report,
                                   uint16_t len);



uint8_t  USBD_CUSTOM_HID_RegisterInterface(USBD_HandleTypeDef   *pdev,
                                           USBD_CUSTOM_HID_ItfTypeDef *fops);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CUSTOMHID_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
