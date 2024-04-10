/**
  ******************************************************************************
  * @file    Demonstrations/CM7/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module for Cortex-M7
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_nucleo.h"
#include "adafruit_802.h"
#include "adafruit_802_lcd.h"
#include "adafruit_802_sd.h"
#include "stm32h7xx_nucleo_bus.h"
#include "stm32_lcd.h"
/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "fatfs_storage.h"
#include "../Components/adafruit_seesaw/adafruit_seesaw.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define  MAX_BMP_FILES  25
#define  MAX_BMP_FILE_NAME 11


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define SEESAW_ADDRESS (0x49) ///< Default Seesaw I2C address 0x2E 49
/*=========================================================================*/


void Error_Handler(void);

// ============================================================================

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
