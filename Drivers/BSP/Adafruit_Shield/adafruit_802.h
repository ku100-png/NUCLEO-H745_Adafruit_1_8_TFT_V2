/**
  ******************************************************************************
  * @file    adafruit_802.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for:
  *          - Joystick available on Adafruit 1.8" TFT LCD shield (reference ID 802)
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ADAFRUIT_802_H
#define ADAFRUIT_802_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "adafruit_802_conf.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup ADAFRUIT_802 ADAFRUIT_802
  * @{
  */

/** @defgroup ADAFRUIT_802_LOW_LEVEL STM32 ADAFRUIT_802 LOW LEVEL
  * @brief This file provides set of firmware functions to manage joystick
  *        available on Adafruit shield.
  * @{
  */


#ifdef USE_ADAFRUIT_SHIELD_V2
#include "../Components/adafruit_seesaw/adafruit_seesaw.h"


#define TFTSHIELD_ADDR 0x2E
#define TFTSHIELD_PA02_PIN	2
#define TFTSHIELD_RESET_PIN 3
#define TFTSHIELD_PA02_RESET	(TFTSHIELD_PA02_PIN	|	TFTSHIELD_RESET_PIN)

#define TFTSHIELD_BACKLIGHT_ON 0xFFFF
#define TFTSHIELD_BACKLIGHT_OFF 0x0000

/* Note: these joystick directions are defined relative to the
 * shield oriented with display on left of joystick and SD card facing right
 */

/* Note: joystick activations are active low. */

#define TFTSHIELD_BUTTON_UP_PIN 5
#define TFTSHIELD_BUTTON_UP (1UL << TFTSHIELD_BUTTON_UP_PIN)

#define TFTSHIELD_BUTTON_DOWN_PIN 8
#define TFTSHIELD_BUTTON_DOWN (1UL << TFTSHIELD_BUTTON_DOWN_PIN)

#define TFTSHIELD_BUTTON_LEFT_PIN 6
#define TFTSHIELD_BUTTON_LEFT (1UL << TFTSHIELD_BUTTON_LEFT_PIN)

#define TFTSHIELD_BUTTON_RIGHT_PIN 9
#define TFTSHIELD_BUTTON_RIGHT (1UL << TFTSHIELD_BUTTON_RIGHT_PIN)

#define TFTSHIELD_BUTTON_IN_PIN 7
#define TFTSHIELD_BUTTON_IN (1UL << TFTSHIELD_BUTTON_IN_PIN)

#define TFTSHIELD_BUTTON_1_PIN 10
#define TFTSHIELD_BUTTON_1 (1UL << TFTSHIELD_BUTTON_1_PIN)

#define TFTSHIELD_BUTTON_2_PIN 11
#define TFTSHIELD_BUTTON_2 (1UL << TFTSHIELD_BUTTON_2_PIN)

#define TFTSHIELD_BUTTON_3_PIN 14
#define TFTSHIELD_BUTTON_3 (1UL << TFTSHIELD_BUTTON_3_PIN)

#define TFTSHIELD_BUTTON_ALL                                                   \
  (TFTSHIELD_BUTTON_UP | TFTSHIELD_BUTTON_DOWN | TFTSHIELD_BUTTON_LEFT |       \
   TFTSHIELD_BUTTON_RIGHT | TFTSHIELD_BUTTON_IN | TFTSHIELD_BUTTON_1 |         \
   TFTSHIELD_BUTTON_2 | TFTSHIELD_BUTTON_3)

#endif



/** @defgroup ADAFRUIT_802_LOW_LEVEL_Exported_Types LOW LEVEL Exported Types
  * @{
  */
typedef enum
{
 JOY1 = 0U,
 JOYn
}JOY_TypeDef;

typedef enum
{
  JOY_MODE_GPIO = 0U,
  JOY_MODE_EXTI = 1U,
}JOYMode_TypeDef;

typedef enum 
{ 
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
} JOYState_TypeDef;

typedef enum 
{ 
  BUTTON_NONE = 0,
  BUTTON_A  	= 1,
  BUTTON_B  	= 2,
  BUTTON_C  	= 3,
} BUTTONState_TypeDef;

#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
typedef struct
{
  pADC_CallbackTypeDef  pMspInitCb;
  pADC_CallbackTypeDef  pMspDeInitCb;
}ADAFRUIT_802_JOY_Cb_t;
#endif /* (USE_HAL_ADC_REGISTER_CALLBACKS == 1) */

/**
  * @}
  */

/** @defgroup ADAFRUIT_802_LOW_LEVEL_Exported_Functions LOW LEVEL Exported_Functions
  * @{
  */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
int32_t ADAFRUIT_802_JOY_RegisterDefaultMspCallbacks(JOY_TypeDef JOY);
int32_t ADAFRUIT_802_JOY_RegisterMspCallbacks(JOY_TypeDef JOY, ADAFRUIT_802_JOY_Cb_t *Callback);
#endif /* (USE_HAL_ADC_REGISTER_CALLBACKS == 1) */
HAL_StatusTypeDef MX_ADAFRUIT_802_ADC_Init(ADC_HandleTypeDef *hadc);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* ADAFRUIT_802_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
