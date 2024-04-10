/** 
  ******************************************************************************
  * @file    stm32h7xx_nucleo_bus.h
  * @author  MCD Application Team
  * @brief   This file is the header of stm32h7xx_nucleo_bus.c
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
  *
  ****************************************************************************** 
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32H7XX_NUCLEO_144_BUS_H
#define STM32H7XX_NUCLEO_144_BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_nucleo_conf.h"
#include "stm32h7xx_nucleo_errno.h"
   
/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32H7XX_NUCLEO_144 STM32H7XX_NUCLEO_144
  * @{
  */

/** @addtogroup STM32H7XX_NUCLEO_144_BUS STM32H7XX_NUCLEO_144 BUS
  * @{
  */ 

/**
  * @brief  SPI Interface pins
  *         used to communicate with LCD and SD available on Adafruit 1.8" TFT shield
  */
#define BUS_SPI1_INSTANCE                            SPI1
#define BUS_SPI1_CLK_ENABLE()                        __HAL_RCC_SPI1_CLK_ENABLE()

#define BUS_SPI1_SCK_AF                              GPIO_AF5_SPI1
#define BUS_SPI1_SCK_GPIO_PORT                       GPIOA
#define BUS_SPI1_SCK_PIN                             GPIO_PIN_5
#define BUS_SPI1_SCK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_SCK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOA_CLK_DISABLE()

#define BUS_SPI1_MISO_AF                             GPIO_AF5_SPI1
#define BUS_SPI1_MISO_GPIO_PORT                      GPIOA
#define BUS_SPI1_MISO_PIN                            GPIO_PIN_6   
#define BUS_SPI1_MISO_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MISO_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOA_CLK_DISABLE() 

#define BUS_SPI1_MOSI_AF                             GPIO_AF5_SPI1
#define BUS_SPI1_MOSI_GPIO_PORT                      GPIOB 
#define BUS_SPI1_MOSI_PIN                            GPIO_PIN_5   
#define BUS_SPI1_MOSI_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_SPI1_MOSI_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOB_CLK_DISABLE()

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define BUS_SPI1_TIMEOUT_MAX                        1000U
#ifndef BUS_SPI1_BAUDRATE
 #define BUS_SPI1_BAUDRATE                              12500000 /* baud rate of SPI1 = 12.5 Mbps*/
#endif


#if defined(HAL_I2C_MODULE_ENABLED)
/*##################### I2C1 ###################################*/
/* User can use this section to tailor I2Cx instance used and associated resources */
/* Definition for I2C1 Pins */
#define NUCLEO_I2C1                        I2C1
#define NUCLEO_I2C1_CLK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
#define NUCLEO_I2C1_CLK_DISABLE()          __HAL_RCC_I2C1_CLK_DISABLE()
#define NUCLEO_I2C1_FORCE_RESET()          __HAL_RCC_I2C1_FORCE_RESET()
#define NUCLEO_I2C1_RELEASE_RESET()        __HAL_RCC_I2C1_RELEASE_RESET()  

#define NUCLEO_I2C1_SCL_PIN                GPIO_PIN_8    /* PB.8 */
#define NUCLEO_I2C1_SDA_PIN                GPIO_PIN_9    /* PB.9 */

#define NUCLEO_I2C1_GPIO_PORT              GPIOB      /* GPIOB */
#define NUCLEO_I2C1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_I2C1_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE() 
#define NUCLEO_I2C1_SCL_SDA_AF             GPIO_AF4_I2C1

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define NUCLEO_I2C1_TIMEOUT_MAX            1000

/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 32 MHz */
#define I2C_TIMING_100KHZ_SLEW           0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 
#define I2C_TIMING_100KHZ                0x009080B5 /* 100 KHz speed (Rise time = 50ns, Fall time = 10ns) */
#define I2C_TIMING_400KHZ_SLEW           0x00B1112E /* 400 kHz with analog Filter ON, Rise Time 250ns, Fall Time 100ns */     

#define I2C1_TIMING                      I2C_TIMING_100KHZ_SLEW

#endif /* HAL_I2C_MODULE_ENABLED */

   
/** @defgroup STM32H7XX_NUCLEO_144_BUS_Exported_Types BUS Exported Types
  * @{
  */

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
typedef struct
{
  pSPI_CallbackTypeDef  pMspSpiInitCb;
  pSPI_CallbackTypeDef  pMspSpiDeInitCb;
}BSP_SPI1_Cb_t;
#endif /* (USE_HAL_SPI_REGISTER_CALLBACKS == 1) */
/**
  * @}
  */ 


/** @defgroup STM32H7XX_NUCLEO_144_BUS_Exported_Functions BUS Exported Functions
  * @{
  */
int32_t BSP_SPI1_Init(void);
int32_t BSP_SPI1_DeInit(void);
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
int32_t BSP_SPI1_RegisterMspCallbacks(uint32_t Instance, BSP_SPI1_Cb_t *CallBacks);
int32_t BSP_SPI1_RegisterDefaultMspCallbacks(uint32_t Instance);
#endif /* (USE_HAL_SPI_REGISTER_CALLBACKS == 1) */
int32_t BSP_SPI1_Send(uint8_t *pTxData, uint32_t Length);
int32_t BSP_SPI1_Recv(uint8_t *pRxData, uint32_t Length);
int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint32_t Length);
HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef *phspi, uint32_t baudrate_presc);
int32_t BSP_GetTick(void);
#if defined(HAL_ADC_MODULE_ENABLED) || defined(USE_ADAFRUIT_SHIELD_V2)
//uint8_t          BSP_JOY_Init(void);
//JOYPin_TypeDef 	 BSP_JOY_GetState(void);
//void             BSP_JOY_DeInit(void);
#endif /* HAL_ADC_MODULE_ENABLED */
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

#endif /* STM32H7XX_NUCLEO_144_BUS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
