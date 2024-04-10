/**
  ******************************************************************************
  * @file    stm32h7xx_nucleo.c
  * @author  MCD Application Team
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32H7xx-Nucleo Kit
  *            from STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_nucleo.h"
#include "../Components/adafruit_seesaw/adafruit_seesaw.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32H7XX_NUCLEO
  * @{
  */

/** @addtogroup STM32H7XX_NUCLEO_LOW_LEVEL
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32H7xx-Nucleo Kit from STMicroelectronics.
  * @{
  */

/** @defgroup STM32H7XX_NUCLEO_LOW_LEVEL_Private_Defines LOW LEVEL Private Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32H7XX_NUCLEO_LOW_LEVEL_Private_TypesDefinitions LOW LEVEL Private Typedef
  * @{
  */
typedef void (* BSP_EXTI_LineCallback) (void);
/**
  * @}
  */

/** @defgroup STM32H7XX_NUCLEO_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Variables
  * @{
  */
EXTI_HandleTypeDef hpb_exti[BUTTONn];
#if (USE_BSP_COM_FEATURE > 0)
UART_HandleTypeDef hcom_uart[COMn];
USART_TypeDef* COM_USART[COMn]   = {COM1_UART};
#endif
/**
  * @}
  */
/** @defgroup STM32H7XX_NUCLEO_LOW_LEVEL_Private_Variables LOW LEVEL Private Variables
  * @{
  */
static GPIO_TypeDef* LED_PORT[LEDn] = {LED1_GPIO_PORT,
                                       LED2_GPIO_PORT,
                                       LED3_GPIO_PORT};

static const uint16_t LED_PIN[LEDn] = {LED1_PIN,
                                       LED2_PIN,
                                       LED3_PIN};

static GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {BUTTON_USER_GPIO_PORT};
static const uint16_t BUTTON_PIN[BUTTONn] = {BUTTON_USER_PIN};
static const IRQn_Type BUTTON_IRQn[BUTTONn] = {BUTTON_USER_EXTI_IRQn};



#if (USE_BSP_COM_FEATURE > 0)
  #if (USE_COM_LOG > 0)
     static COM_TypeDef COM_ActiveLogPort = COM1;
  #endif
  #if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
     static uint32_t IsComMspCbValid[COMn] = {0};
  #endif
#endif


#if defined(HAL_I2C_MODULE_ENABLED) && defined (USE_ADAFRUIT_SHIELD_V2)
uint32_t I2cxTimeout = NUCLEO_I2C1_TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */
static I2C_HandleTypeDef hnucleo_I2c1;
#endif /* HAL_I2C_MODULE_ENABLED */

#if defined(HAL_I2C_MODULE_ENABLED) && defined (USE_ADAFRUIT_SHIELD_V2)
/* I2C1 bus function */
/* Link function for I2C SEESAW peripheral */
static void               I2C1_Init(void);
static void               I2C1_Error(void);
static void               I2C1_MspInit(I2C_HandleTypeDef* hi2c);
static void               I2C1_Write(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value);
static uint8_t            I2C1_Read(uint8_t Addr, uint16_t Reg, uint16_t RegSize);
static HAL_StatusTypeDef  I2C1_WriteBuffer(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t* pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2C1_ReadBuffer(uint8_t Addr, uint8_t* pBuffer, uint16_t Length);
static HAL_StatusTypeDef  I2C1_SendAddr(uint8_t Addr, uint16_t Reg, uint16_t RegSize);
static HAL_StatusTypeDef  I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

#endif /* HAL_I2C_MODULE_ENABLED */

/**
  * @}
  */

/** @defgroup STM32H7XX_NUCLEO_LOW_LEVEL_Private_FunctionPrototypes LOW LEVEL Private functions Prototypes
  * @{
  */
static void BUTTON_USER_EXTI_Callback(void);
#if (USE_BSP_COM_FEATURE > 0)
static void COM1_MspInit(UART_HandleTypeDef *huart);
static void COM1_MspDeInit(UART_HandleTypeDef *huart);
#endif
/**
  * @}
  */

/** @defgroup STM32H7XX_NUCLEO_LOW_LEVEL_Exported_Functions LOW LEVEL Exported Functions
  * @{
  */

/**
  * @brief  This method returns the STM32H7XX NUCLEO BSP Driver revision
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
int32_t BSP_GetVersion(void)
{
  return (int32_t)STM32H7XX_NUCLEO_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval BSP status
  */
int32_t BSP_LED_Init(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;
  GPIO_InitTypeDef  gpio_init_structure;

  if((Led != LED1) && (Led != LED2) && (Led != LED3))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Enable the GPIO LED Clock */
    if(Led == LED1)
    {
      LED1_GPIO_CLK_ENABLE();
    }
    else if(Led == LED2)
    {
      LED2_GPIO_CLK_ENABLE();
    }
    else
    {
      LED3_GPIO_CLK_ENABLE();
    }
    /* Configure the GPIO_LED pin */
    gpio_init_structure.Pin   = LED_PIN[Led];
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(LED_PORT[Led], &gpio_init_structure);
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  }

  return ret;
}

/**
  * @brief  DeInit LEDs.
  * @param  Led LED to be de-init.
  *   This parameter can be one of the following values:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
  * @retval BSP status
  */
int32_t BSP_LED_DeInit(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;
  GPIO_InitTypeDef  gpio_init_structure;

  if((Led != LED1) && (Led != LED2) && (Led != LED3))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Turn off LED */
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
    /* DeInit the GPIO_LED pin */
    gpio_init_structure.Pin = LED_PIN[Led];
    HAL_GPIO_DeInit(LED_PORT[Led], gpio_init_structure.Pin);
  }

  return ret;
}

/**
  * @brief  Turns selected LED On.
  * @param  Led Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval BSP status
  */
int32_t BSP_LED_On(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;

  if((Led != LED1) && (Led != LED2) && (Led != LED3))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
  }

  return ret;
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval BSP status
  */
int32_t BSP_LED_Off(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;

  if((Led != LED1) && (Led != LED2) && (Led != LED3))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  }

  return ret;
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval BSP status
  */
int32_t BSP_LED_Toggle(Led_TypeDef Led)
{
  int32_t ret = BSP_ERROR_NONE;

  if((Led != LED1) && (Led != LED2) && (Led != LED3))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
  }

  return ret;
}

/**
  * @brief  Get the state of the selected LED.
  * @param  Led LED to get its state
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval LED status
  */
int32_t BSP_LED_GetState (Led_TypeDef Led)
{
  int32_t ret;

  if((Led != LED1) && (Led != LED2) && (Led != LED3))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ret = (int32_t)HAL_GPIO_ReadPin (LED_PORT [Led], LED_PIN [Led]);
  }

  return ret;
}

/**
  * @brief  Configures button GPIO and EXTI Line.
  * @param  Button Button to be configured
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_USER: Wakeup Push Button
  * @param  ButtonMode Button mode
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *            @arg  BUTTON_MODE_EXTI: Button will be connected to EXTI line
  *                                    with interrupt generation capability
  */
int32_t BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpio_init_structure;
  static BSP_EXTI_LineCallback ButtonCallback[BUTTONn] = {BUTTON_USER_EXTI_Callback};
  static uint32_t  BSP_BUTTON_PRIO [BUTTONn] = {BSP_BUTTON_USER_IT_PRIORITY};
  static const uint32_t BUTTON_EXTI_LINE[BUTTONn] = {BUTTON_USER_EXTI_LINE};

  /* Enable the BUTTON clock */
  BUTTON_USER_GPIO_CLK_ENABLE();

  gpio_init_structure.Pin = BUTTON_PIN [Button];
  gpio_init_structure.Pull = GPIO_PULLDOWN;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;

  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(BUTTON_PORT [Button], &gpio_init_structure);
  }
  else /* (ButtonMode == BUTTON_MODE_EXTI) */
  {
    /* Configure Button pin as input with External interrupt */
    gpio_init_structure.Mode = GPIO_MODE_IT_RISING;

    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);

    (void)HAL_EXTI_GetHandle(&hpb_exti[Button], BUTTON_EXTI_LINE[Button]);
    (void)HAL_EXTI_RegisterCallback(&hpb_exti[Button],  HAL_EXTI_COMMON_CB_ID, ButtonCallback[Button]);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((BUTTON_IRQn[Button]), BSP_BUTTON_PRIO[Button], 0x00);
    HAL_NVIC_EnableIRQ((BUTTON_IRQn[Button]));
  }

  return BSP_ERROR_NONE;
}

/**
  * @brief  Push Button DeInit.
  * @param  Button Button to be configured
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_USER: Wakeup Push Button
  * @note PB DeInit does not disable the GPIO clock
  */
int32_t BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;

  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
  * @brief  Returns the selected button state.
  * @param  Button Button to be checked
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_USER: Wakeup Push Button
  * @retval The Button GPIO pin value (GPIO_PIN_RESET = button pressed)
  */
int32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return (int32_t)HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  BSP Button IRQ handler
  * @param  Button Can only be BUTTON_USER
  * @retval None
  */
void BSP_PB_IRQHandler(Button_TypeDef Button)
{
  HAL_EXTI_IRQHandler(&hpb_exti[Button]);
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None
  */
__weak void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Button);

  /* This function should be implemented by the user application.
     It is called into this driver when an event on Button is triggered. */
}

#if (USE_BSP_COM_FEATURE > 0)
/**
  * @brief  Configures COM port.
  * @param  COM COM port to be configured.
  *          This parameter can be COM1
  * @param  COM_Init Pointer to a UART_HandleTypeDef structure that contains the
  *                  configuration information for the specified USART peripheral.
  * @retval BSP error code
  */
int32_t BSP_COM_Init(COM_TypeDef COM, COM_InitTypeDef *COM_Init)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
#if (USE_HAL_UART_REGISTER_CALLBACKS == 0)
    /* Init the UART Msp */
    COM1_MspInit(&hcom_uart[COM]);
#else
    if(IsComMspCbValid[COM] == 0U)
    {
      if(BSP_COM_RegisterDefaultMspCallbacks(COM) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif

    if(MX_USART3_Init(&hcom_uart[COM], COM_Init) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  DeInit COM port.
  * @param  COM COM port to be configured.
  *          This parameter can be COM1
  * @retval BSP status
  */
int32_t BSP_COM_DeInit(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* USART configuration */
    hcom_uart[COM].Instance = COM_USART[COM];

#if (USE_HAL_UART_REGISTER_CALLBACKS == 0)
    COM1_MspDeInit(&hcom_uart[COM]);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 0) */

    if(HAL_UART_DeInit(&hcom_uart[COM]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  Configures COM port.
  * @param  huart USART handle
  * @param  COM_Init Pointer to a UART_HandleTypeDef structure that contains the
  *                  configuration information for the specified USART peripheral.
  * @retval HAL error code
  */
__weak HAL_StatusTypeDef MX_USART3_Init(UART_HandleTypeDef *huart, MX_UART_InitTypeDef *COM_Init)
{
  /* USART configuration */
 huart->Instance          = COM_USART[COM1];
 huart->Init.BaudRate     = COM_Init->BaudRate;
 huart->Init.Mode         = UART_MODE_TX_RX;
 huart->Init.Parity       = (uint32_t)COM_Init->Parity;
 huart->Init.WordLength   = (uint32_t)COM_Init->WordLength;
 huart->Init.StopBits     = (uint32_t)COM_Init->StopBits;
 huart->Init.HwFlowCtl    = (uint32_t)COM_Init->HwFlowCtl;
 huart->Init.OverSampling = UART_OVERSAMPLING_8;

 return HAL_UART_Init(huart);
}

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
/**
  * @brief Register Default COM Msp Callbacks
  * @param  COM COM port to be configured.
  *          This parameter can be COM1
  * @retval BSP status
  */
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);

    /* Register default MspInit/MspDeInit Callback */
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, COM1_MspInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, COM1_MspDeInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      IsComMspCbValid[COM] = 1U;
    }
  }

  /* BSP status */
  return ret;
}

/**
  * @brief Register COM Msp Callback registering
  * @param  COM COM port to be configured.
  *          This parameter can be COM1
  * @param Callbacks     pointer to COM1 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_COM_RegisterMspCallbacks(COM_TypeDef COM , BSP_COM_Cb_t *Callback)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);

    /* Register MspInit/MspDeInit Callbacks */
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, Callback->pMspInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, Callback->pMspDeInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      IsComMspCbValid[COM] = 1U;
    }
  }
  /* BSP status */
  return ret;
}
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

#if (USE_COM_LOG > 0)
/**
  * @brief  Select the active COM port.
  * @param  COM COM port to be activated.
  *          This parameter can be COM1
  * @retval BSP status
  */
int32_t BSP_COM_SelectLogPort(COM_TypeDef COM)
{
  if(COM_ActiveLogPort != COM)
  {
    COM_ActiveLogPort = COM;
  }
  return BSP_ERROR_NONE;
}

/**
  * @brief  Redirect console output to COM
  */
 #ifdef __GNUC__
 int __io_putchar (int ch)
 #else
 int fputc (int ch, FILE *f)
 #endif /* __GNUC__ */
{
  HAL_UART_Transmit (&hcom_uart [COM_ActiveLogPort], (uint8_t *) &ch, 1, COM_POLL_TIMEOUT);
  return ch;
}
#endif /* USE_COM_LOG */
#endif /* USE_BSP_COM_FEATURE */

/**
  * @}
  */

/** @defgroup STM32H7XX_NUCLEO_LOW_LEVEL_Private_Functions LOW LEVEL Private functions
  * @{
  */
/**
  * @brief  Key EXTI line detection callbacks.
  * @retval BSP status
  */
static void BUTTON_USER_EXTI_Callback(void)
{
  BSP_PB_Callback(BUTTON_USER);
}

#if (USE_BSP_COM_FEATURE > 0)
/**
  * @brief  Initializes UART MSP.
  * @param  huart UART handle
  * @retval BSP status
  */
static void COM1_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* Enable GPIO clock */
  COM1_TX_GPIO_CLK_ENABLE();
  COM1_RX_GPIO_CLK_ENABLE();

  /* Enable USART clock */
  COM1_CLK_ENABLE();

  /* Configure USART Tx as alternate function */
  gpio_init_structure.Pin = COM1_TX_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Alternate = COM1_TX_AF;
  HAL_GPIO_Init(COM1_TX_GPIO_PORT, &gpio_init_structure);

  /* Configure USART Rx as alternate function */
  gpio_init_structure.Pin = COM1_RX_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Alternate = COM1_RX_AF;
  HAL_GPIO_Init(COM1_RX_GPIO_PORT, &gpio_init_structure);
}

/**
  * @brief  Initialize USART3 Msp part
  * @param  huart UART handle
  * @retval BSP status
  */
static void COM1_MspDeInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef          gpio_init_structure;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* COM GPIO pin configuration */
  gpio_init_structure.Pin  = COM1_TX_PIN;
  HAL_GPIO_DeInit(COM1_TX_GPIO_PORT, gpio_init_structure.Pin);

  gpio_init_structure.Pin  = COM1_RX_PIN;
  HAL_GPIO_DeInit(COM1_RX_GPIO_PORT, gpio_init_structure.Pin);

  /* Disable USART clock */
  COM1_CLK_DISABLE();
}
#endif /* USE_BSP_COM_FEATURE */


/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
#if defined(HAL_I2C_MODULE_ENABLED) && defined (USE_ADAFRUIT_SHIELD_V2)
/******************************* I2C Routines *********************************/

/**
  * @brief I2C Bus initialization
  * @param None
  * @retval None
  */
static void I2C1_Init(void)
{
    if (HAL_I2C_GetState(&hnucleo_I2c1) == HAL_I2C_STATE_RESET)
    {
        hnucleo_I2c1.Instance = NUCLEO_I2C1;
        hnucleo_I2c1.Init.Timing = I2C1_TIMING;
        hnucleo_I2c1.Init.OwnAddress1 = 0;
        hnucleo_I2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hnucleo_I2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hnucleo_I2c1.Init.OwnAddress2 = 0;
        hnucleo_I2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
        hnucleo_I2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hnucleo_I2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        /* Init the I2C */
        I2C1_MspInit(&hnucleo_I2c1);
        HAL_I2C_Init(&hnucleo_I2c1);
    }
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
static void I2C1_Write(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    status = HAL_I2C_Mem_Write(&hnucleo_I2c1, Addr, Reg, RegSize, &Value, 1, 100);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Execute user timeout callback */
        I2C1_Error();
    }
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
static uint8_t I2C1_Read(uint8_t Addr, uint16_t Reg, uint16_t RegSize)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Value = 0;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    status = HAL_I2C_Mem_Read(&hnucleo_I2c1, Addr, Reg, RegSize, &Value, 1, 1000);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Execute user timeout callback */
        I2C1_Error();
    }
    return Value;
}


/**
  * @brief  Sends address I2C transaction with STOP.
  * @note   Used in conjunction with I2C1_ReadBuffer to follow up with the read.
  * @param  Addr  : I2C Address
  * @param  Reg   : Reg Address
  * @param  RegSize : The target register size (can be 8BIT or 16BIT)
  * @retval 0 if no problems to send address
  */
static HAL_StatusTypeDef I2C1_SendAddr(uint8_t Addr, uint16_t Reg, uint16_t RegSize)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    /* Master transaction to set register address. */
    uint8_t buf[2];
    uint16_t size;
    
    if (RegSize == I2C_MEMADD_SIZE_16BIT)
    {
        buf[0] = Reg >> 8;
        buf[1] = Reg;
        size = 2;
    }
    else
    {
        buf[0] = Reg;
        size = 1;
    }    
    
    status = HAL_I2C_Master_Transmit(&hnucleo_I2c1, Addr, buf, size, I2cxTimeout);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Re-Initiaize the BUS */
        I2C1_Error();
    }
    return status;
}

/**
  * @brief  Reads multiple data on the BUS.
  * @note   To be preceded by I2C1_SendAddr
  * @param  Addr  : I2C Address
  * @param  pBuffer : pointer to read data buffer
  * @param  Length : length of the data
  * @retval 0 if no problems to read multiple data
  */
static HAL_StatusTypeDef I2C1_ReadBuffer(uint8_t Addr, uint8_t* pBuffer, uint16_t Length)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    /* Master transaction to receive data, having already sent register address. */
    status = HAL_I2C_Master_Receive(&hnucleo_I2c1, Addr, pBuffer, Length,
       I2cxTimeout);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Re-Initiaize the BUS */
        I2C1_Error();
    }
    return status;
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
    /* Shift address left per HAL API requirement */
    DevAddress <<= 1;

    return (HAL_I2C_IsDeviceReady(&hnucleo_I2c1, DevAddress, Trials, I2cxTimeout));
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2C1_WriteBuffer(uint8_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t* pBuffer, uint16_t Length)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Shift address left per HAL API requirement */
    Addr <<= 1;

    status = HAL_I2C_Mem_Write(&hnucleo_I2c1, Addr, Reg, RegSize, pBuffer, Length, I2cxTimeout);

    /* Check the communication status */
    if (status != HAL_OK)
    {
        /* Re-Initiaize the BUS */
        I2C1_Error();
    }
    return status;
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  None
  * @retval None
  */
static void I2C1_Error(void)
{
    /* De-initialize the I2C communication BUS */
    HAL_I2C_DeInit(&hnucleo_I2c1);

    /* Re-Initiaize the I2C communication BUS */
    I2C1_Init();
}

/**
  * @brief I2C MSP Initialization
  * @param hi2c: I2C handle
  * @retval None
  */
static void I2C1_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

    /*##-1- Set source clock to PCLK1 for I2C1 ################################################*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/

    /* Enable GPIO clock */
    NUCLEO_I2C1_GPIO_CLK_ENABLE();

    /* Configure I2C SCL & SDA as alternate function  */
    GPIO_InitStruct.Pin = (NUCLEO_I2C1_SCL_PIN | NUCLEO_I2C1_SDA_PIN);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = NUCLEO_I2C1_SCL_SDA_AF;
    HAL_GPIO_Init(NUCLEO_I2C1_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the Eval I2C peripheral #######################################*/
    /* Enable I2C clock */
    NUCLEO_I2C1_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    NUCLEO_I2C1_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    NUCLEO_I2C1_RELEASE_RESET();
}


#endif /*HAL_I2C_MODULE_ENABLED*/



/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK SEESAW ********************************/

#if defined(HAL_I2C_MODULE_ENABLED) && defined(USE_ADAFRUIT_SHIELD_V2)

/**
 * @brief  SEESAW delay
 * @param  Delay: Delay in ms
 * @retval None
 */
void SEESAW_IO_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

void SEESAW_IO_DelayMicroseconds(uint32_t Delay_us)
{
    /* Round up to nearest millisecond. */

    uint32_t delay_ms = ((Delay_us)+999) / 1000;
    HAL_Delay(delay_ms);
}

/**
  * @brief  Initializes IOE low level.
  * @param  None
  * @retval None
  */
void SEESAW_IO_Init(void)
{
    I2C1_Init();
}
/**
  * @brief  SEESAW writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
void SEESAW_IO_Write(uint8_t Addr, uint16_t Reg, uint8_t Value)
{
    I2C1_Write(Addr, Reg, I2C_MEMADD_SIZE_16BIT, Value);
}

/**
  * @brief  SEESAW reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t SEESAW_IO_Read(uint8_t Addr, uint16_t Reg)
{
    return I2C1_Read(Addr, Reg, I2C_MEMADD_SIZE_16BIT);
}

/**
  * @brief  SEESAW reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t SEESAW_IO_ReadMultiple(uint8_t Addr, uint16_t Reg, uint8_t* Buffer, uint16_t Length, uint16_t Delay_us)
{
    HAL_StatusTypeDef status;

    status = I2C1_SendAddr(Addr, Reg, I2C_MEMADD_SIZE_16BIT);

    if (status == HAL_OK)
    {
        SEESAW_IO_DelayMicroseconds(Delay_us);

        status = I2C1_ReadBuffer(Addr, Buffer, Length);
    }

    return status;
}

/**
  * @brief  SEESAW  writes multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */

void SEESAW_IO_WriteMultiple(uint8_t Addr, uint16_t Reg, uint8_t *Buffer, uint16_t Length)
{
    I2C1_WriteBuffer(Addr, Reg, I2C_MEMADD_SIZE_16BIT, Buffer, Length);
}




/**
  * @brief  Checks if Temperature Sensor is ready for communication.
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
uint16_t SEESAW_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
    return (I2C1_IsDeviceReady(DevAddress, Trials));
}

#endif /* HAL_I2C_MODULE_ENABLED */



/******************************* LINK JOYSTICK ********************************/

#if USE_ADAFRUIT_SHIELD_V2
//#include "adafruit_802.h"

/**
  * @brief  Configures joystick available on adafruit 1.8" TFT shield V2
  *         managed through seesaw.
  * @retval Joystickstatus (0=> success, 1=> fail)
  */
uint8_t BSP_JOY_Init(void)
{
    /* Note: don't do a software reset of seesaw - this should 
     * have been done with LCD module init 
     */
    if (Adafruit_seesaw_init(TFTSHIELD_ADDR, false) != true)
    {
        return (uint8_t)HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  DeInit joystick GPIOs.
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
    /* Do nothing */
}

/**
  * @brief  Returns the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to read seesaw.
  * @retval JOYState_TypeDef: Code of the Joystick key pressed.
  */
JOYState_TypeDef BSP_JOY_GetState(void)
{
    JOYState_TypeDef state = JOY_NONE;
    uint32_t  keyconvertedvalue = 0;

    /* Start the conversion process */
    keyconvertedvalue = Adafruit_seesaw_digitalReadBulk(TFTSHIELD_BUTTON_ALL);

#if 0 /* diagnostics */
    if (keyconvertedvalue != 0) {
        char buf[12];
        sprintf(buf, "%08x", keyconvertedvalue);
        BSP_LCD_DisplayStringAtLine(12, (uint8_t*)buf);
    }
#endif

    if ((keyconvertedvalue & TFTSHIELD_BUTTON_UP) == 0)
    {
        state = JOY_UP;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_RIGHT) == 0)
    {
        state = JOY_RIGHT;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_IN) == 0)
    {
        state = JOY_SEL;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_DOWN) == 0)
    {
        state = JOY_DOWN;
    }
    else if ((keyconvertedvalue & TFTSHIELD_BUTTON_LEFT) == 0)
    {
        state = JOY_LEFT;
    }
    else
    {
        state = JOY_NONE;
    }

    /* Return the code of the Joystick key pressed */
    return state;
}

/**
  * @brief  Returns the Button key pressed.
  * @note   To know which Button key is pressed we need to read seesaw.
  * @retval BUTTONState_TypeDef: Code of the Button key pressed.
  */
BUTTONState_TypeDef	BSP_BUTTONS_GetState(void)
{
	BUTTONState_TypeDef state = BUTTON_NONE;
	uint32_t  keyconvertedvalue = 0;

	/* Start the conversion process */
	keyconvertedvalue = Adafruit_seesaw_digitalReadBulk(TFTSHIELD_BUTTON_ALL);

	if ((keyconvertedvalue & TFTSHIELD_BUTTON_1) == 0)
	{
			state = BUTTON_A;
	}
	else if ((keyconvertedvalue & TFTSHIELD_BUTTON_2) == 0)
	{
			state = BUTTON_B;
	}
	else if ((keyconvertedvalue & TFTSHIELD_BUTTON_3) == 0)
	{
			state = BUTTON_C;
	}
	else
	{
			state = BUTTON_NONE;
	}

	/* Return the code of the Button key pressed */
	return state;
}

#else /* USE_ADAFRUIT_SHIELD_V2 */

#ifdef HAL_ADC_MODULE_ENABLED
/**
  * @brief  Configures joystick available on adafruit 1.8" TFT shield 
  *         managed through ADC to detect motion.
  * @retval Joystickstatus (0=> success, 1=> fail) 
  */
uint8_t BSP_JOY_Init(void)
{
  if (ADCx_Init() != HAL_OK)
  {
    return (uint8_t) HAL_ERROR; 
  }
  
  /* Select Channel 8 to be converted */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

  /* Return Joystick initialization status */
  return (uint8_t) HAL_ADC_ConfigChannel(&hnucleo_Adc, &sConfig);
}

/**
  * @brief  DeInit joystick GPIOs.
  * @note   JOY DeInit does not disable the Mfx, just set the Mfx pins in Off mode
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
    ADCx_DeInit();
}

/**
  * @brief  Returns the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to detect the voltage
  *         level on each key output
  *           - None  : 3.3 V / 4095
  *           - SEL   : 1.055 V / 1308
  *           - DOWN  : 0.71 V / 88
  *           - LEFT  : 3.0 V / 3720 
  *           - RIGHT : 0.595 V / 737
  *           - UP    : 1.65 V / 2046
  * @retval JOYState_TypeDef: Code of the Joystick key pressed.
  */
JOYState_TypeDef BSP_JOY_GetState(void)
{
  JOYState_TypeDef state = JOY_NONE;
  uint16_t  keyconvertedvalue = 0;
  
 /* Start the conversion process */
  HAL_ADC_Start(&hnucleo_Adc);
  
  /* Wait for the end of conversion */
  if (HAL_ADC_PollForConversion(&hnucleo_Adc, 10) != HAL_TIMEOUT)
  {
    /* Get the converted value of regular channel */
    keyconvertedvalue = HAL_ADC_GetValue(&hnucleo_Adc);
  }
 
  if((keyconvertedvalue > 2010) && (keyconvertedvalue < 2090))
  {
    state = JOY_UP;
  }
  else if((keyconvertedvalue > 680) && (keyconvertedvalue < 780))
  {
    state = JOY_RIGHT;
  }
  else if((keyconvertedvalue > 1270) && (keyconvertedvalue < 1350))
  {
    state = JOY_SEL;
  }
  else if((keyconvertedvalue > 50) && (keyconvertedvalue < 130))
  {
    state = JOY_DOWN;
  }
  else if((keyconvertedvalue > 3570) && (keyconvertedvalue < 3800))
  {
    state = JOY_LEFT;
  }
  else
  {
    state = JOY_NONE;
  }
  
  /* Return the code of the Joystick key pressed */
  return state;
}
#endif /* HAL_ADC_MODULE_ENABLED */

#endif /* USE_ADAFRUIT_SHIELD_V2 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
