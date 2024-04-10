/*!
 * @file Adafruit_seesaw.h
 *
 * This is part of Adafruit's seesaw driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit products that use seesaw
 * technology.
 *
 * These chips use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the board.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Dean Miller for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 * 
 */
/*
 * Adapted to STM32Cube ecosystem by Evan Ross / Firmware Modules.
 * BSD license
 * 
 * In any project using the Adafruit_Shield BSP component with the 
 * Adafruit 1.8" TFT Shield V2 board, ensure to:
 *
 *   * define USE_ADAFRUIT_SHIELD_V2 in the preprocessor
 *   * set HAL_I2C_MODULE_ENABLE in stm32xxxx_hal_conf.h
 *
 */

#ifndef LIB_SEESAW_H
#define LIB_SEESAW_H

#include <stdbool.h>

 /*=========================================================================
     I2C ADDRESS/BITS
     -----------------------------------------------------------------------*/
#define SEESAW_ADDRESS (0x49) ///< Default Seesaw I2C address
     /*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/

    /** Module Base Addreses
    *  The module base addresses for different seesaw modules.
    */
enum {
    SEESAW_STATUS_BASE = 0x00,
    SEESAW_GPIO_BASE = 0x01,
    SEESAW_SERCOM0_BASE = 0x02,

    SEESAW_TIMER_BASE = 0x08,
    SEESAW_ADC_BASE = 0x09,
    SEESAW_DAC_BASE = 0x0A,
    SEESAW_INTERRUPT_BASE = 0x0B,
    SEESAW_DAP_BASE = 0x0C,
    SEESAW_EEPROM_BASE = 0x0D,
    SEESAW_NEOPIXEL_BASE = 0x0E,
    SEESAW_TOUCH_BASE = 0x0F,
    SEESAW_KEYPAD_BASE = 0x10,
    SEESAW_ENCODER_BASE = 0x11,
};

/** GPIO module function addres registers
 */
enum {
    SEESAW_GPIO_DIRSET_BULK = 0x02,
    SEESAW_GPIO_DIRCLR_BULK = 0x03,
    SEESAW_GPIO_BULK = 0x04,
    SEESAW_GPIO_BULK_SET = 0x05,
    SEESAW_GPIO_BULK_CLR = 0x06,
    SEESAW_GPIO_BULK_TOGGLE = 0x07,
    SEESAW_GPIO_INTENSET = 0x08,
    SEESAW_GPIO_INTENCLR = 0x09,
    SEESAW_GPIO_INTFLAG = 0x0A,
    SEESAW_GPIO_PULLENSET = 0x0B,
    SEESAW_GPIO_PULLENCLR = 0x0C,
};

/** status module function addres registers
 */
enum {
    SEESAW_STATUS_HW_ID = 0x01,
    SEESAW_STATUS_VERSION = 0x02,
    SEESAW_STATUS_OPTIONS = 0x03,
    SEESAW_STATUS_TEMP = 0x04,
    SEESAW_STATUS_SWRST = 0x7F,
};

/** timer module function addres registers
 */
enum {
    SEESAW_TIMER_STATUS = 0x00,
    SEESAW_TIMER_PWM = 0x01,
    SEESAW_TIMER_FREQ = 0x02,
};

/** ADC module function addres registers
 */
enum {
    SEESAW_ADC_STATUS = 0x00,
    SEESAW_ADC_INTEN = 0x02,
    SEESAW_ADC_INTENCLR = 0x03,
    SEESAW_ADC_WINMODE = 0x04,
    SEESAW_ADC_WINTHRESH = 0x05,
    SEESAW_ADC_CHANNEL_OFFSET = 0x07,
};

/** Sercom module function addres registers
 */
enum {
    SEESAW_SERCOM_STATUS = 0x00,
    SEESAW_SERCOM_INTEN = 0x02,
    SEESAW_SERCOM_INTENCLR = 0x03,
    SEESAW_SERCOM_BAUD = 0x04,
    SEESAW_SERCOM_DATA = 0x05,
};

/** neopixel module function addres registers
 */
enum {
    SEESAW_NEOPIXEL_STATUS = 0x00,
    SEESAW_NEOPIXEL_PIN = 0x01,
    SEESAW_NEOPIXEL_SPEED = 0x02,
    SEESAW_NEOPIXEL_BUF_LENGTH = 0x03,
    SEESAW_NEOPIXEL_BUF = 0x04,
    SEESAW_NEOPIXEL_SHOW = 0x05,
};

/** touch module function addres registers
 */
enum {
    SEESAW_TOUCH_CHANNEL_OFFSET = 0x10,
};

/** keypad module function addres registers
 */
enum {
    SEESAW_KEYPAD_STATUS = 0x00,
    SEESAW_KEYPAD_EVENT = 0x01,
    SEESAW_KEYPAD_INTENSET = 0x02,
    SEESAW_KEYPAD_INTENCLR = 0x03,
    SEESAW_KEYPAD_COUNT = 0x04,
    SEESAW_KEYPAD_FIFO = 0x10,
};

/** keypad module edge definitions
 */
enum {
    SEESAW_KEYPAD_EDGE_HIGH = 0,
    SEESAW_KEYPAD_EDGE_LOW,
    SEESAW_KEYPAD_EDGE_FALLING,
    SEESAW_KEYPAD_EDGE_RISING,
};

/** encoder module edge definitions
 */
enum {
    SEESAW_ENCODER_STATUS = 0x00,
    SEESAW_ENCODER_INTENSET = 0x02,
    SEESAW_ENCODER_INTENCLR = 0x03,
    SEESAW_ENCODER_POSITION = 0x04,
    SEESAW_ENCODER_DELTA = 0x05,
};

#define ADC_INPUT_0_PIN 2 ///< default ADC input pin
#define ADC_INPUT_1_PIN 3 ///< default ADC input pin
#define ADC_INPUT_2_PIN 4 ///< default ADC input pin
#define ADC_INPUT_3_PIN 5 ///< default ADC input pin

#define PWM_0_PIN 4 ///< default PWM output pin
#define PWM_1_PIN 5 ///< default PWM output pin
#define PWM_2_PIN 6 ///< default PWM output pin
#define PWM_3_PIN 7 ///< default PWM output pin


/* Arduino constants that MUST be defined as such. */
#define INPUT               0x00
#define OUTPUT              0x01
#define INPUT_PULLUP        0x02
#define INPUT_PULLDOWN      0x03 

/*=========================================================================*/

#define SEESAW_HW_ID_CODE 0x55 ///< seesaw HW ID code
#define SEESAW_EEPROM_I2C_ADDR                                                 \
  0x3F ///< EEPROM address of i2c address to start up with (for devices that
       ///< support this feature)

/** raw key event stucture for keypad module */
union keyEventRaw {
    struct {
        uint8_t EDGE : 2; ///< the edge that was triggered
        uint8_t NUM : 6;  ///< the event number
    } bit;              ///< bitfield format
    uint8_t reg;        ///< register format
};

/** extended key event stucture for keypad module */
union keyEvent {
    struct {
        uint8_t EDGE : 2;  ///< the edge that was triggered
        uint16_t NUM : 14; ///< the event number
    } bit;               ///< bitfield format
    uint16_t reg;        ///< register format
};

/** key state struct that will be written to seesaw chip keypad module */
union keyState {
    struct {
        uint8_t STATE : 1;  ///< the current state of the key
        uint8_t ACTIVE : 4; ///< the registered events for that key
    } bit;                ///< bitfield format
    uint8_t reg;          ///< register format
};

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with seesaw
   helper IC
*/
/**************************************************************************/


bool Adafruit_seesaw_init(uint8_t addr, bool reset);

uint32_t Adafruit_seesaw_getOptions(void);
uint32_t Adafruit_seesaw_getVersion(void);
void Adafruit_seesaw_SWReset(void);

void Adafruit_seesaw_pinMode(uint8_t pin, uint8_t mode);
void Adafruit_seesaw_pinModeBulk(uint32_t pins, uint8_t mode);
void Adafruit_seesaw_pinModeBulkB(uint32_t pinsa, uint32_t pinsb, uint8_t mode);
void Adafruit_seesaw_analogWrite(uint8_t pin, uint16_t value, uint8_t width);
void Adafruit_seesaw_digitalWrite(uint8_t pin, uint8_t value);
void Adafruit_seesaw_digitalWriteBulk(uint32_t pins, uint8_t value);
void Adafruit_seesaw_digitalWriteBulkB(uint32_t pinsa, uint32_t pinsb, uint8_t value);

bool Adafruit_seesaw_digitalRead(uint8_t pin);
uint32_t Adafruit_seesaw_digitalReadBulk(uint32_t pins);
uint32_t Adafruit_seesaw_digitalReadBulkB(uint32_t pins);

void Adafruit_seesaw_setGPIOInterrupts(uint32_t pins, bool enabled);

uint16_t Adafruit_seesaw_analogRead(uint8_t pin);

uint16_t Adafruit_seesaw_touchRead(uint8_t pin);

void Adafruit_seesaw_setPWMFreq(uint8_t pin, uint16_t freq);

void Adafruit_seesaw_enableSercomDataRdyInterrupt(uint8_t sercom);
void Adafruit_seesaw_disableSercomDataRdyInterrupt(uint8_t sercom);

char Adafruit_seesaw_readSercomData(uint8_t sercom);

void Adafruit_seesaw_EEPROMWrite8(uint8_t addr, uint8_t val);
void Adafruit_seesaw_EEPROMWrite(uint8_t addr, uint8_t* buf, uint8_t size);
uint8_t Adafruit_seesaw_EEPROMRead8(uint8_t addr);

void Adafruit_seesaw_setI2CAddr(uint8_t addr);
uint8_t Adafruit_seesaw_getI2CAddr(void);

void Adafruit_seesaw_UARTSetBaud(uint32_t baud);

void Adafruit_seesaw_setKeypadEvent(uint8_t key, uint8_t edge, bool enable);
void Adafruit_seesaw_enableKeypadInterrupt(void);
void Adafruit_seesaw_disableKeypadInterrupt(void);
uint8_t Adafruit_seesaw_getKeypadCount(void);
void Adafruit_seesaw_readKeypad(union keyEventRaw* buf, uint8_t count);

float Adafruit_seesaw_getTemp(void);

int32_t Adafruit_seesaw_getEncoderPosition(void);
int32_t Adafruit_seesaw_getEncoderDelta(void);
void Adafruit_seesaw_enableEncoderInterrupt(void);
void Adafruit_seesaw_disableEncoderInterrupt(void);
void Adafruit_seesaw_setEncoderPosition(int32_t pos);



#define ADAFRUIT_SEESAW_DELAY_DEFAULT_US       125

void Adafruit_seesaw_write8(uint8_t regHigh, uint8_t regLow, uint8_t value);
uint8_t Adafruit_seesaw_read8(uint8_t regHigh, uint8_t regLow, uint16_t delay_us);
void Adafruit_seesaw_read(uint8_t regHigh, uint8_t regLow, uint8_t* buf, uint8_t num, uint16_t delay_us);
void Adafruit_seesaw_write(uint8_t regHigh, uint8_t regLow, uint8_t* buf, uint8_t num);

/* SEESAW IO functions to be implemented in BSP */
void SEESAW_IO_Init(void);
void SEESAW_IO_Write(uint8_t Addr, uint16_t Reg, uint8_t Value);
uint8_t SEESAW_IO_Read(uint8_t Addr, uint16_t Reg);
uint16_t SEESAW_IO_ReadMultiple(uint8_t Addr, uint16_t Reg, uint8_t* Buffer, uint16_t Length, uint16_t Delay_us);
void SEESAW_IO_WriteMultiple(uint8_t Addr, uint16_t Reg, uint8_t* Buffer, uint16_t Length);
void SEESAW_IO_Delay(uint32_t Delay);
void SEESAW_IO_DelayMicroseconds(uint32_t Delay_us);
uint16_t SEESAW_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

/*=========================================================================
        REGISTER BITFIELDS
    -----------------------------------------------------------------------*/

    /** Sercom interrupt enable register
        */
/*
union sercom_inten {
    struct {
        uint8_t DATA_RDY : 1; ///< this bit is set when data becomes available
    } bit;                  ///< bitfields
    uint8_t reg;            ///< full register
} _sercom_inten; ///< sercom interrupt enable register instance
*/
    /*=========================================================================*/


#endif
