/*!
 * @file Adafruit_seesaw.cpp
 *
 * @mainpage Adafruit seesaw arduino driver
 *
 * @section intro_sec Introduction
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
 * @section author Author
 *
 * Written by Dean Miller for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
 /*
  * Adapted to STM32Cube ecosystem by Evan Ross / Firmware Modules.
  * BSD license
  */

#include <stdint.h>
#include <stdbool.h>
#include "adafruit_seesaw.h"


static uint8_t _i2caddr;

/*!
 *****************************************************************************************
 *  @brief      Start the seesaw
 *
 *				This should be called when your sketch is
 *connecting to the seesaw
 *
 *  @param      addr the I2C address of the seesaw
 *  @param		reset pass true to reset the seesaw on startup. Defaults
 *to true.
 *
 *  @return     true if we could connect to the seesaw, false otherwise
 ****************************************************************************************/
bool Adafruit_seesaw_init(uint8_t addr, bool reset) 
{
    _i2caddr = addr;

    SEESAW_IO_Init();

    if (reset) {
        Adafruit_seesaw_SWReset();
        SEESAW_IO_Delay(500);
    }

    uint8_t c = Adafruit_seesaw_read8(SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
    if (c != SEESAW_HW_ID_CODE) {
        return false;
    }
    return true;
}

/*!
 *******************************************************************
 *  @brief      perform a software reset. This resets all seesaw registers to
 *their default values.
 *  			This is called automatically from
 *Adafruit_seesaw.begin()

 ********************************************************************/
void Adafruit_seesaw_SWReset(void) {
    Adafruit_seesaw_write8(SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xFF);
}

/*!
 **************************************************************************
 *  @brief      Returns the available options compiled into the seesaw firmware.
 *  @return     the available options compiled into the seesaw firmware. If the
 *option is included, the corresponding bit is set. For example, if the ADC
 *module is compiled in then (ss.getOptions() & (1UL << SEESAW_ADC_BASE)) > 0
 ***********************************************************************/
uint32_t Adafruit_seesaw_getOptions(void) {
    uint8_t buf[4];
    Adafruit_seesaw_read(SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS, buf, 4, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
    uint32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
        ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret;
}

/*!
 *********************************************************************
 *  @brief      Returns the version of the seesaw
 *  @return     The version code. Bits [31:16] will be a date code, [15:0] will
 *be the product id.
 ********************************************************************/
uint32_t Adafruit_seesaw_getVersion(void) {
    uint8_t buf[4];
    Adafruit_seesaw_read(SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION, buf, 4, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
    uint32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
        ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret;
}

/*!
 **************************************************************************
 *  @brief      Set the mode of a GPIO pin.
 *
 *  @param      pin the pin number. On the SAMD09 breakout, this corresponds to
 *the number on the silkscreen.
 *  @param		mode the mode to set the pin. One of INPUT, OUTPUT, or
 *INPUT_PULLUP.

 ************************************************************************/
void Adafruit_seesaw_pinMode(uint8_t pin, uint8_t mode) {
    if (pin >= 32)
        Adafruit_seesaw_pinModeBulkB(0, 1ul << (pin - 32), mode);
    else
        Adafruit_seesaw_pinModeBulk(1ul << pin, mode);
}

/*!
 ***************************************************************************
 *  @brief      Set the output of a GPIO pin
 *
 *  @param      pin the pin number. On the SAMD09 breakout, this corresponds to
 *the number on the silkscreen.
 *	@param		value the value to write to the GPIO pin. This should be
 *HIGH or LOW.
 ***************************************************************************/
void Adafruit_seesaw_digitalWrite(uint8_t pin, uint8_t value) {
    if (pin >= 32)
        Adafruit_seesaw_digitalWriteBulkB(0, 1ul << (pin - 32), value);
    else
        Adafruit_seesaw_digitalWriteBulk(1ul << pin, value);
}

/*!
 ****************************************************************************
 *  @brief      Read the current status of a GPIO pin
 *
 *  @param      pin the pin number. On the SAMD09 breakout, this corresponds to
 *the number on the silkscreen.
 *
 *  @return     the status of the pin. HIGH or LOW (1 or 0).
 ***********************************************************************/
bool Adafruit_seesaw_digitalRead(uint8_t pin) {
    if (pin >= 32)
        return Adafruit_seesaw_digitalReadBulkB((1ul << (pin - 32))) != 0;
    else
        return Adafruit_seesaw_digitalReadBulk((1ul << pin)) != 0;
}

/*!
 ****************************************************************************
 *  @brief      read the status of multiple pins on port A.
 *
 *  @param      pins a bitmask of the pins to write. On the SAMD09 breakout,
 *this corresponds to the number on the silkscreen. For example, passing 0b0110
 *will return the values of pins 2 and 3.
 *
 *  @return     the status of the passed pins. If 0b0110 was passed and pin 2 is
 *high and pin 3 is low, 0b0010 (decimal number 2) will be returned.
 *******************************************************************/
uint32_t Adafruit_seesaw_digitalReadBulk(uint32_t pins) {
    uint8_t buf[4];
    Adafruit_seesaw_read(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, buf, 4, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
    uint32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
        ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret & pins;
}

/*!
 **************************************************************************
 *  @brief      read the status of multiple pins on port B.
 *
 *  @param      pins a bitmask of the pins to write.
 *
 *  @return     the status of the passed pins. If 0b0110 was passed and pin 2 is
 *high and pin 3 is low, 0b0010 (decimal number 2) will be returned.
 ************************************************************************/
uint32_t Adafruit_seesaw_digitalReadBulkB(uint32_t pins) {
    uint8_t buf[8];
    Adafruit_seesaw_read(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, buf, 8, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
    uint32_t ret = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16) |
        ((uint32_t)buf[6] << 8) | (uint32_t)buf[7];
    return ret & pins;
}

/*!
 **********************************************************************
 *  @brief      Enable or disable GPIO interrupts on the passed pins
 *
 *  @param      pins a bitmask of the pins to write. On the SAMD09 breakout,
 *this corresponds to the number on the silkscreen. For example, passing 0b0110
 *will enable or disable interrups on pins 2 and 3.
 *	@param		enabled pass true to enable the interrupts on the passed
 *pins, false to disable the interrupts on the passed pins.
 ***********************************************************************/
void Adafruit_seesaw_setGPIOInterrupts(uint32_t pins, bool enabled) {
    uint8_t cmd[] = { (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                     (uint8_t)(pins >> 8), (uint8_t)pins };
    if (enabled)
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_INTENSET, cmd, 4);
    else
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_INTENCLR, cmd, 4);
}

/*!
 ****************************************************************
 *  @brief      read the analog value on an ADC-enabled pin.
 *
 *  @param      pin the number of the pin to read. On the SAMD09 breakout, this
 *corresponds to the number on the silkscreen. On the default seesaw firmware on
 *the SAMD09 breakout, pins 2, 3, and 4 are ADC-enabled.
 *
 *  @return     the analog value. This is an integer between 0 and 1023
 ***********************************************************************/
uint16_t Adafruit_seesaw_analogRead(uint8_t pin) {
    uint8_t buf[2];
    uint8_t p;
    switch (pin) {
    case ADC_INPUT_0_PIN:
        p = 0;
        break;
    case ADC_INPUT_1_PIN:
        p = 1;
        break;
    case ADC_INPUT_2_PIN:
        p = 2;
        break;
    case ADC_INPUT_3_PIN:
        p = 3;
        break;
    default:
        return 0;
    }

    Adafruit_seesaw_read(SEESAW_ADC_BASE, SEESAW_ADC_CHANNEL_OFFSET + p, buf, 2, 500);
    uint16_t ret = ((uint16_t)buf[0] << 8) | buf[1];
    SEESAW_IO_Delay(1);
    return ret;
}

/*!
 ******************************************************************************
 *  @brief      read the analog value on an capacitive touch-enabled pin.
 *
 *  @param      pin the number of the pin to read.
 *
 *  @return     the analog value. This is an integer between 0 and 1023
 ****************************************************************************/
uint16_t Adafruit_seesaw_touchRead(uint8_t pin) {
    uint8_t buf[2];
    uint8_t p = pin;
    uint16_t ret = 65535;
    do {
        SEESAW_IO_Delay(1);
        Adafruit_seesaw_read(SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET + p, buf, 2,
            1000);
        ret = ((uint16_t)buf[0] << 8) | buf[1];
    } while (ret == 65535);
    return ret;
}

/*!
 ***************************************************************************
 *  @brief      set the mode of multiple GPIO pins at once.
 *
 *  @param      pins a bitmask of the pins to write. On the SAMD09 breakout,
 *this corresponds to the number on the silkscreen. For example, passing 0b0110
 *will set the mode of pins 2 and 3.
 *	@param		mode the mode to set the pins to. One of INPUT, OUTPUT,
 *or INPUT_PULLUP.
 ************************************************************************/
void Adafruit_seesaw_pinModeBulk(uint32_t pins, uint8_t mode) {
    uint8_t cmd[] = { (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                     (uint8_t)(pins >> 8), (uint8_t)pins };
    switch (mode) {
    case OUTPUT:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, cmd, 4);
        break;
    case INPUT:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
        break;
    case INPUT_PULLUP:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 4);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, 4);
        break;
    case INPUT_PULLDOWN:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 4);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, 4);
        break;
    }
}

/*!
 *****************************************************************************************
 *  @brief      set the mode of multiple GPIO pins at once. This supports both
 *ports A and B.
 *
 *  @param      pinsa a bitmask of the pins to write on port A. On the SAMD09
 *breakout, this corresponds to the number on the silkscreen. For example,
 *passing 0b0110 will set the mode of pins 2 and 3.
 *  @param      pinsb a bitmask of the pins to write on port B.
 *	@param		mode the mode to set the pins to. One of INPUT, OUTPUT,
 *or INPUT_PULLUP.
 ****************************************************************************************/
void Adafruit_seesaw_pinModeBulkB(uint32_t pinsa, uint32_t pinsb,
    uint8_t mode) {
    uint8_t cmd[] = { (uint8_t)(pinsa >> 24), (uint8_t)(pinsa >> 16),
                     (uint8_t)(pinsa >> 8),  (uint8_t)pinsa,
                     (uint8_t)(pinsb >> 24), (uint8_t)(pinsb >> 16),
                     (uint8_t)(pinsb >> 8),  (uint8_t)pinsb };
    switch (mode) {
    case OUTPUT:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, cmd, 8);
        break;
    case INPUT:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 8);
        break;
    case INPUT_PULLUP:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 8);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 8);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, 8);
        break;
    case INPUT_PULLDOWN:
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 8);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 8);
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, 8);
        break;
    }
}

/*!
 *****************************************************************************************
 *  @brief      write a value to multiple GPIO pins at once.
 *
 *  @param      pins a bitmask of the pins to write. On the SAMD09 breakout,
 *this corresponds to the number on the silkscreen. For example, passing 0b0110
 *will write the passed value to pins 2 and 3.
 *	@param		value pass HIGH to set the output on the passed pins to
 *HIGH, low to set the output on the passed pins to LOW.
 ****************************************************************************************/
void Adafruit_seesaw_digitalWriteBulk(uint32_t pins, uint8_t value) {
    uint8_t cmd[] = { (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                     (uint8_t)(pins >> 8), (uint8_t)pins };
    if (value)
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, 4);
    else
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, 4);
}

/*!
 *****************************************************************************************
 *  @brief      write a value to multiple GPIO pins at once. This supports both
 *ports A and B
 *
 *  @param      pinsa a bitmask of the pins to write on port A. On the SAMD09
 *breakout, this corresponds to the number on the silkscreen. For example,
 *passing 0b0110 will write the passed value to pins 2 and 3.
 *  @param      pinsb a bitmask of the pins to write on port B.
 *	@param		value pass HIGH to set the output on the passed pins to
 *HIGH, low to set the output on the passed pins to LOW.
 ****************************************************************************************/
void Adafruit_seesaw_digitalWriteBulkB(uint32_t pinsa, uint32_t pinsb,
    uint8_t value) {
    uint8_t cmd[] = { (uint8_t)(pinsa >> 24), (uint8_t)(pinsa >> 16),
                     (uint8_t)(pinsa >> 8),  (uint8_t)pinsa,
                     (uint8_t)(pinsb >> 24), (uint8_t)(pinsb >> 16),
                     (uint8_t)(pinsb >> 8),  (uint8_t)pinsb };
    if (value)
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, 8);
    else
        Adafruit_seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, 8);
}

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*!
 *****************************************************************************************
 *  @brief      write a PWM value to a PWM-enabled pin
 *
 *  @param      pin the number of the pin to write. On the SAMD09 breakout, this
 *corresponds to the number on the silkscreen. on the default seesaw firmware on
 *the SAMD09 breakout, pins 5, 6, and 7 are PWM enabled.
 *	@param		value the value to write to the pin
 *	@param		width the width of the value to write. Defaults to 8. If
 *16 is passed a 16 bit value will be written.
 ****************************************************************************************/
void Adafruit_seesaw_analogWrite(uint8_t pin, uint16_t value, uint8_t width) {
    int8_t p = -1;
    switch (pin) {
    case PWM_0_PIN:
        p = 0;
        break;
    case PWM_1_PIN:
        p = 1;
        break;
    case PWM_2_PIN:
        p = 2;
        break;
    case PWM_3_PIN:
        p = 3;
        break;
    default:
        break;
    }
    if (p > -1) {
        if (width == 16) {
            uint8_t cmd[] = { (uint8_t)p, (uint8_t)(value >> 8), (uint8_t)value };
            Adafruit_seesaw_write(SEESAW_TIMER_BASE, SEESAW_TIMER_PWM, cmd, 3);
        }
        else {
            uint16_t mappedVal = map(value, 0, 255, 0, 65535);
            uint8_t cmd[] = { (uint8_t)p, (uint8_t)(mappedVal >> 8),
                             (uint8_t)mappedVal };
            Adafruit_seesaw_write(SEESAW_TIMER_BASE, SEESAW_TIMER_PWM, cmd, 3);
        }
    }
}

/*!
 *  @brief      set the PWM frequency of a PWM-enabled pin. Note that on SAMD09,
 *              SAMD11 boards the frequency will be mapped to closest match
 *		fixed frequencies. Also note that PWM pins 4 and 5 share a
 *timer, and PWM pins 6 and 7 share a timer. Changing the frequency for one pin
 *will change the frequency for the other pin that is on the timer.
 *
 *  @param      pin the number of the pin to change frequency of. On the SAMD09
 *              breakout, this corresponds to the number on the silkscreen.
 *              on the default seesaw firmware on the SAMD09 breakout, pins 5,
 *6, and 7 are PWM enabled.
 *  @param      freq the frequency to set.
 ******************************************************************************/
void Adafruit_seesaw_setPWMFreq(uint8_t pin, uint16_t freq) {
    int8_t p = -1;
    switch (pin) {
    case PWM_0_PIN:
        p = 0;
        break;
    case PWM_1_PIN:
        p = 1;
        break;
    case PWM_2_PIN:
        p = 2;
        break;
    case PWM_3_PIN:
        p = 3;
        break;
    default:
        break;
    }
    if (p > -1) {
        uint8_t cmd[] = { (uint8_t)p, (uint8_t)(freq >> 8), (uint8_t)freq };
        Adafruit_seesaw_write(SEESAW_TIMER_BASE, SEESAW_TIMER_FREQ, cmd, 3);
    }
}

/*!
 *  @brief      Enable the data ready interrupt on the passed sercom. Note that
 *both the interrupt module and the passed sercom must be compiled into the
 *seesaw firmware for this to function. If both of these things are true, the
 *interrupt pin on the seesaw will fire when there is data to be read from the
 *passed sercom. On the default seesaw firmeare on the SAMD09 breakout, no
 *sercoms are enabled.
 *
 *  @param      sercom the sercom to enable the interrupt on.
 ****************************************************************************************/
/*void Adafruit_seesaw_enableSercomDataRdyInterrupt(uint8_t sercom) {
    _sercom_inten.bit.DATA_RDY = 1;
    Adafruit_seesaw_write8(SEESAW_SERCOM0_BASE + sercom, SEESAW_SERCOM_INTEN,
        _sercom_inten.reg);
}*/

/*!
 ***************************************************************************************
 *  @brief      Disable the data ready interrupt on the passed sercom.
 *
 *  @param      sercom the sercom to disable the interrupt on.
 ****************************************************************************************/
/*void Adafruit_seesaw_disableSercomDataRdyInterrupt(uint8_t sercom) {
    _sercom_inten.bit.DATA_RDY = 0;
    Adafruit_seesaw_write8(SEESAW_SERCOM0_BASE + sercom, SEESAW_SERCOM_INTEN,
        _sercom_inten.reg);
}*/

/*!
 *****************************************************************************************
 *  @brief      Reads a character from the passed sercom if one is available.
 *Note that on the default seesaw firmware on the SAMD09 breakout no sercoms are
 *enabled.
 *
 *  @param      sercom the sercom to read data from.
 *  @returns    One byte of data
 ****************************************************************************************/
char Adafruit_seesaw_readSercomData(uint8_t sercom) {
    return Adafruit_seesaw_read8(SEESAW_SERCOM0_BASE + sercom, SEESAW_SERCOM_DATA, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
}

/*!
 *****************************************************************************************
 *  @brief      Set the seesaw I2C address. This will automatically call
 *Adafruit_seesaw.begin() with the new address.
 *
 *  @param      addr the new address for the seesaw. This must be a valid 7 bit
 *I2C address.
 ****************************************************************************************/
void Adafruit_seesaw_setI2CAddr(uint8_t addr) {
    Adafruit_seesaw_EEPROMWrite8(SEESAW_EEPROM_I2C_ADDR, addr);
    SEESAW_IO_Delay(250);
    Adafruit_seesaw_init(addr, true); // restart w/ the new addr
}

/*!
 *****************************************************************************************
 *  @brief      Read the I2C address of the seesaw
 *
 *  @return     the 7 bit I2C address of the seesaw... which you probably
 *already know because you just read data from it.
 ****************************************************************************************/
uint8_t Adafruit_seesaw_getI2CAddr(void) {
    return Adafruit_seesaw_read8(SEESAW_EEPROM_BASE, SEESAW_EEPROM_I2C_ADDR, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
}

/*!
 *****************************************************************************************
 *  @brief      Write a 1 byte to an EEPROM address
 *
 *  @param      addr the address to write to. On the default seesaw firmware on
 *the SAMD09 breakout this is between 0 and 63.
 *	@param		val to write between 0 and 255
 ****************************************************************************************/
void Adafruit_seesaw_EEPROMWrite8(uint8_t addr, uint8_t val) {
    Adafruit_seesaw_EEPROMWrite(addr, &val, 1);
}

/*!
 *****************************************************************************************
 *  @brief      write a string of bytes to EEPROM starting at the passed address
 *
 *  @param      addr the starting address to write the first byte. This will be
 *automatically incremented with each byte written.
 *	@param		buf the buffer of bytes to be written.
 *	@param		size the number of bytes to write. Writing past the end
 *of available EEPROM may result in undefined behavior.
 ****************************************************************************************/
void Adafruit_seesaw_EEPROMWrite(uint8_t addr, uint8_t* buf, uint8_t size) {
    Adafruit_seesaw_write(SEESAW_EEPROM_BASE, addr, buf, size);
}

/*!
 *****************************************************************************************
 *  @brief      Read 1 byte from the specified EEPROM address.
 *
 *  @param      addr the address to read from. One the default seesaw firmware
 *on the SAMD09 breakout this is between 0 and 63.
 *
 *  @return     the value between 0 and 255 that was read from the passed
 *address.
 ****************************************************************************************/
uint8_t Adafruit_seesaw_EEPROMRead8(uint8_t addr) {
    return Adafruit_seesaw_read8(SEESAW_EEPROM_BASE, addr, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
}

/*!
 *****************************************************************************************
 *  @brief      Set the baud rate on SERCOM0.
 *
 *  @param      baud the baud rate to set. This is an integer value. Baud rates
 *up to 115200 are supported.
 ****************************************************************************************/
void Adafruit_seesaw_UARTSetBaud(uint32_t baud) {
    uint8_t cmd[] = { (uint8_t)(baud >> 24), (uint8_t)(baud >> 16),
                     (uint8_t)(baud >> 8), (uint8_t)baud };
    Adafruit_seesaw_write(SEESAW_SERCOM0_BASE, SEESAW_SERCOM_BAUD, cmd, 4);
}

/*!
 *****************************************************************************************
 *  @brief      activate or deactivate a key and edge on the keypad module
 *
 *  @param      key the key number to activate
 *  @param		edge the edge to trigger on
 *  @param		enable passing true will enable the passed event,
 *passing false will disable it.
 ****************************************************************************************/
void Adafruit_seesaw_setKeypadEvent(uint8_t key, uint8_t edge, bool enable) {
    union keyState ks;
    ks.bit.STATE = enable;
    ks.bit.ACTIVE = (1 << edge);
    uint8_t cmd[] = { key, ks.reg };
    Adafruit_seesaw_write(SEESAW_KEYPAD_BASE, SEESAW_KEYPAD_EVENT, cmd, 2);
}

/**
 *****************************************************************************************
 *  @brief      enable the keypad interrupt that fires when events are in the
 *fifo.
 ****************************************************************************************/
void Adafruit_seesaw_enableKeypadInterrupt(void) {
    Adafruit_seesaw_write8(SEESAW_KEYPAD_BASE, SEESAW_KEYPAD_INTENSET, 0x01);
}

/**
 *****************************************************************************************
 *  @brief      disable the keypad interrupt that fires when events are in the
 *fifo.
 ****************************************************************************************/
void Adafruit_seesaw_disableKeypadInterrupt(void) {
    Adafruit_seesaw_write8(SEESAW_KEYPAD_BASE, SEESAW_KEYPAD_INTENCLR, 0x01);
}

/**
 *****************************************************************************************
 *  @brief      Get the number of events currently in the fifo
 *  @return     the number of events in the fifo
 ****************************************************************************************/
uint8_t Adafruit_seesaw_getKeypadCount(void) {
    return Adafruit_seesaw_read8(SEESAW_KEYPAD_BASE, SEESAW_KEYPAD_COUNT, 500);
}

/**
 *****************************************************************************************
 *  @brief      Read all keyEvents into the passed buffer
 *
 *  @param      buf pointer to where the keyEvents should be stored
 *  @param		count the number of events to read
 ****************************************************************************************/
void Adafruit_seesaw_readKeypad(union keyEventRaw* buf, uint8_t count) {
    Adafruit_seesaw_read(SEESAW_KEYPAD_BASE, SEESAW_KEYPAD_FIFO, (uint8_t*)buf,
        count, 1000);
}

/**
 *****************************************************************************************
 *  @brief      Read the temperature of the seesaw board in degrees Celsius.
 *NOTE: not all seesaw firmwares have the temperature sensor enabled.
 *  @return     Temperature in degrees Celsius as a floating point value.
 ****************************************************************************************/
float Adafruit_seesaw_getTemp(void) {
    uint8_t buf[4];
    Adafruit_seesaw_read(SEESAW_STATUS_BASE, SEESAW_STATUS_TEMP, buf, 4, 1000);
    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
        ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return (1.0 / (1UL << 16)) * ret;
}

/**
 *****************************************************************************************
 *  @brief      Read the current position of the encoder
 *  @return     The encoder position as a 32 bit signed integer.
 ****************************************************************************************/
int32_t Adafruit_seesaw_getEncoderPosition(void) {
    uint8_t buf[4];
    Adafruit_seesaw_read(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, buf, 4, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
        ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

    return ret;
}

/**
 *****************************************************************************************
 *  @brief      Set the current position of the encoder
 *  @param     pos the position to set the encoder to.
 ****************************************************************************************/
void Adafruit_seesaw_setEncoderPosition(int32_t pos) {
    uint8_t buf[] = { (uint8_t)(pos >> 24), (uint8_t)(pos >> 16),
                     (uint8_t)(pos >> 8), (uint8_t)(pos & 0xFF) };
    Adafruit_seesaw_write(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, buf, 4);
}

/**
 *****************************************************************************************
 *  @brief      Read the change in encoder position since it was last read.
 *  @return     The encoder change as a 32 bit signed integer.
 ****************************************************************************************/
int32_t Adafruit_seesaw_getEncoderDelta(void) {
    uint8_t buf[4];
    Adafruit_seesaw_read(SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA, buf, 4, ADAFRUIT_SEESAW_DELAY_DEFAULT_US);
    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
        ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

    return ret;
}

/**
 *****************************************************************************************
 *  @brief      Enable the interrupt to fire when the encoder changes position.
 ****************************************************************************************/
void Adafruit_seesaw_enableEncoderInterrupt(void) {
    Adafruit_seesaw_write8(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET, 0x01);
}

/**
 *****************************************************************************************
 *  @brief      Disable the interrupt from firing when the encoder changes
 *position.
 ****************************************************************************************/
void Adafruit_seesaw_disableEncoderInterrupt(void) {
    Adafruit_seesaw_write8(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENCLR, 0x01);
}

/**
 *****************************************************************************************
 *  @brief      Write 1 byte to the specified seesaw register.
 *
 *  @param      regHigh the module address register (ex. SEESAW_NEOPIXEL_BASE)
 *	@param		regLow the function address register (ex.
 *SEESAW_NEOPIXEL_PIN)
 *	@param		value the value between 0 and 255 to write
 ****************************************************************************************/
void Adafruit_seesaw_write8(uint8_t regHigh, uint8_t regLow, uint8_t value) {
    Adafruit_seesaw_write(regHigh, regLow, &value, 1);
}

/**
 *****************************************************************************************
 *  @brief      read 1 byte from the specified seesaw register.
 *
 *  @param      regHigh the module address register (ex. SEESAW_STATUS_BASE)
 *	@param		regLow the function address register (ex.
 *SEESAW_STATUS_VERSION)
 *	@param		delay a number of microseconds to delay before reading
 *out the data. Different delay values may be necessary to ensure the seesaw
 *chip has time to process the requested data. Defaults to 125.
 *
 *  @return     the value between 0 and 255 read from the passed register
 ****************************************************************************************/
uint8_t Adafruit_seesaw_read8(uint8_t regHigh, uint8_t regLow, uint16_t delay_us) {
    uint8_t ret = 0;
    Adafruit_seesaw_read(regHigh, regLow, &ret, 1, delay_us);
    return ret;
}


/**
 *****************************************************************************************
 *  @brief      Read a specified number of bytes into a buffer from the seesaw.
 *
 *  @param      regHigh the module address register (ex. SEESAW_STATUS_BASE)
 *	@param		regLow the function address register (ex.
 *SEESAW_STATUS_VERSION)
 *	@param		buf the buffer to read the bytes into
 *	@param		num the number of bytes to read.
 *	@param		delay an optional delay in between setting the read
 *register and reading out the data. This is required for some seesaw functions
 *(ex. reading ADC data)
 ****************************************************************************************/
void Adafruit_seesaw_read(uint8_t regHigh, uint8_t regLow, uint8_t* buf,
    uint8_t num, uint16_t delay_us) 
{
#if 1
    uint16_t reg = 
        ((uint16_t)regHigh << 8) |
        ((uint16_t)regLow  << 0);

    SEESAW_IO_ReadMultiple(_i2caddr, reg, buf, num, delay_us);
#endif

#if 0
    HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t * pData, uint16_t Size,
        uint32_t Timeout);
    SEESAW_IO_DelayMicroseconds(delay_us);
    HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t * pData, uint16_t Size,
        uint32_t Timeout);
#endif


#if 0
    uint8_t pos = 0;

    // on arduino we need to read in 32 byte chunks
    while (pos < num) {
        uint8_t read_now = min(32, num - pos);
        _i2cbus->beginTransmission((uint8_t)_i2caddr);
        _i2cbus->write((uint8_t)regHigh);
        _i2cbus->write((uint8_t)regLow);
#ifdef SEESAW_I2C_DEBUG
        Serial.print("I2C read $");
        Serial.print((uint16_t)regHigh << 8 | regLow, HEX);
        Serial.print(" : ");
#endif

        if (_flow != -1)
            while (!::digitalRead(_flow))
                ;
        _i2cbus->endTransmission();

        SEESAW_IO_DelayMicroseconds(delay_us);

        if (_flow != -1)
            while (!::digitalRead(_flow))
                ;
        _i2cbus->requestFrom((uint8_t)_i2caddr, read_now);

        for (int i = 0; i < read_now; i++) {
            buf[pos] = _i2cbus->read();
#ifdef SEESAW_I2C_DEBUG
            Serial.print("0x");
            Serial.print(buf[pos], HEX);
            Serial.print(",");
#endif
            pos++;
        }
#ifdef SEESAW_I2C_DEBUG
        Serial.println();
#endif
    }

#endif
}

/*!
 *****************************************************************************************
 *  @brief      Write a specified number of bytes to the seesaw from the passed
 *buffer.
 *
 *  @param      regHigh the module address register (ex. SEESAW_GPIO_BASE)
 *  @param	regLow the function address register (ex. SEESAW_GPIO_BULK_SET)
 *  @param	buf the buffer the the bytes from
 *  @param	num the number of bytes to write.
 ****************************************************************************************/
void Adafruit_seesaw_write(uint8_t regHigh, uint8_t regLow, uint8_t* buf,
    uint8_t num) 
{


    uint16_t reg =
        ((uint16_t)regHigh << 8) |
        ((uint16_t)regLow << 0);

    SEESAW_IO_WriteMultiple(_i2caddr, reg, buf, num);

}

