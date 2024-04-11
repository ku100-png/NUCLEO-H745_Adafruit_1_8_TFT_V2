# NUCLEO-H745_Adafruit_1_8_TFT_V2
 
Add support for the newer Adafruit 1.8" TFT Shield V2 on the NUCLEO-H745-ZI-Q. The examples previously only supported the older V1 board which is no longer available. The V2 board adds an I2C expander to access the joystick, buttons and LCD backlight and reset IO. The expander is called the seesaw and the driver has been ported into a separate component under Drivers\BSP\Components. <br>
<br>

This shield is a useful addition to the Nucleo ecosystem for building prototypes where user interaction would be helpful. <br>
![NUCLEO-H745](https://github.com/ku100-png/NUCLEO-H745_Adafruit_1_8_TFT_V2/blob/main/IMG_NUCLEO-H745.jpg)

<br>
Setup <br>
<br>

For connect use 6-pin dual-row header on Adafruit 1.8" TFT Shield V2. It requires 3 wires to be run
to the appropriate digital I/O pins to route SPI from the Nucleo to the TFT-display and to the SD card. Solder them to the
free holes marked D11,D12,D13 at the top and connect then to 6-pin header to pin4, pin1 and pin3, looking up at the image.
Lastly, attach the shield to a NUCLEO-H745-ZI-Q.
<br>

A FAT(32) formatted SD-card hosting 128w x 160h RGB565 BMP images is required. These images can be copied from \Media folder. 
It is more difficult to create your own RBG565 image data. Use Photoshop program or online converter to bmp files of the correct 16-bit RGB565 format (https://online-converting.com/image/convert2bmp/). 
The image file names must be 11-character maximum and sit in the root directory. Max count of .bmp files on SD card is 25.
<br>

Project
<br>
This project base on the CubeMX "Demonstration" example that locate in STM32Cube\Repository\STM32Cube_FW_H7_V1.9.1\Projects\NUCLEO-H745ZI-Q\Demonstrations\MDK-ARM <br>
A Keil MDK-ARM IDE project has been created for NUCLEO-H745ZI-Q\Demonstrations. Import this project into your workspace, build and debug it. The display will show a menu requiring use of the joystick to select the picture mode.
In the project add support buttons A,B,C on the Adafruit 1.8" TFT Shield V2. In manual mode push button A turn on LED1 on the NUCLEO boad, push button B turn off LED1, push button C toggle LED3.
<br>
Integrating into another project
Add USE_ADAFRUIT_SHIELD_V2 to the IDE preprocessor defines.
Add BSP/Components/adafruit_seesaw/adafruit_seesaw.c to the project.
Ensure HAL_I2C_MODULE_ENABLED is set in stm32l0xx_hal_conf.h and that stm32l0xx_hal_i2c.c is part of the build.
Add two files: fatfs_sd.c and fatfs_sd.h from https://github.com/eziya for work with SD card.
<br>

Usefull links
<br>
- http://elm-chan.org/docs/mmc/mmc_e.html
- https://github.com/eziya
- https://github.com/firmwaremodules/STM32CubeL0/issues/1
- https://online-converting.com/image/convert2bmp/
- https://cdn-learn.adafruit.com/downloads/pdf/1-8-tft-display.pdf
<br>
