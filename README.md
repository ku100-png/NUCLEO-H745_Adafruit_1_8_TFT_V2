# NUCLEO-H745_Adafruit_1_8_TFT_V2
 
Add support for the newer Adafruit 1.8" TFT Shield V2 on the NUCLEO-H745-ZI-Q. The examples previously only supported the older V1 board which is no longer available. The V2 board adds an I2C expander to access the joystick, buttons and LCD backlight and reset IO. The expander is called the seesaw and the driver has been ported into a separate component under Drivers\BSP\Components. <br>
<br>
This project base on the CubeMX "Demonstration" example that locate in STM32Cube\Repository\STM32Cube_FW_H7_V1.9.1\Projects\NUCLEO-H745ZI-Q\Demonstrations\MDK-ARM <br>
<br>
This shield is a useful addition to the Nucleo ecosystem for building prototypes where user interaction would be helpful. <br>
![NUCLEO-H745](https://github.com/ku100-png/NUCLEO-H745_Adafruit_1_8_TFT_V2/blob/main/IMG_NUCLEO-H745.jpg)

<br>
For connect use 6-pin dual-row header on Adafruit 1.8" TFT Shield V2. It requires 3 wires to be run
to the appropriate digital I/O pins to route SPI from the Nucleo to the TFT-display and to the SD card. Solder them to the
free holes marked D11,D12,D13 at the top and connect then to 6-pin header to pin4, pin1 and pin3, looking up at the image.
