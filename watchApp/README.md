CustomOSDWatch watchApp
=======================

Watch Application to run on the OpenSeizureDetector custom watch.

It is built using the [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Hardware
========

Proof-of-concept hardware utilises:
  * ESP WROOM 32 module (to give usb interface for flashing esp32 chip and debuging
  * ADXL345 module (accelerometer)
  * MAX30102 module (heart rate and oxygen saturation)
  * I2C display module

All three modules are connected to the same i2c bus connected to GPIO_18 and GPIO_19.  We are utilising the internal pull-up resisors in the ESP32 module, so no other external hardware is used.

The intention is to design a custom PCB to take an ESP32 chip along with the ADXL345 and MAX30102 ICs and ancilary hardware to make it fit into a watch case.

Software
========
Compile with
- cd watchApp
- make
Flash onto the ESP32 chip with
- make flash
View debugging information on the console over USB using
- make monitor
(exit the monitor with CTRL-])

Status
======
- Modified my adxl345 library to use an intermediary higher level ESP32 i2c
library (https://github.com/natanaeljr/esp32-I2Cbus) - we can now detect the adxl345 module.
- we can also see the other two i2c modules on the bus.

