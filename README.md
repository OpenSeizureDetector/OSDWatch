OpenSeizureDetector : OSDWatch
==============================

OSDWatch is (or rather will be) a custom wrist watch with sensors for seizure
detection, and the associated software to use it as a seizure alarm system.

It is in a very early state of development!

Hardware
========
Design constraints:
  * Provide a battery life of at least 12 hours (preferably 24 hours) per charge.
  * Monitor acceleration (to allow the equivalent system to the Pebble seizure detector)
  * Communicate with a mobile device (phone or tablet) to provide an alarm system (so we can use the existing OpenSeizureDetector android app framework).
  * Monitor additional paramaters as space and battery life allows, such as heart rate, blood oxygen saturation, skin temperature, skin conductivity.


Likely Components:
  * ESP32 microcontroller module with integral bluetooth radio
  * ADXL345 accelerometer
  * Max30102 heart rate and blood oxygen saturation sensor
  * OLED dislay
  * LiPo battery charge controller.


Software
========
  * FreeRTOS based system (for portability if we change microcontroller)
  * Tasks to carry out the following functions:
    * I2C hardware interface (so different tasks do not try to access the I2C bus at the same time)
    * Communications Interface (To handle communication with the parent android device)
    * Acceleration Handler (set up the acceleration sensor, and maintain a buffer of acceleration data, by responding to interrupts from the sensor)
    * Display Handler (update the display periodically)
    * Handlers for other sensors
    * Analysis Task - periodically analyse the sensor data to perform seizure detection.
    * Logging Task - periodically log data to the parent android device or local storage for future analysis.


