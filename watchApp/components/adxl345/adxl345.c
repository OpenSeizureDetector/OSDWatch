/*
 *  ADXL345.c - ADXL345 library for freeRTOS on ESP8266 and ESP32.  
 * Based on the Arduino ADXL345 library at 
 *    https://github.com/jarzebski/Arduino-ADXL345
 * (c) 2014 Korneliusz Jarzebski (www.jarzebski.pl)
 * Copyright Graham Jones, 2017, 2018.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include "adxl345.h"

// Module variables - global within this module.
i2c_port_t ADXL345_devAddr = (i2c_port_t)0xff;  // 7 bit i2c address range so this is off scale to
                         //    show  it is not initialised yet.
ADXL345_Vector f;


uint8_t ADXL345_init(int scl, int sda) {
  printf("ADXL345_init(%d,%d)\n",scl,sda);
#if defined(ESP_PLATFORM)
  ADXL345_devAddr = (i2c_port_t)0;
  ADXL345_devAddr = (i2c_port_t)ADXL345_findDevice();
#elif defined (ESP_SDK)
  i2c_init(scl,sda);
  ADXL345_devAddr = 0;
  ADXL345_devAddr = ADXL345_findDevice();
#else
  i2c_master_gpio_init();
  i2c_master_init();
  ADXL345_devAddr = 0;
  ADXL345_devAddr = ADXL345_findDevice();
#endif
  if (ADXL345_devAddr==0) {
    printf("***ERROR - ADXL345 not found ***\n");
  } else {
    printf("ADXL345 found at address 0x%x\n",
	   ADXL345_devAddr);
  }
  f.XAxis = 0;
  f.YAxis = 0;
  f.ZAxis = 0;

  printf("ADXL345_init() complete\n");
  return(ADXL345_devAddr);
}




/*****************************************
 * Basic register input/output functions *
 *****************************************/
// Write byte to register
uint8_t ADXL345_writeRegister8(uint8_t reg, uint8_t value)
{
  i2cTransaction_t trans;
  trans.type = I2C_TX;
  trans.devAddr=ADXL345_devAddr;
  trans.reg = reg;
  trans.nData = 1;
  trans.data[0]=value;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  if (trans.retVal) {
    // non-zero response = error
    printf("ADXL345_writeRegister8(0x%x, 0x%x) - Error\n",ADXL345_devAddr,reg);
    return trans.retVal;
  } else {
    return 0;
  }

}


/* Reads a single byte from the ADXL345 register reg
 * @return: the byte read, or -1 on error.
 */
uint8_t ADXL345_readRegister8(uint8_t reg) {
  i2cTransaction_t trans;
  uint8_t byte;
#if defined(ESP_PLATFORM)
  trans.type = I2C_RX;
  trans.devAddr=ADXL345_devAddr;
  trans.reg = reg;
  trans.nData = 1;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  byte = trans.data[0];
  if (trans.retVal) {
#else
  if (i2c_slave_read(ADXL345_devAddr, &reg, &byte, 1)) {
#endif
    // non-zero response = error
    //printf("ADXL345_readRegister8(0x%x, 0x%x) - Error\n",ADXL345_devAddr,reg);
    return -1;
  } else {
    return byte;
  }
}


/* Reads a 2 byte word from the ADXL345 at address devAddr, register regAddr
 * @return: the word read, or -1 on error.
 */
int16_t ADXL345_readRegister16(uint8_t reg) {
  int16_t value;
  uint8_t data[2];
  /* Note - this fiddle is something to do with C99 or C11 because
        saying uint8_t bytes[2]; bytes = &(trans.data[0]) failed with an error
        assignment to expression with array type
  */
  uint8_t *bytes;
  bytes = &data[0];
#if defined(ESP_PLATFORM)
  i2cTransaction_t trans;
  trans.type = I2C_RX;
  trans.devAddr=ADXL345_devAddr;
  trans.reg = reg;
  trans.nData = 2;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  bytes = &(trans.data[0]);
  if (trans.retVal) {
#else
  if (i2c_slave_read(ADXL345_devAddr, &reg, &bytes[0], 2)) {
#endif
    // non-zero response = error
    //printf("ADXL345_readRegister8(0x%x, 0x%x) - Error\n",devAddr,reg);
    return -1;
  } else {
    value = bytes[1]<<8 | bytes[0];
    return value;
  }
}

// Read raw acceleration values
// read all 12 bytes at once so it is compatible with
// FIFO (otherwise finishing reading one axis value would pop the FIFO
ADXL345_IVector ADXL345_readRaw(void)
{
  ADXL345_IVector r;
  uint8_t data[6];
  /* Note - this fiddle is something to do with C99 or C11 because
        saying uint8_t bytes[2]; bytes = &(trans.data[0]) failed with an error
        assignment to expression with array type
  */
  uint8_t *bytes;
  bytes = &data[0];
  uint8_t reg;

  reg = ADXL345_REG_DATAX0;

  // read all 6 data bytes in one read operation.
  i2cTransaction_t trans;
  trans.type = I2C_RX;
  trans.devAddr=ADXL345_devAddr;
  trans.reg = reg;
  trans.nData = 6;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  bytes = trans.data;
  if (trans.retVal) {
    // non-zero response = error
    printf("ADXL345_readRaw - Error\n");
    r.XAxis = 0;
    r.YAxis = 0;
    r.ZAxis = 0;
  } else {
    r.XAxis = (int16_t)(4*(bytes[1]<<8 | bytes[0]));
    r.YAxis = (int16_t)(4*(bytes[3]<<8 | bytes[2]));
    r.ZAxis = (int16_t)(4*(bytes[5]<<8 | bytes[4]));
  }
  return r;
}

  /****************************************************
   * END OF PLATFORM SPECIFIC CODE                    *
   ****************************************************/

  
void ADXL345_writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = ADXL345_readRegister8(reg);

    if (state)
    {
	value |= (1 << pos);
    } else 
    {
	value &= ~(1 << pos);
    }

    ADXL345_writeRegister8(reg, value);
}

bool ADXL345_readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = ADXL345_readRegister8(reg);
    return ((value >> pos) & 1);
}




/**
 * Scan the i2c bus for the first ADXL345 chip on the bus.
 * @return the address of the first ADXL345 chip on the bus.
 * on Exit ADXL345_devAddr global variable is set to id of device.
 */
uint8_t ADXL345_findDevice() {
  uint8_t id;
  uint8_t i;
  for (i=0;i<127;i++) {
    ADXL345_devAddr = (i2c_port_t)i;
    id = ADXL345_readRegister8(ADXL345_REG_DEVID);
    if (id!=0xff) {
      printf("Response received from address %d (0x%x) - id=%d (0x%x)\n",i,i,id,id);
      if ((i==ADXL345_ADDRESS) || (i==ADXL345_ALT_ADDRESS)) {
	printf("Address correct for ADXL345 chip.\n");
	if (id==ADXL345_DEVID) {
	  printf("Device ID correct for ADXL345\n");
	  return(i);
	} else {
	  printf("Device ID incorrect - not an ADXL345 chip\n");
	}
      } else {
	printf("Address incorrect - not an ADXL345 chip\n");
      }
    }
  }
  return(-1);
}





// Set Range
void ADXL345_setRange(ADXL345_range_t range)
{
  // Get actual value register
  uint8_t value = ADXL345_readRegister8(ADXL345_REG_DATA_FORMAT);

  // Update the data rate
  // (&) 0b11110000 (0xF0 - Leave HSB)
  // (|) 0b0000xx?? (range - Set range)
  // (|) 0b00001000 (0x08 - Set Full Res)
  value &= 0xF0;
  value |= range;
  value |= 0x08;

  ADXL345_writeRegister8(ADXL345_REG_DATA_FORMAT, value);
}

// Get Range
ADXL345_range_t ADXL345_getRange(void)
{
    return (ADXL345_range_t)(ADXL345_readRegister8(ADXL345_REG_DATA_FORMAT) & 0x03);
}

// Set Data Rate
void ADXL345_setDataRate(ADXL345_dataRate_t dataRate)
{
    ADXL345_writeRegister8(ADXL345_REG_BW_RATE, dataRate);
}

// Get Data Rate
ADXL345_dataRate_t ADXL345_getDataRate(void)
{
    return (ADXL345_dataRate_t)(ADXL345_readRegister8(ADXL345_REG_BW_RATE) & 0x0F);
}

// Low Pass Filter
ADXL345_Vector ADXL345_lowPassFilter(ADXL345_Vector vector, float alpha)
{
    f.XAxis = vector.XAxis * alpha + (f.XAxis * (1.0 - alpha));
    f.YAxis = vector.YAxis * alpha + (f.YAxis * (1.0 - alpha));
    f.ZAxis = vector.ZAxis * alpha + (f.ZAxis * (1.0 - alpha));
    return f;
}


// Read normalized values
ADXL345_Vector ADXL345_readNormalize(float gravityFactor)
{
  ADXL345_IVector r;
  ADXL345_Vector n;
  r = ADXL345_readRaw();

  // (4 mg/LSB scale factor in Full Res) * gravity factor
  n.XAxis = r.XAxis * 0.004 * gravityFactor;
  n.YAxis = r.YAxis * 0.004 * gravityFactor;
  n.ZAxis = r.ZAxis * 0.004 * gravityFactor;
  
  return n;
}

// Read scaled values
ADXL345_Vector ADXL345_readScaled(void)
{
  ADXL345_IVector r;
  ADXL345_Vector n;
  r = ADXL345_readRaw();

  // (4 mg/LSB scale factor in Full Res)
  n.XAxis = r.XAxis * 0.004;
  n.YAxis = r.YAxis * 0.004;
  n.ZAxis = r.ZAxis * 0.004;
  
  return n;
}

void ADXL345_clearSettings(void)
{
    ADXL345_setRange(ADXL345_RANGE_2G);
    ADXL345_setDataRate(ADXL345_DATARATE_100HZ);

    ADXL345_writeRegister8(ADXL345_REG_THRESH_TAP, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_DUR, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_LATENT, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_WINDOW, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_THRESH_ACT, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_THRESH_INACT, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_TIME_INACT, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_THRESH_FF, 0x00);
    ADXL345_writeRegister8(ADXL345_REG_TIME_FF, 0x00);

    uint8_t value;

    value = ADXL345_readRegister8(ADXL345_REG_ACT_INACT_CTL);
    value &= 0b10001000;
    ADXL345_writeRegister8(ADXL345_REG_ACT_INACT_CTL, value);

    value = ADXL345_readRegister8(ADXL345_REG_TAP_AXES);
    value &= 0b11111000;
    ADXL345_writeRegister8(ADXL345_REG_TAP_AXES, value);
}

// Set Tap Threshold (62.5mg / LSB)
void ADXL345_setTapThreshold(float threshold)
{
  // FIXME - add constrain() function to avoid over-runs.
  //uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
  uint8_t scaled = threshold / 0.0625f;
  ADXL345_writeRegister8(ADXL345_REG_THRESH_TAP, scaled);
}

// Get Tap Threshold (62.5mg / LSB)
float ADXL345_getTapThreshold(void)
{
    return ADXL345_readRegister8(ADXL345_REG_THRESH_TAP) * 0.0625f;
}

// Set Tap Duration (625us / LSB)
void ADXL345_setTapDuration(float duration)
{
  // FIXME - add constrain() function to avoid over-runs.
  //  uint8_t scaled = constrain(duration / 0.000625f, 0, 255);
    uint8_t scaled = duration / 0.000625f;
    ADXL345_writeRegister8(ADXL345_REG_DUR, scaled);
}

// Get Tap Duration (625us / LSB)
float ADXL345_getTapDuration(void)
{
    return ADXL345_readRegister8(ADXL345_REG_DUR) * 0.000625f;
}

// Set Double Tap Latency (1.25ms / LSB)
void ADXL345_setDoubleTapLatency(float latency)
{
  // FIXME - add constrain() function to avoid over-runs.
  //  uint8_t scaled = constrain(latency / 0.00125f, 0, 255);
  uint8_t scaled = latency / 0.00125f;
  ADXL345_writeRegister8(ADXL345_REG_LATENT, scaled);
}

// Get Double Tap Latency (1.25ms / LSB)
float ADXL345_getDoubleTapLatency()
{
    return ADXL345_readRegister8(ADXL345_REG_LATENT) * 0.00125f;
}

// Set Double Tap Window (1.25ms / LSB)
void ADXL345_setDoubleTapWindow(float window)
{
  // FIXME - add constrain() function to avoid over-runs.
  //  uint8_t scaled = constrain(window / 0.00125f, 0, 255);
    uint8_t scaled = window / 0.00125f;
    ADXL345_writeRegister8(ADXL345_REG_WINDOW, scaled);
}

// Get Double Tap Window (1.25ms / LSB)
float ADXL345_getDoubleTapWindow(void)
{
    return ADXL345_readRegister8(ADXL345_REG_WINDOW) * 0.00125f;
}

// Set Activity Threshold (62.5mg / LSB)
void ADXL345_setActivityThreshold(float threshold)
{
  // FIXME - add constrain() function to avoid over-runs.
  //  uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
    uint8_t scaled = threshold / 0.0625f;
    ADXL345_writeRegister8(ADXL345_REG_THRESH_ACT, scaled);
}

// Get Activity Threshold (65.5mg / LSB)
float ADXL345_getActivityThreshold(void)
{
    return ADXL345_readRegister8(ADXL345_REG_THRESH_ACT) * 0.0625f;
}

// Set Inactivity Threshold (65.5mg / LSB)
void ADXL345_setInactivityThreshold(float threshold)
{
  // FIXME - add constrain() function to avoid over-runs.
  //  uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
  uint8_t scaled = threshold / 0.0625f;
  ADXL345_writeRegister8(ADXL345_REG_THRESH_INACT, scaled);
}

// Get Incactivity Threshold (65.5mg / LSB)
float ADXL345_getInactivityThreshold(void)
{
    return ADXL345_readRegister8(ADXL345_REG_THRESH_INACT) * 0.0625f;
}

// Set Inactivity Time (s / LSB)
void ADXL345_setTimeInactivity(uint8_t time)
{
    ADXL345_writeRegister8(ADXL345_REG_TIME_INACT, time);
}

// Get Inactivity Time (s / LSB)
uint8_t ADXL345_getTimeInactivity(void)
{
    return ADXL345_readRegister8(ADXL345_REG_TIME_INACT);
}

// Set Free Fall Threshold (65.5mg / LSB)
void ADXL345_setFreeFallThreshold(float threshold)
{
  // FIXME - add constrain() function to avoid over-runs.
  //  uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
  uint8_t scaled = threshold / 0.0625f;
  ADXL345_writeRegister8(ADXL345_REG_THRESH_FF, scaled);
}

// Get Free Fall Threshold (65.5mg / LSB)
float ADXL345_getFreeFallThreshold(void)
{
    return ADXL345_readRegister8(ADXL345_REG_THRESH_FF) * 0.0625f;
}

// Set Free Fall Duratiom (5ms / LSB)
void ADXL345_setFreeFallDuration(float duration)
{
  // FIXME - add constrain() function to avoid over-runs.
  //uint8_t scaled = constrain(duration / 0.005f, 0, 255);
  uint8_t scaled = duration / 0.005f;
  ADXL345_writeRegister8(ADXL345_REG_TIME_FF, scaled);
}

// Get Free Fall Duratiom
float ADXL345_getFreeFallDuration()
{
    return ADXL345_readRegister8(ADXL345_REG_TIME_FF) * 0.005f;
}

void ADXL345_setActivityX(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 6, state);
}

bool ADXL345_getActivityX(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 6);
}

void ADXL345_setActivityY(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 5, state);
}

bool ADXL345_getActivityY(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 5);
}

void ADXL345_setActivityZ(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 4, state);
}

bool ADXL345_getActivityZ(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 4);
}

void ADXL345_setActivityXYZ(bool state)
{
    uint8_t value;

    value = ADXL345_readRegister8(ADXL345_REG_ACT_INACT_CTL);

    if (state)
    {
	value |= 0b00111000;
    } else
    {
	value &= 0b11000111;
    }

    ADXL345_writeRegister8(ADXL345_REG_ACT_INACT_CTL, value);
}


void ADXL345_setInactivityX(bool state) 
{
    ADXL345_writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 2, state);
}

bool ADXL345_getInactivityX(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 2);
}

void ADXL345_setInactivityY(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 1, state);
}

bool ADXL345_getInactivityY(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 1);
}

void ADXL345_setInactivityZ(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 0, state);
}

bool ADXL345_getInactivityZ(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 0);
}

void ADXL345_setInactivityXYZ(bool state)
{
    uint8_t value;

    value = ADXL345_readRegister8(ADXL345_REG_ACT_INACT_CTL);

    if (state)
    {
	value |= 0b00000111;
    } else
    {
	value &= 0b11111000;
    }

    ADXL345_writeRegister8(ADXL345_REG_ACT_INACT_CTL, value);
}

void ADXL345_setTapDetectionX(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_TAP_AXES, 2, state);
}

bool ADXL345_getTapDetectionX(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_TAP_AXES, 2);
}

void ADXL345_setTapDetectionY(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_TAP_AXES, 1, state);
}

bool ADXL345_getTapDetectionY(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_TAP_AXES, 1);
}

void ADXL345_setTapDetectionZ(bool state)
{
    ADXL345_writeRegisterBit(ADXL345_REG_TAP_AXES, 0, state);
}

bool ADXL345_getTapDetectionZ(void)
{
    return ADXL345_readRegisterBit(ADXL345_REG_TAP_AXES, 0);
}

void ADXL345_setTapDetectionXYZ(bool state)
{
    uint8_t value;

    value = ADXL345_readRegister8(ADXL345_REG_TAP_AXES);

    if (state)
    {
	value |= 0b00000111;
    } else
    {
	value &= 0b11111000;
    }

    ADXL345_writeRegister8(ADXL345_REG_TAP_AXES, value);
}


void ADXL345_useInterrupt(ADXL345_int_t interrupt)
{
    if (interrupt == 0)
    {
	ADXL345_writeRegister8(ADXL345_REG_INT_MAP, 0x00);
    } else
    {
	ADXL345_writeRegister8(ADXL345_REG_INT_MAP, 0xFF);
    }

    ADXL345_writeRegister8(ADXL345_REG_INT_ENABLE, 0xFF);
}


/**
 * getFifoCtl(void)
 * returns the value of the FifoCtl register
 */
uint8_t ADXL345_getFifoCtl(void) {
    return ADXL345_readRegister8(ADXL345_REG_FIFO_CTL);
}

/**
 * setFifoCtl(ctlVal)
 * Sets the value of the FifoCtl register to ctlVal.
 */
uint8_t ADXL345_setFifoCtl(uint8_t ctlVal) {
  return ADXL345_writeRegister8(ADXL345_REG_FIFO_CTL,ctlVal);
}

/**
 * getFifoStatus(void)
 * returns the number of rows of data available in the FIFO buffer.
 */
uint8_t ADXL345_getFifoStatus(void) {
    return ADXL345_readRegister8(ADXL345_REG_FIFO_STATUS);
}

ADXL345_Activites ADXL345_readActivites(void)
{
  ADXL345_Activites a;
  uint8_t data = ADXL345_readRegister8(ADXL345_REG_INT_SOURCE);
  
  a.isOverrun = ((data >> ADXL345_OVERRUN) & 1);
  a.isWatermark = ((data >> ADXL345_WATERMARK) & 1);
  a.isFreeFall = ((data >> ADXL345_FREE_FALL) & 1);
  a.isInactivity = ((data >> ADXL345_INACTIVITY) & 1);
  a.isActivity = ((data >> ADXL345_ACTIVITY) & 1);
  a.isDoubleTap = ((data >> ADXL345_DOUBLE_TAP) & 1);
  a.isTap = ((data >> ADXL345_SINGLE_TAP) & 1);
  a.isDataReady = ((data >> ADXL345_DATA_READY) & 1);
  
  data = ADXL345_readRegister8(ADXL345_REG_ACT_TAP_STATUS);
  
  a.isActivityOnX = ((data >> 6) & 1);
  a.isActivityOnY = ((data >> 5) & 1);
  a.isActivityOnZ = ((data >> 4) & 1);
  a.isTapOnX = ((data >> 2) & 1);
  a.isTapOnY = ((data >> 1) & 1);
  a.isTapOnZ = ((data >> 0) & 1);
  
  return a;
}



/**
 * enableFifo():
 * Puts the FIFO into FIFO mode, disables the  DATA_READY intrrupt
 * and enables the WATERMARK interrupt.
 * WATERMARK is set to 30 samples.
 */
void ADXL345_enableFifo(void) {
  uint8_t value;
  // Put device into standby mode for programming it:
  ADXL345_writeRegisterBit(ADXL345_REG_POWER_CTL,3,false);
  // Enable FIFO Mode  FIFO_CTL register D6 and D7 = 01
  ADXL345_writeRegisterBit(ADXL345_REG_FIFO_CTL,7,false);
  ADXL345_writeRegisterBit(ADXL345_REG_FIFO_CTL,6,true);
  // Enable watermark interrupt, disable data_ready interrupt
  ADXL345_writeRegisterBit(ADXL345_REG_INT_ENABLE,ADXL345_WATERMARK,true);
  ADXL345_writeRegisterBit(ADXL345_REG_INT_ENABLE,ADXL345_DATA_READY,false);

  value = ADXL345_readRegister8(ADXL345_REG_FIFO_CTL);
  // first zero the samples bits (bits 1-4)
  value &= 0b11100000;
  // Then set the samples bits (bits1-4) to 31 (0b00011111)
  value |= 0b00011111;
  
  ADXL345_writeRegister8(ADXL345_REG_FIFO_CTL, value);

  // Put device into measure mode:
  ADXL345_writeRegisterBit(ADXL345_REG_POWER_CTL,3,true);

  printf("************************************\n");
  printf("ADXL_enableFifo()\n");
  printf("INT_SOURCE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_SOURCE));
  printf("INT_ENABLE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_ENABLE));
  printf("INT_MAP=    0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_MAP));
  printf("POWER_CTL=  0x%02x\n",ADXL345_readRegister8(ADXL345_REG_POWER_CTL));
  printf("FIFO_CTL=   0x%02x\n",ADXL345_readRegister8(ADXL345_REG_FIFO_CTL));
  printf("************************************\n");


}

/**
 * disableFifo():
 * Puts the FIFO into bypass mode, enables the  DATA_READY intrrupt
 * and disable the WATERMARK interrupt.
 */
void ADXL345_disableFifo(void) {
  // Put FIFO into bypass Mode  FIFO_CTL register D6 and D7 = 00
  ADXL345_writeRegisterBit(ADXL345_REG_FIFO_CTL,7,false);
  ADXL345_writeRegisterBit(ADXL345_REG_FIFO_CTL,6,false);
  // Disable watermark interrupt, enable data_ready interrupt.
  ADXL345_writeRegisterBit(ADXL345_REG_INT_ENABLE,ADXL345_WATERMARK,false);
  ADXL345_writeRegisterBit(ADXL345_REG_INT_ENABLE,ADXL345_DATA_READY,true);
}

