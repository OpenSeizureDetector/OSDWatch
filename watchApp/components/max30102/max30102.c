/*
 * MAX30102 library - based on https://github.com/Protocentral/Pulse_MAX30102
 */

#include "max30102.h"

/* Global Variables */
long MAX30100_IR = 0;      // Last IR reflectance datapoint
long MAX30100_RED = 0;     // Last Red reflectance datapoint


/* 'Private' Functions */
void MAX30100_I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
uint8_t MAX30100_I2CreadByte(uint8_t address, uint8_t subAddress);
void MAX30100_I2CwriteByte(uint8_t address, uint8_t reg, uint8_t value);



void MAX30100_setLEDs(MAX30100_pulseWidth_t pw, MAX30100_ledCurrent_t red, MAX30100_ledCurrent_t ir){
  uint8_t reg = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_SPO2_CONFIG);
  reg = reg & 0xFC; // Set LED_PW to 00
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_SPO2_CONFIG, reg | pw);     // Mask LED_PW
 // MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_LED_CONFIG, (red<<4) | ir); // write LED configs
}

void MAX30100_setSPO2(MAX30100_sampleRate_t sr){
  uint8_t reg = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_SPO2_CONFIG);
  reg = reg & 0xE3; // Set SPO2_SR to 000
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_SPO2_CONFIG, reg | (sr<<2)); // Mask SPO2_SR
  reg = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG);
  reg = reg & 0xf8; // Set Mode to 000
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_SPO2_CONFIG, reg | 0x03); // Mask MODE
}

int MAX30100_getNumSamp(void){
    uint8_t wrPtr = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_FIFO_WR_PTR);
    uint8_t rdPtr = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_FIFO_RD_PTR);
    return (abs( 16 + wrPtr - rdPtr ) % 16);
}

void MAX30100_readSensor(void){
  uint8_t temp[6] = {0};  // Temporary buffer for read values
  MAX30100_I2CreadBytes(MAX30100_ADDRESS, MAX30100_FIFO_DATA, &temp[0], 6);  // Read four times from the FIFO
  //temp[0] = 0x11; temp[1] = 0x22; temp[2] = 0x33; temp[4] = 0x44; temp[5] = 0x55;temp[6] = 0x66;
  MAX30100_IR =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
  MAX30100_RED = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
}

void MAX30100_shutdown(void){
  uint8_t reg = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG);  // Get the current register
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG, reg | 0x80);   
}

void MAX30100_reset(void){
  uint8_t reg = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG);  // Get the current register
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG, reg | 0x40);   // mask the RESET bit
}

void MAX30100_startup(void){
  uint8_t reg = MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG);  // Get the current register
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG, reg & 0x7F);   // mask the SHDN bit
}

int MAX30100_getRevID(void){
  return MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_REV_ID);
}

int MAX30100_getPartID(void){
  return MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_PART_ID);
}

void MAX30100_begin(MAX30100_pulseWidth_t pw, MAX30100_ledCurrent_t ir, MAX30100_sampleRate_t sr){

  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_INT_ENABLE1, 0xc0); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_INT_ENABLE2, 0x00); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_FIFO_WR_PTR, 0x00); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_OVRFLOW_CTR, 0x00); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_FIFO_RD_PTR, 0x00); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_FIFO_CONFIG, 0x00); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG, 0x03); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_SPO2_CONFIG, 0x07); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_LED1_AMP , 0x24); 
  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_LED2_AMP, 0x24); 
//  MAX30100_I2CwriteByte(MAX30100_ADDRESS, MAX30100_PILOT, 0x7f); 
  
}

/*void MAX30100_printRegisters(void){
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_INT_STATUS1),  BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_INT_STATUS2),  BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_FIFO_WR_PTR), BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_INT_ENABLE1), BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_OVRFLOW_CTR), BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_FIFO_RD_PTR),   BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_FIFO_DATA), BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_FIFO_CONFIG), BIN);
 // Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_LED_CONFIG),  BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_MODE_CONFIG),   BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_SPO2_CONFIG),   BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_LED1_AMP),      BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_LED2_AMP),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_PILOT ),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX310100_MLED_CTRL1),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX310100_MLED_CTRL2),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_TEMP_INTG),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS,  MAX30100_TEMP_FRAC),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS,MAX30100_DIE_TEMP),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_PROX_INT_TRESH),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_REV_ID),     BIN);
  Serial.println(MAX30100_I2CreadByte(MAX30100_ADDRESS, MAX30100_PART_ID),     BIN);
}
*/


void MAX30100_I2CwriteByte(uint8_t address, uint8_t reg, uint8_t value)
{
  i2cTransaction_t trans;
  trans.type = I2C_TX;
  trans.devAddr=address;
  trans.reg = reg;
  trans.nData = 1;
  trans.data[0]=value;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  if (trans.retVal) {
    // non-zero response = error
    printf("MAX30100_I2CwriteByte(0x%x, 0x%x) - Error\n",address,reg);
  }
}



uint8_t MAX30100_I2CreadByte(uint8_t address, uint8_t reg)
{
  i2cTransaction_t trans;
  trans.type = I2C_RX;
  trans.devAddr=address;
  trans.reg = reg;
  trans.nData = 1;
  trans.data[0]=0;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  if (trans.retVal) {
    // non-zero response = error
    printf("MAX30100_I2CwriteByte(0x%x, 0x%x) - Error\n",address,reg);
    return -1;
  } else {
    return trans.data[0];
  }
}

void MAX30100_I2CreadBytes(uint8_t address, uint8_t reg,
			   uint8_t * dest, uint8_t count)
{
  i2cTransaction_t trans;
  trans.type = I2C_RX;
  trans.devAddr=address;
  trans.reg = reg;
  trans.nData = count;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  if (trans.retVal) {
    // non-zero response = error
    printf("MAX30100_I2CwriteByte(0x%x, 0x%x) - Error\n",address,reg);
    dest[0] = -1;
  } else {
    for (int i=0;i<count;i++)
      dest[i] = trans.data[i];
  }
}

