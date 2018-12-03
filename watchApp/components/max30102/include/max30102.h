/*
 * MAX30102 library - based on https://github.com/Protocentral/Pulse_MAX30102
  Library for the Maxim MAX30100 pulse oximetry system
  Connor Huffine/Kontakt
  February 2016
*/
#include "i2cTask.h"

// Registers
#define MAX30100_INT_STATUS1     0x00  // Which interrupts are tripped
#define MAX30100_INT_STATUS2     0x01  // Which interrupts are tripped
#define MAX30100_INT_ENABLE1     0x02  // Which interrupts are active
#define MAX30100_INT_ENABLE2    0x03  // Which interrupts are active

#define MAX30100_FIFO_WR_PTR    0x04  // Where data is being written
#define MAX30100_OVRFLOW_CTR    0x05  // Number of lost samples
#define MAX30100_FIFO_RD_PTR    0x06  // Where to read from
#define MAX30100_FIFO_DATA      0x07 // Ouput data buffer

#define MAX30100_FIFO_CONFIG    0x08
#define MAX30100_MODE_CONFIG    0x09  // Control register
#define MAX30100_SPO2_CONFIG    0x0A  // Oximetry settings
#define MAX30100_LED1_AMP        0x0C
#define MAX30100_LED2_AMP        0x0D
#define MAX30100_PILOT           0x10
#define MAX310100_MLED_CTRL1 	0x11
#define MAX310100_MLED_CTRL2	0x12
#define MAX30100_TEMP_INTG      0x1F  // Temperature value, whole number
#define MAX30100_TEMP_FRAC      0x20  // Temperature value, fraction
#define MAX30100_DIE_TEMP        0x21
#define MAX30100_PROX_INT_TRESH  0x30
#define MAX30100_REV_ID         0xFE  // Part revision
#define MAX30100_PART_ID        0xFF  // Part ID, normally 0x11

#define MAX30100_ADDRESS        0x57  // 8bit address converted to 7bit

typedef unsigned char uint8_t;
//typedef unsigned int uint16_t;
typedef enum{ // This is the same for both LEDs
  pw200,    // 200us pulse
  pw400,    // 400us pulse
  pw800,    // 800us pulse
  pw1600    // 1600us pulse
} MAX30100_pulseWidth_t;

typedef enum{
  sr50,    // 50 samples per second
  sr100,   // 100 samples per second
  sr167,   // 167 samples per second
  sr200,   // 200 samples per second
  sr400,   // 400 samples per second
  sr600,   // 600 samples per second
  sr800,   // 800 samples per second
  sr1000   // 1000 samples per second
} MAX30100_sampleRate_t;

typedef enum{
  i0,    // No current
  i4,    // 4.4mA
  i8,    // 7.6mA
  i11,   // 11.0mA
  i14,   // 14.2mA
  i17,   // 17.4mA
  i21,   // 20.8mA
  i27,   // 27.1mA
  i31,   // 30.6mA
  i34,   // 33.8mA
  i37,   // 37.0mA
  i40,   // 40.2mA
  i44,   // 43.6mA
  i47,   // 46.8mA
  i50    // 50.0mA
} MAX30100_ledCurrent_t;


extern long MAX30100_IR;      // Last IR reflectance datapoint
extern long MAX30100_RED;     // Last Red reflectance datapoint

void  MAX30100_setLEDs(MAX30100_pulseWidth_t pw, MAX30100_ledCurrent_t red, MAX30100_ledCurrent_t ir);  // Sets the LED state
void  MAX30100_setSPO2(MAX30100_sampleRate_t sr); // Setup the SPO2 sensor, disabled by default
int   MAX30100_getNumSamp(void);       // Get number of samples
void  MAX30100_readSensor(void);       // Updates the values
void  MAX30100_shutdown(void);   // Instructs device to power-save
void  MAX30100_reset(void);      // Resets the device
void  MAX30100_startup(void);    // Leaves power-save
int   MAX30100_getRevID(void);   // Gets revision ID
int   MAX30100_getPartID(void);  // Gets part ID
void  MAX30100_begin(MAX30100_pulseWidth_t pw,
		     MAX30100_ledCurrent_t ir,
		     MAX30100_sampleRate_t sr);
void  MAX30100_printRegisters(void); // Dumps contents of registers for debug


