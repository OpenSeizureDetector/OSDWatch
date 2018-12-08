/*
 * OpenSeizureDetector - OSDWatch Version.
 *
 * OSDWatch - a simple accelerometer based seizure detector that runs on 
 * an ESP32 system on chip, with ADXL345 accelerometer attached by i2c 
 * interface.
 *
 * See http://openseizuredetector.org for more information.
 *
 * Copyright Graham Jones, 2015, 2016, 2017, 2018
 *
 * This file is part of OSDWatch.
 *
 * OSDWatch is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * OSDWatch is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with OSDWatch.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "osd_app.h"
#include "adxl345.h"
#include "accelTask.h"
#include "displayTask.h"
#include "driver/gpio.h"

xQueueHandle *accelDataQueue;

/**
 * Initialise the ADXL345 accelerometer trip to use a FIFO buffer
 * and send an interrupt when the FIFO is full.
 */
void setup_adxl345() {
  uint8_t devAddr;
  int retVal;
  
  printf("setup_adxl345()\n");
  
  // Initialise the ADXL345 i2c interface and search for the ADXL345
  devAddr = ADXL345_init(SCL_PIN,SDA_PIN);
  printf("ADXL345 found at address 0x%x\n",devAddr);
  
  //printf("Clearing Settings...\n");
  //ADXL345_clearSettings();
  
  // Bypass the FIFO buffer
  ADXL345_writeRegisterBit(ADXL345_REG_FIFO_CTL,6,0);
  ADXL345_writeRegisterBit(ADXL345_REG_FIFO_CTL,7,0);
  
  printf("Setting 4G range\n");
  ADXL345_setRange(ADXL345_RANGE_4G);
  
  printf("Setting to 12.5Hz data rate\n");
  //ADXL345_setDataRate(ADXL345_DATARATE_100HZ);
  ADXL345_setDataRate(ADXL345_DATARATE_12_5HZ);
  
  /*printf("Setting Data Format - interupts active low.\n");
    if (ADXL345_writeRegister8(ADXL345_REG_DATA_FORMAT,0x2B)!=0) {
    printf("*** Error setting Data Format ***\n");
    } else {
    printf("Data Format Set to 0x%02x\n",
    ADXL345_readRegister8(ADXL345_REG_DATA_FORMAT));
    }
  */
  // Set to 4g range, 4mg/LSB resolution, interrupts active high.
  //ADXL345_writeRegister8(ADXL345_REG_DATA_FORMAT,0b00001001);
  
  printf("Starting Measurement\n");
  retVal = ADXL345_writeRegister8(ADXL345_REG_POWER_CTL,0x08);
  if (retVal!=0) {
    printf("*** Error setting measurement Mode - retval = 0x%02x***\n",retVal);
  } else {
    printf("Measurement Mode set to 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_POWER_CTL));
  }
  
  
  printf("Enabling Data Ready Interrupt\n");
  retVal = ADXL345_writeRegister8(ADXL345_REG_INT_ENABLE,0x80);
  if (retVal) {
    printf("*** Error enabling interrupt - retVal =0x%02x***\n",retVal);
  } else {
    printf("Interrupt Enabled - set to 0x%02x\n",
	   ADXL345_readRegister8(ADXL345_REG_INT_ENABLE));
  }
  
  
  printf("************************************\n");
  printf("*           setup_adxl345()        *\n");
  printf("DEV_ID=     0x%02x\n",ADXL345_readRegister8(ADXL345_REG_DEVID));
  printf("INT_SOURCE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_SOURCE));
  printf("BW_RATE=    0x%02x\n",ADXL345_readRegister8(ADXL345_REG_BW_RATE));
  printf("DATA_FORMAT=0x%02x\n",ADXL345_readRegister8(ADXL345_REG_DATA_FORMAT));
  printf("INT_ENABLE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_ENABLE));
  printf("INT_MAP=    0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_MAP));
  printf("POWER_CTL=  0x%02x\n",ADXL345_readRegister8(ADXL345_REG_POWER_CTL));
  printf("INT_SOURCE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_SOURCE));
  printf("FIFO_STATUS= %d\n",ADXL345_readRegister8(ADXL345_REG_FIFO_STATUS));
  printf("Interrupt Pin GPIO%d value = %d\n",INTR_PIN,GPIO_INPUT_GET(INTR_PIN));
  printf("************************************\n");
  
}


/**
 * monitor the ADXL345 regularly to check register states.
 * Assumes it has been initialised by calling setup_adxl345() before
 * this task is started.
 */
void monitorAdxl345Task(void *pvParameters) {
  //ADXL345_IVector r;
  TaskHandle_t *xTaskToNotify = pvParameters;
  
  // Notify  the sending task that we are ready
  printf("monitorAdxl345Task - Notifying parent task that we are ready\n");
  xTaskNotifyGive(*xTaskToNotify);

  while(1) {
    printf("*****************************************\n");
    printf("*        Periodic Monitoring            *\n");
    printf("INT_ENABLE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_ENABLE));
    printf("INT_MAP=    0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_MAP));
    printf("INT_SOURCE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_SOURCE));
    printf("FIFO_STATUS= %d\n",ADXL345_readRegister8(ADXL345_REG_FIFO_STATUS));
    //    printf("Interrupt Pin GPIO%d value = %d\n",INTR_PIN,gpio_read(INTR_PIN));
    printf("Interrupt Pin GPIO%d value = %d\n",INTR_PIN,GPIO_INPUT_GET(INTR_PIN));
    printf("*****************************************\n");
    vTaskDelay(3000 / portTICK_RATE_MS);
  }
}


// Interrupt handler - looks at the accerometer data ready
// signal on pin INTR_PIN, and the configuration switch on BUTTON_IO_PIN
static void IRAM_ATTR gpio_intr_handler(void* arg)
{
  uint32_t gpio_num = (uint32_t) arg;
  if (gpio_num ==  INTR_PIN) {
    xQueueSendFromISR(accelDataQueue, &gpio_num, NULL);
  }

  //CLEAR THE STATUS IN THE W1 INTERRUPT REGISTER
  //GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, status);       
  
  //#warning "FIXME - gpio_intr_handler needs converting to ESP32"
}


/**
 * accel_handler():  Called whenever accelerometer data is available.
 * Add data to circular buffer accData[] and increments accDataPos to show
 * the position of the latest data in the buffer.
 */
void accel_handler(ADXL345_IVector *data, uint32_t num_samples) {
  int i;

  //if (debug) APP_LOG(APP_LOG_LEVEL_DEBUG,"accel_handler(): num_samples=%d",num_samples);
  if (sdS.sdMode==SD_MODE_RAW) {
    if (debug) APP_LOG(APP_LOG_LEVEL_DEBUG,"num_samples=%d",num_samples);
    //sendRawData(data,num_samples);
  } else {
    // Add the new data to the accData buffer
    for (i=0;i<(int)num_samples;i++) {
      // Wrap around the buffer if necessary
      if (anD.accDataPos>=anD.nSamp) { 
	//anD.accDataPos = 0;
	//anD.accDataFull = 1;
	// a separate task looks for anD.accDataFull being true, and does
	// the analysis if this is the case.
	//do_analysis();
	break;
      }
      // add good data to the accData array
      //anD.accData[anD.accDataPos] =
      //abs(data[i].XAxis)
      //+ abs(data[i].YAxis)
      //+ abs(data[i].ZAxis);
      //anD.accDataPos++;
    }
    //anD.latestAccelData = data[num_samples-1];
  }
}


/* 
 *  Initialises the ADX345 accelerometer, 
 *  If USE_INTR is true,  waits for queue message from
 *  interrupt handler to say data is ready.
 *  Otherwise polls periodically to read data from the adxl345 FIFO
 */
void accelTask(void *pvParameters)
{
  TaskHandle_t *xTaskToNotify = pvParameters;
  ADXL345_IVector r;
  ADXL345_IVector buf[ACC_BUF_LEN];
    char rowStr[32];
  
  printf("receiveAccelDataTask - SCL=GPIO%d, SDA=GPIO%d\n",SCL_PIN,SDA_PIN);

  // Create the transaction queue - a queue of pointers
  accelDataQueue = xQueueCreate(QUEUE_SIZE,sizeof(char *));
  
  if (USE_INTR) {
    printf("SETUP INTERRUPT..\r\n");
    gpio_config_t io_in_conf;
    io_in_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_in_conf.mode = GPIO_MODE_INPUT;
    io_in_conf.pin_bit_mask = INTR_PIN;
    io_in_conf.pull_up_en = 1;
    gpio_config(&io_in_conf);

    gpio_set_intr_type(INTR_PIN, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTR_PIN, gpio_intr_handler, (void*) INTR_PIN);

    printf("Waiting for accelerometer data ready interrupt on gpio %d...\r\n", INTR_PIN);    
  } else {
    printf("NOT using interrupts - will poll adxl345 periodically\n");
  }

  // Now initialise the adxl345 (which also initialises the i2c bus
  setup_adxl345();
  ADXL345_enableFifo();

  // Notify  the sending task that we are ready
  printf("accelTask - Notifying parent task that we are ready\n");
  xTaskNotifyGive(*xTaskToNotify);
  
  while(1) {
    uint32_t data_ts;
    if (USE_INTR) {
      xQueueReceive(accelDataQueue, &data_ts, portMAX_DELAY);
      printf("Accelerometer data ready - GPIO_%d....\n",data_ts);
    } else {
      // Wait for 310 ms - 100Hz sample rate, fifo is 32 readings
      // so we wait for 31 readings = 310 ms.
      vTaskDelay(310 / portTICK_RATE_MS);
      int nFifo = ADXL345_readRegister8(ADXL345_REG_FIFO_STATUS);
      if (nFifo ==32) {
	printf("receiveAccelDataTask() - Warning - FIFO Overflow\n");
      } else {
	//printf("receiveAccelDataTask() - Reading data based on timer - %d readings\n",nFifo);
      }
      data_ts = 0;
    }
    
    /// Now read all the data from the FIFO buffer on the ADXL345.
    int i=0;
    bool finished = false;
    while (!finished) {
      r = ADXL345_readRaw();
      buf[i] = r;
      i++;
      //printf("%d:%d,  receiveAccelDataTask: %dms r.x=%7d, r.y=%7d, r.z=%7d\n",
      //     i,
      //     ADXL345_readRegister8(ADXL345_REG_FIFO_STATUS),
      //     data_ts,r.XAxis,r.YAxis,r.ZAxis);
      
      // have we emptied the fifo or filled our buffer yet?
      if ((ADXL345_readRegister8(ADXL345_REG_FIFO_STATUS)==0)
	  || (i==ACC_BUF_LEN)) finished = 1;
    }
    printf("accelTask: read %d points from fifo, %dms r.x=%7d, r.y=%7d, r.z=%7d\n",
    	   i,
    	   data_ts,r.XAxis,r.YAxis,r.ZAxis);
    sprintf(rowStr," x=%7d mg",r.XAxis);
    printf("%s\n",rowStr);
    displayTask_setRow(1,rowStr);
    sprintf(rowStr," y=%7d mg",r.YAxis);
    printf("%s\n",rowStr);
    displayTask_setRow(2,rowStr);
    sprintf(rowStr," z=%7d mg",r.ZAxis);
    printf("%s\n",rowStr);
    displayTask_setRow(3,rowStr);
    displayTask_updateDisplay();

    // Call the acceleration handler in analysis.c
    accel_handler(buf,i);
  }
}

