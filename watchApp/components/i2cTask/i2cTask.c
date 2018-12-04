/*
 *  i2cTask.c 
 * i2cTask is a FreeRTOS task that monitors a queue for requests to process
 *       i2c transactions, processes the requested transaction, and notifies
 *       the requesting task of the result.
 *       Its purpose is to allow multiple tasks to access the I2C bus without
 *       creating conflicts between tasks.
 * The overall structure was inspired by 
 *     https://github.com/iotexpert/PSoC-FreeRTOS-Examples/blob/master/7-Shared-I2C.cydsn/i2cmaster.h
 * Copyright Graham Jones, 2018.

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
#include "i2cTask.h"


// Global Variables
static QueueHandle_t i2cQueue;
i2c_config_t conf;

esp_err_t checkDevice(i2c_port_t port, uint8_t devAddr) {
  //esp_err_t err;
  //err = i2c_param_config(port, &conf);
  //if (!err) err = i2c_driver_install(port, conf.mode, 0, 0, 0);
  //if (err) printf("checkDevice: ERROR - %04x\n", err);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd,
			(devAddr << 1) | I2C_MASTER_WRITE,
			I2C_MASTER_ACK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(port, cmd, 50 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  //i2c_driver_delete(port);
  return ret;
}
  


esp_err_t readBytes(i2c_port_t port,
		    uint8_t devAddr,
		    uint8_t reg,
		    size_t nData,
		    uint8_t *data) {
  int32_t ticksToWait = pdMS_TO_TICKS(100); // 100ms timeout
  esp_err_t err;
  //err = i2c_param_config(port, &conf);
  //if (!err) err = i2c_driver_install(port, conf.mode, 0, 0, 0);
  //if (err) printf("checkDevice: ERROR - %04x\n", err);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd,
			(devAddr << 1) | I2C_MASTER_WRITE,
			I2C_MASTER_ACK_EN);
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd,
			(devAddr << 1) | I2C_MASTER_READ,
			I2C_MASTER_ACK_EN);
  i2c_master_read(cmd, data, nData, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(port, cmd, ticksToWait);
  if (I2C_DEBUG) printf("readBytes - data[0]=%02x\n",data[0]);
  i2c_cmd_link_delete(cmd);
  //i2c_driver_delete(port);
  return err;
}

esp_err_t writeBytes(i2c_port_t port,
		     uint8_t devAddr,
		     uint8_t reg,
		     size_t nData,
		     uint8_t *data) {
  int32_t ticksToWait = pdMS_TO_TICKS(100); // 100ms timeout
  esp_err_t err;
  //err = i2c_param_config(port, &conf);
  //if (!err) err = i2c_driver_install(port, conf.mode, 0, 0, 0);
  //if (err) printf("checkDevice: ERROR - %04x\n", err);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK_EN);
  i2c_master_write(cmd, (uint8_t*) data, nData, I2C_MASTER_ACK_EN);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(port, cmd, ticksToWait);
  i2c_cmd_link_delete(cmd);
  //i2c_driver_delete(port);
  return err;
}



/*
 * NAME: i2cTask(void *arg)
 * DESC: A task that listens for i2c transaction requests on queue i2cQueue
 *         and executes them.
 * PARAM: *arg should be a pointer to a i2cConfig_t structure
 */
void i2cTask(void *arg) {
  //uint32_t retVal;
  i2cTransaction_t *trans;
  i2cConfig_t *param = (i2cConfig_t*)arg;

  printf("i2cTask - SDA=GPIO_%02d, SCL=GPIO_%02d\n",
	param->sda,
	param->scl);

  // Configure I2C Bus
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = param->sda;
  if (param->gpio_pullup)
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  else
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = param->scl;
  if (param->gpio_pullup)
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  else
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = param->clkSpeed;

  esp_err_t err = i2c_param_config(param->port, &conf);
  if (!err) err = i2c_driver_install(param->port, conf.mode, 0, 0, 0);

  if(!err) {
    printf("i2cTask - initialised i2c bus ok\n");
  } else {
    printf("i2cTask - **** ERROR - FAILED TO INITIALISE I2C BUS ****\n");
  }
  
  // Create the transaction queue - a queue of pointers
  i2cQueue = xQueueCreate(QUEUE_SIZE,sizeof(char *));

  // Loop continuously, waiting for transaction requests
  printf("i2cTask - queue created - waiting for transactions on queue\n");
  while(1) {
    if(xQueueReceive(i2cQueue,&trans,portMAX_DELAY) == pdTRUE)
        {
	  if (I2C_DEBUG) printf("xQueueReceive\n");
	  if (I2C_DEBUG) printf("xQueueReceive - trans=%p\n", (void*)trans);
	  if (I2C_DEBUG) printf("xQueueReceive - devAddr=%02x, reg=%02x\n",
		 trans->devAddr,
		 trans->reg);

	  if (trans->type == I2C_TX) {
	    trans->retVal = writeBytes(param->port,
				trans->devAddr,
				trans->reg,
				trans->nData,
				trans->data);
	  }
	  else if (trans->type == I2C_RX) {
	    trans->retVal = readBytes(param->port,
				trans->devAddr,
				trans->reg,
				trans->nData,
				trans->data);
	  }
	  else if (trans->type == I2C_SCAN) {
	    for( uint8_t devAddr = 0; devAddr<=127;devAddr++ ) {
	      esp_err_t retVal = checkDevice(param->port,devAddr);
	      if (retVal==ESP_OK) {
		if (I2C_DEBUG) printf("%02x:%03x\n",
		       devAddr, retVal);
		trans->data[devAddr] = devAddr;
	      } else {
		trans->data[devAddr] = 0;
	      }
	    }
	  }
	  else {
	    printf("i2cTask - ERROR - Unrecognised transaction type %d",
		   trans->type);
	  }
	  //vTaskDelay(1000 / portTICK_PERIOD_MS);

	  if (trans->wait) {
	    if (I2C_DEBUG) printf("Notifying originating task...");
	    xTaskNotifyGive(trans->taskToNotify);
	  }
	}
  }

}



void i2cRunTransaction(i2cTransaction_t *trans) {
  uint32_t ulNotificationValue;
  // allow 2 seconds for response before giving up
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 2000 );
  trans->taskToNotify = xTaskGetCurrentTaskHandle();
  if (I2C_DEBUG) printf("i2cRunTransaction trans=%p\n", (void*)trans);
  if(xQueueSend(i2cQueue,&trans,portMAX_DELAY) != pdTRUE) {
    fprintf(stderr,"**** ERROR: Failed to add transaction to the queue ****\n");
  }
  if(trans->wait) {
    /* Wait to be notified that the transmission is complete.  See:
       https://www.freertos.org/RTOS_Task_Notification_As_Binary_Semaphore.html
    */
    if (I2C_DEBUG) printf("i2cRunTransaction - waiting for notification....\n");
    ulNotificationValue = ulTaskNotifyTake( pdTRUE,
                                            xMaxBlockTime );
    if (I2C_DEBUG) printf("i2cRunTransaction - notification received\n");

    if( ulNotificationValue == 1 )
    {
      if (I2C_DEBUG) printf("i2cRunTransaction - succcessful completion\n");
        /* The transmission ended as expected. */
    }
    else
    {
  fprintf(stderr,"i2cRunTransaction - ERROR - TIMEOUT\n");
        /* The call to ulTaskNotifyTake() timed out. */
    }
  }
}



/*****************************************
 * Basic register input/output functions *
 *****************************************/
// Write byte to register
uint8_t I2C_writeRegister8(uint8_t devAddr, uint8_t reg, uint8_t value)
{
  i2cTransaction_t trans;
  trans.type = I2C_TX;
  trans.devAddr=devAddr;
  trans.reg = reg;
  trans.nData = 1;
  trans.data[0]=value;
  trans.wait = true;
  i2cRunTransaction(&trans);
  if (trans.retVal) {
    // non-zero response = error
    printf("I2C_writeRegister8(0x%x, 0x%x) - Error\n",devAddr,reg);
    return -1;
  } else {
    return value;
  }
}


/* Reads a single byte from the register reg
 * @return: the byte read, or -1 on error.
 */
uint8_t I2C_readRegister8(uint8_t devAddr, uint8_t reg) {
  i2cTransaction_t trans;
  uint8_t byte;
  trans.type = I2C_RX;
  trans.devAddr=devAddr;
  trans.reg = reg;
  trans.nData = 1;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  byte = trans.data[0];
  if (trans.retVal) {
    // non-zero response = error
    printf("I2C_readRegister8(0x%x, 0x%x) - Error\n",devAddr,reg);
    return -1;
  } else {
    return byte;
  }
}


/* Reads a 2 byte word from the device at address devAddr, register reg
 * @return: the word read, or -1 on error.
 */
int16_t I2C_readRegister16(uint8_t devAddr, uint8_t reg) {
  int16_t value;
  uint8_t data[2];
  /* Note - this fiddle is something to do with C99 or C11 because
        saying uint8_t bytes[2]; bytes = &(trans.data[0]) failed with an error
        assignment to expression with array type
  */
  uint8_t *bytes;
  bytes = &data[0];
  i2cTransaction_t trans;
  trans.type = I2C_RX;
  trans.devAddr=devAddr;
  trans.reg = reg;
  trans.nData = 2;
  trans.wait = true;
  i2cRunTransaction(&trans);
  //printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
  //	 trans.retVal,(int)trans.data[0]);
  bytes = &(trans.data[0]);
  if (trans.retVal) {
    // non-zero response = error
    printf("I2C_readRegister16(0x%x, 0x%x) - Error\n",devAddr,reg);
    return -1;
  } else {
    value = bytes[1]<<8 | bytes[0];
    return value;
  }
}






/**
 * test the function of the i2cTask by scanning the i2c bus attached to
 * pins sda and scl, then attempting to read from two devices.
 */
void runi2cTaskTest(gpio_num_t sda, gpio_num_t scl) {


  /* //  Assume that the main program has started the i2cTask already 
  i2cConfig_t conf;
  conf.port = I2C_NUM_0;
  conf.sda = sda;
  conf.scl = scl;
  conf.clkSpeed = 100000;
  conf.gpio_pullup = true;
  xTaskCreate(i2cTask,"i2cTask",8000,&conf,2,NULL);
  */
  
  i2cTransaction_t trans;


  trans.type = I2C_SCAN;
  trans.wait = true;
  printf("main - trans=%p\n", &trans);
  i2cRunTransaction(&trans);
  int nCol = 16;
  for (int r=0; r<128/nCol;r++) {
    for (int c=0; c<nCol;c++) {
      int i = nCol * r + c;
      if (trans.data[i]==0)
	printf(" %02x:-- ",i);
      else
	printf(" %02x:%02x ",i,trans.data[i]);
    }
    printf("\n");
  }


  
  trans.type = I2C_RX;
  trans.devAddr=0x53;
  trans.reg = 0x00;
  trans.nData = 1;
  trans.wait = true;

  i2cRunTransaction(&trans);
  printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
	 trans.retVal,(int)trans.data[0]);

    
  trans.type = I2C_RX;
  trans.devAddr=0x57;
  trans.reg = 0x00;
  trans.nData = 1;
  trans.wait = true;

  i2cRunTransaction(&trans);
  printf("i2cRunTransaction Complete retVal=%d, data[0]=%02x\n",
	 trans.retVal,(int)trans.data[0]);



}
