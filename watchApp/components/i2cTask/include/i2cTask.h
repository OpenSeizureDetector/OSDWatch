/*
 * i2cTask FreeRTOS Library Header File
 * i2cTask is a FreeRTOS task that monitors a queue for requests to process
 *       i2c transactions, processes the requested transaction, and notifies
 *       the requesting task of the result.
 *       Its purpose is to allow multiple tasks to access the I2C bus without
 *       creating conflicts between tasks.
 * Copyright Graham Jones, 2018.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifdef ESP_PLATFORM
#ifndef I2CTASK_h
#define I2CTASK_h

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
//#warning "Compiling for ESP_PLATFORM - OK"

#define I2C_DEBUG 0     // include i2c debugging printouts
#define QUEUE_SIZE 16   // length of task queue (I think 16 will be plenty)
#define I2C_MASTER_ACK_EN   true    /*!< Enable ack check for master */
#define I2C_MASTER_ACK_DIS  false   /*!< Disable ack check for master */


typedef enum {
    I2C_TX,
    I2C_RX,
    I2C_SCAN
} i2cTransactionType_t;


typedef struct {
  i2c_port_t port;  // I2C port number (I2C_NUM_0 or I2C_NUM_1)
  gpio_num_t sda;   // GPIO pin number of SDA line
  gpio_num_t scl;   // GPIO pin number of SCL line
  uint32_t clkSpeed; // Clock Speed (eg. 100000)
  bool gpio_pullup;  // true to enable internal pullup resistors.
  TaskHandle_t xTaskToNotify;   // We send a notification to this task when i2c is ready to use.
} i2cConfig_t;

typedef struct {
  i2cTransactionType_t type;
  uint8_t devAddr;
  uint16_t reg;     // stored as 16 bits in case we need 16 bit addresses later.
  uint8_t data[256];    // pointer to data to be transmitted (or storage space for data to be read)
  int nData;        // number of bytes to read/write.
  bool wait;     // if true, i2cRunTransaction waits for the transaction to complete before returning.
  TaskHandle_t taskToNotify;
  uint32_t retVal;  // return value from i2c transaction

} i2cTransaction_t;


void i2cTask(void *arg);
void i2cRunTransaction(i2cTransaction_t *trans);
uint8_t I2C_writeRegister8(uint8_t devAddr, uint8_t reg, uint8_t value);
uint8_t I2C_readRegister8(uint8_t devAddr, uint8_t reg);
int16_t I2C_readRegister16(uint8_t devAddr, uint8_t reg);
void runi2cTaskTest(gpio_num_t sda, gpio_num_t scl);



#endif // ifndef I2CTASK_h
#else
    #warning "This only works for the esp-idf platform at the moment!"
#endif  // ifdef ESP_PLATFORM

