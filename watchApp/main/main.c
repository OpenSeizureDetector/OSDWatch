#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "adxl345.h"


#include "i2cTask.h"

#include "osd_app.h"


// Create Global Variables
An_Data anD;        // analysis data



esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}


void LEDBlinkTask(void *pvParam) {
  //printf("LEDBlinkTask()");
  //gpio_enable(2,GPIO_OUTPUT);
  gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
  int level = 0;
  while (true) {
    gpio_set_level(GPIO_NUM_4, level);
    level = !level;
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

void i2cScanTask(void *pvParameters) {

  while(1) {
    ADXL345_IVector v = ADXL345_readRaw();
    printf("(%4d, %4d, %4d)\n",v.XAxis,v.YAxis,v.ZAxis);
    vTaskDelay(1000 / portTICK_RATE_MS);
  } 
}

/**
 * Initialise the ADXL345 accelerometer trip to use a FIFO buffer
 * and send an interrupt when the FIFO is full.
 */
void setup_adxl345() {
  uint8_t devAddr;
  
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
  
  printf("Setting to 100Hz data rate\n");
  ADXL345_setDataRate(ADXL345_DATARATE_100HZ);
  //ADXL345_setDataRate(ADXL345_DATARATE_12_5HZ);
  
  /*printf("Setting Data Format - interupts active low.\n");
    if (ADXL345_writeRegister8(ADXL345_REG_DATA_FORMAT,0x2B)==-1) {
    printf("*** Error setting Data Format ***\n");
    } else {
    printf("Data Format Set to 0x%02x\n",
    ADXL345_readRegister8(ADXL345_REG_DATA_FORMAT));
    }
  */
  // Set to 4g range, 4mg/LSB resolution, interrupts active high.
  ADXL345_writeRegister8(ADXL345_REG_DATA_FORMAT,0b00001001);
  
  printf("Starting Measurement\n");
  if (ADXL345_writeRegister8(ADXL345_REG_POWER_CTL,0x08)!=0) {
    printf("*** Error setting measurement Mode ***\n");
  } else {
    printf("Measurement Mode set to 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_POWER_CTL));
  }
  
  
  printf("Enabling Data Ready Interrupt\n");
  if (ADXL345_writeRegister8(ADXL345_REG_INT_ENABLE,0x80)!=0) {
    printf("*** Error enabling interrupt ***\n");
  } else {
    printf("Interrupt Enabled - set to 0x%02x\n",
	   ADXL345_readRegister8(ADXL345_REG_INT_ENABLE));
  }
  
  
  printf("************************************\n");
  printf("DEV_ID=     0x%02x\n",ADXL345_readRegister8(ADXL345_REG_DEVID));
  printf("INT_SOURCE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_SOURCE));
  printf("BW_RATE=    0x%02x\n",ADXL345_readRegister8(ADXL345_REG_BW_RATE));
  printf("DATA_FORMAT=0x%02x\n",ADXL345_readRegister8(ADXL345_REG_DATA_FORMAT));
  printf("INT_ENABLE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_ENABLE));
  printf("INT_MAP=    0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_MAP));
  printf("POWER_CTL=  0x%02x\n",ADXL345_readRegister8(ADXL345_REG_POWER_CTL));
  printf("************************************\n");
  
}


/**
 * monitor the ADXL345 regularly to check register states.
 * Assumes it has been initialised by calling setup_adxl345() before
 * this task is started.
 */
void monitorAdxl345Task(void *pvParameters) {
  ADXL345_IVector r;
  while(1) {
    printf("*****************************************\n");
    printf("*        Periodic Monitoring            *\n");
    printf("INT_ENABLE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_ENABLE));
    printf("INT_MAP=    0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_MAP));
    printf("INT_SOURCE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_SOURCE));
    printf("FIFO_STATUS= %d\n",ADXL345_readRegister8(ADXL345_REG_FIFO_STATUS));
    printf("Interrupt Pin GPIO%d value = %d\n",INTR_PIN,gpio_get_level((gpio_num_t)INTR_PIN));
    //for (int i=0;i<33;i++) {
    r = ADXL345_readRaw();
    printf("r.x=%7d, r.y=%7d, r.z=%7d\n",r.XAxis,r.YAxis,r.ZAxis);
    //}
    printf("INT_SOURCE= 0x%02x\n",ADXL345_readRegister8(ADXL345_REG_INT_SOURCE));
    printf("*****************************************\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}




//extern "C"
void app_main(void)
{

  /* First Start the I2C task so that other tasks can access the bus */
  i2cConfig_t conf;
  conf.port = I2C_NUM_0;
  conf.sda = (gpio_num_t)SDA_PIN;
  conf.scl = (gpio_num_t)SCL_PIN;
  conf.clkSpeed = 100000;
  conf.gpio_pullup = true;
  printf("Starting i2cTask....\n");
  xTaskCreate(i2cTask,"i2cTask",8000,&conf,2,NULL);

  //nvs_flash_init();
  
  //setup_adxl345();
  
  printf("Starting Blink Task...\n");
  xTaskCreate(LEDBlinkTask,"Blink",8000,NULL,2,NULL);
  //xTaskCreate(i2cScanTask,"i2cScanTask",8000,NULL,2,NULL);
  //xTaskCreate(monitorAdxl345Task,"monitorAdxl345Task",8000,NULL,2,NULL);
 
  printf("Starting heartTask...\n");
  xTaskCreate(heartTask,"heartTask",8000,NULL,2,NULL);


  
  
  //runi2cTaskTest((gpio_num_t)SDA_PIN,(gpio_num_t)SCL_PIN);


}

