#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "adxl345.h"


//#include "i2cTask.h"

#include "osd_app.h"

#include "displayTask.h"
#include "accelTask.h"

/* SHOULD NOT BE NEEDED!! */
#include "ssd1306.h"

// Create Global Variables
An_Data anD;        // analysis data
int debug = 0;            // enable or disable logging output
Sd_Settings sdS;        // SD setings structure.



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

  printf("Starting accelTask\n");
  xTaskCreate(accelTask,"accelTask",8000,NULL,2,NULL);
  xTaskCreate(monitorAdxl345Task,"monitorAdxl345Task",8000,NULL,2,NULL);
 
  //printf("Starting heartTask...\n");
  //xTaskCreate(heartTask,"heartTask",8000,NULL,2,NULL);

  //printf("Starting displayTask\n");
  //xTaskCreate(displayTask,"displayTask",8000,NULL,2,NULL);

  //runi2cTaskTest((gpio_num_t)SDA_PIN,(gpio_num_t)SCL_PIN);


}

