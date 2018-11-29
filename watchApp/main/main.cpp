#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "adxl345.h"


#define SDA_PIN 18
#define SCL_PIN 19

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
  uint8_t i;
  printf("i2cScanTask - SCL=GPIO%d, SDA=GPIO%d\n",SCL_PIN,SDA_PIN);
  ADXL345_init(SCL_PIN,SDA_PIN);
  printf("i2c_init() complete\n");

  while(1) {
    i = ADXL345_findDevice();
    printf("ADXL found at address %x\n",i);
    vTaskDelay(5000 / portTICK_RATE_MS);
  } 
}


extern "C" void app_main(void)
{
    nvs_flash_init();

    xTaskCreate(LEDBlinkTask,"Blink",8000,NULL,2,NULL);
    xTaskCreate(i2cScanTask,"i2cScanTask",8000,NULL,2,NULL);

    
    /*gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    */
}

