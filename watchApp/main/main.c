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

// Create Global Variables
An_Data anD;        // analysis data
int debug = 0;            // enable or disable logging output
Sd_Settings sdS;        // SD setings structure.



esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}


void LEDBlinkTask(void *pvParameters) {
  TaskHandle_t *xTaskToNotify = pvParameters;
  //printf("LEDBlinkTask()");
  //gpio_enable(2,GPIO_OUTPUT);
  gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
  int level = 0;
  // Notify  the sending task that we are ready
  printf("LEDBlinkTask - Notifying parent task that we are ready\n");
  xTaskNotifyGive(*xTaskToNotify);
  while (true) {
    gpio_set_level(GPIO_NUM_4, level);
    level = !level;
    printf("MinEverHeapSize=%d\n", xPortGetMinimumEverFreeHeapSize());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void i2cScanTask(void *pvParameters) {

  while(1) {
    ADXL345_IVector v = ADXL345_readRaw();
    printf("(%4d, %4d, %4d)\n",v.XAxis,v.YAxis,v.ZAxis);
    vTaskDelay(1000 / portTICK_RATE_MS);
  } 
}





void app_main(void)
{
  esp_err_t ret;
  static TaskHandle_t xTaskToNotify = NULL;
  xTaskToNotify = xTaskGetCurrentTaskHandle();

  printf("Initialise Non-Volatile Storage (needed fro BLE)\n");
  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );
  printf("nvs_init: MinEverHeapSize=%d\n", xPortGetMinimumEverFreeHeapSize());

  /* First Start the I2C task so that other tasks can access the bus */
  printf("Starting i2cTask....\n");
  i2cConfig_t conf;
  conf.port = I2C_NUM_0;
  conf.sda = (gpio_num_t)SDA_PIN;
  conf.scl = (gpio_num_t)SCL_PIN;
  conf.clkSpeed = 100000;
  conf.gpio_pullup = true;
  conf.xTaskToNotify = xTaskToNotify;
  xTaskCreate(i2cTask,"i2cTask",8000,&conf,2,NULL);
  printf("i2cTask: MinEverHeapSize=%d\n",
	 xPortGetMinimumEverFreeHeapSize());

  printf("Waiting (for up to 1 sec) for i2c to establish before starting other tasks....\n");
  ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS( 1000UL ));
  
  printf("Starting displayTask\n");
  xTaskCreate(displayTask,"displayTask",8000,&xTaskToNotify,2,NULL);
  printf("Waiting (for up to 10 sec) for display to initialise before starting other tasks....\n");
  ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS( 10000UL ));
  printf("displayTask: MinEverHeapSize=%d\n",
	 xPortGetMinimumEverFreeHeapSize());


  printf("Starting Blink Task...\n");
  xTaskCreate(LEDBlinkTask,"Blink",8000,&xTaskToNotify,2,NULL);
  printf("Waiting (for up to 10 sec) for task to initialise before starting other tasks....\n");
  ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS( 10000UL ));
  printf("MinEverHeapSize=%d\n", xPortGetMinimumEverFreeHeapSize());

  //xTaskCreate(i2cScanTask,"i2cScanTask",8000,NULL,2,NULL);

  printf("Starting accelTask...\n");
  xTaskCreate(accelTask,"accelTask",8000,&xTaskToNotify,2,NULL);
  printf("Waiting (for up to 10 sec) for task to initialise before starting other tasks....\n");
  ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS( 10000UL ));
  printf("accelTask: MinEverHeapSize=%d\n", xPortGetMinimumEverFreeHeapSize());
  printf("startting monitorTask\n");
  xTaskCreate(monitorAdxl345Task,"monitorAdxl345Task",8000,&xTaskToNotify,2,NULL);
  printf("Waiting (for up to 10 sec) for task to initialise before starting other tasks....\n");
  ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS( 10000UL ));
  printf("monitorTask: MinEverHeapSize=%d\n", xPortGetMinimumEverFreeHeapSize());
 


  printf("Starting heartTask...\n");
  xTaskCreate(heartTask,"heartTask",8000,&xTaskToNotify,2,NULL);
  printf("Waiting (for up to 10 sec) for task to initialise before starting other tasks....\n");
  ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS( 10000UL ));
  printf("heartTask: MinEverHeapSize=%d\n", xPortGetMinimumEverFreeHeapSize());

  printf("Starting commsTask...\n");
  xTaskCreate(commsTask,"commsTask",8000,&xTaskToNotify,2,NULL);
  printf("Waiting (for up to 10 sec) for task to initialise before starting other tasks....\n");
  ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS( 10000UL ));
  printf("commsTask: MinEverHeapSize=%d\n", xPortGetMinimumEverFreeHeapSize());

  //runi2cTaskTest((gpio_num_t)SDA_PIN,(gpio_num_t)SCL_PIN);


}

