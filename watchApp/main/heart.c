#include "osd_app.h"
#include "max30102.h"
#include "freertos/FreeRTOS.h"


void heartTask(void *pvParam) {
  printf("heartTask");
  MAX30100_startup();
  MAX30100_begin(pw400,i40,sr50);
  printf("heartTask() - MAX30100_startup() complete");
  while (true) {
    MAX30100_readSensor();
    printf("%ld, %ld\n", MAX30100_IR, MAX30100_RED);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

