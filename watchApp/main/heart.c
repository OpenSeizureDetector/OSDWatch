#include "osd_app.h"
#include "max30102.h"
#include "freertos/FreeRTOS.h"

#include "displayTask.h"

void heartTask(void *pvParam) {
  char rowStr[32];
  printf("heartTask");
  MAX30100_startup();
  MAX30100_begin(pw400,i40,sr50);
  printf("heartTask() - MAX30100_startup() complete");
  while (true) {
    MAX30100_readSensor();
    //printf("%ld, %ld\n", MAX30100_IR, MAX30100_RED);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    sprintf(rowStr,"IR  =%ld",MAX30100_IR);
    printf("%s\n",rowStr);
    displayTask_setRow(5,rowStr);
    sprintf(rowStr,"RED =%ld",MAX30100_RED);
    printf("%s\n",rowStr);
    displayTask_setRow(6,rowStr);

    sprintf(rowStr,"t1=%03d t2=%03d",MAX30100_temp_intg, MAX30100_temp_frac);
    printf("%s\n",rowStr);
    displayTask_setRow(7,rowStr);
    sprintf(rowStr,"t3=%03d",MAX30100_die_temp);
    printf("%s\n",rowStr);
    /*displayTask_setRow(8,rowStr);*/
  }
}

