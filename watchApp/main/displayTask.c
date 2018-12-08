/*
 * OSDWatch
 * Copyright Graham Jones, 2017, 2018.
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
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "osd_app.h"
#include "displayTask.h"
#include "ssd1306.h"

static QueueHandle_t displayQueue;

int displayMode;

#define NROW 8
#define NCOL 17
char displayText[NROW][NCOL];   // 8 rows of 16 characters

void displayTask_updateDisplay() {
  char txtStr[256];
  txtStr[0]=0;
  for (int irow=0;irow<NROW;irow++) {
    strncat(txtStr,&displayText[irow][0],NCOL);
    txtStr[strlen(txtStr)+1] = '\0';   
    txtStr[strlen(txtStr)] = '\n';   // overwrite the null termination
  }
  //printf("displayTask_upateDisplay: len(txtStr)=%d, txtStr=\n%s\n",
  // strlen(txtStr),txtStr);
  //for (int irow=0;irow<NROW;irow++) {
  //  printf("displayText(%d)=%s\n", irow,&displayText[irow][0]);
  //  for (int icol=0;icol<NCOL;icol++)
  //    printf("%02x:%1c ",displayText[irow][icol],displayText[irow][icol]);
  //  printf("\n");
  //}

  //printf("txtStr=");
  //for (int i=0;i<256;i++)
  //  printf("%02x:%c ",txtStr[i],txtStr[i]);
  //printf("\n");
  //SSD1306_clearDisplay();
  SSD1306_displayText(txtStr);
}

void displayTask_setRow(int nrow, char *txt) {
  strncpy(&displayText[nrow][0],txt,NCOL);
}


void displayTask(void *pvParameters) {
  msg_t *msg;
  TaskHandle_t *xTaskToNotify = pvParameters;

  printf("displayTask()");
  SSD1306_init();
  char bmp[1024];
  for(int c=0; c!=1024;c++)
    bmp[c] = 0b00000001;
  SSD1306_displayBitmap(bmp);
  SSD1306_clearDisplay();
  SSD1306_displayBitmap(bmp);
  SSD1306_clearDisplay();
  SSD1306_displayBitmap(bmp);
  SSD1306_clearDisplay();
  SSD1306_displayBitmap(bmp);
  SSD1306_clearDisplay();
  SSD1306_displayBitmap(bmp);
  SSD1306_clearDisplay();
  SSD1306_displayBitmap(bmp);
  SSD1306_clearDisplay();
  SSD1306_displayText(" Open     \n    Seizure    \n      Detector");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  SSD1306_clearDisplay();
  displayTask_setRow(0,"_____ OSD _____");
  displayTask_updateDisplay();
  //SSD1306_displayText("_____ OSD _____");


  //displayTask_setRow(0,"123456789abcdefg");
  //displayTask_setRow(1,"123456789abcdefg");
  //displayTask_setRow(2,"123456789abcdefg");
  //displayTask_setRow(3,"123456789abcdefg");
  //displayTask_setRow(4,"123456789abcdefg");
  //displayTask_setRow(5,"123456789abcdefg");
  //displayTask_setRow(6,"123456789abcdefg");
  //displayTask_setRow(7,"123456789abcdefg");
  //displayTask_updateDisplay();

  displayMode = DISPLAY_MODE_ACC;
  // Create the transaction queue - a queue of pointers
  displayQueue = xQueueCreate(QUEUE_SIZE,sizeof(char *));

  // Notify  the sending task that we are ready to receive i2c requests
  printf("displayTask - Notifying parent task that we are ready\n");
  xTaskNotifyGive(*xTaskToNotify);


  // Loop continuously, waiting for transaction requests
  printf("displayTask - queue created - waiting for transactions on queue\n");
  while(1) {
    if(xQueueReceive(displayQueue,&msg,portMAX_DELAY) == pdTRUE)
      printf("displayTask() - Message Received: cmd=%d, val=%s\n",
           msg->cmd, msg->val);
    if ((msg->cmd == DISPLAY_MODE_TIME)
	|| (msg->cmd == DISPLAY_MODE_ACC)
	|| (msg->cmd == DISPLAY_MODE_HR)
	|| (msg->cmd == DISPLAY_MODE_SD)
	) {
      displayMode = msg->cmd;
      displayTask_updateDisplay();
    }
    else {
      fprintf(stderr,"displayTask - ERROR - Unrecognised Command %d\n", msg->cmd);
    }
  }
}

void displaySetMode(int mode) {

}
