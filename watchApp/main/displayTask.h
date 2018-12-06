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


typedef struct {
  int cmd;
  char val[256];
} msg_t;

typedef enum {
  DISPLAY_MODE_TIME,
  DISPLAY_MODE_ACC,
  DISPLAY_MODE_HR,
  DISPLAY_MODE_SD
} displayMode_t;


void displayTask_setRow(int nrow, char *txt);
void displayTask_updateDisplay();
void displayTask(void *pvParameters);
