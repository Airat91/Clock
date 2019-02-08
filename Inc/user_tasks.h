#ifndef USER_TASKS_H_
#define USER_TASKS_H_
#include "stm32f1xx_hal.h"

#define AM2302_TASK_PERIOD  2000
#define SSD1306_TASK_PERIOD 100
#define RTC_TASK_PERIOD     1000
#define BUTTONS_TASK_PERIOD 10
#define NTC_TASK_PERIOD     10000

void am2302_task (const void *pvParameters);
void ssd1306_task (const void *pvParameters);
void rtc_task (const void *pvParameters);
void buttons_task (const void *pvParameters);

typedef enum{
    display_on,
    display_off
}display_st;

#endif //USER_TASKS_H_