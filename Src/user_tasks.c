#include "FreeRTOS.h"
#include "task.h"
#include "user_tasks.h"
#include "am2302.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include "dcts.h"

void am2302_task (const void *pvParameters){
    TickType_t next_wake_time;
    next_wake_time = xTaskGetTickCount();
    am2302_data_t hum_tmpr;
    char buff[18];
    float tmpr = 0;
    float hum = 0;
    uint8_t buff_len;
    
    am2302_init();
    
    while(1){
        //taskENTER_CRITICAL();
        SSD1306_DrawFilledRectangle(1, 1, 128, 46, OLED_FON);
        //SSD1306_UpdateScreen();
        //SSD1306_Fill(OLED_FON);
        //SSD1306_UpdateScreen();
        hum_tmpr = am2302_get ();
        //taskEXIT_CRITICAL();
        if ((hum_tmpr.hum == 0) && (hum_tmpr.tmpr == 0)) {
            taskENTER_CRITICAL();
            meas[0].value = 0;
            meas[1].value = 0;
            taskEXIT_CRITICAL();
            
            SSD1306_GotoXY(1, 3);
            SSD1306_Puts("AM3202 LOST", &Font_11x18, OLED_TEXT);
            SSD1306_GotoXY(1, 28);
            SSD1306_Puts("  CONNECT  ", &Font_11x18, OLED_TEXT);
        } else {
            taskENTER_CRITICAL();
            meas[0].value = hum_tmpr.tmpr;
            meas[1].value = hum_tmpr.hum;
            taskEXIT_CRITICAL();
            
            tmpr = (float)hum_tmpr.tmpr/10;
            hum = (float)hum_tmpr.hum/10;
            
            sprintf(buff, "  %s %.1f%s", meas[0].name, tmpr, meas[0].unit);
            buff_len = strlen(buff);
            if (buff_len < MAX_STR_LEN_11x18){
                for(buff_len; buff_len < MAX_STR_LEN_11x18; buff_len++){
                    buff[buff_len] = ' ';
                }
            }
            SSD1306_GotoXY(1, 3);
            SSD1306_Puts(buff, &Font_11x18, OLED_TEXT);
            
            sprintf(buff, "  %s %.1f%s", meas[1].name, hum, meas[1].unit);
            buff_len = strlen(buff);
            if (buff_len < MAX_STR_LEN_11x18){
                for(buff_len; buff_len < MAX_STR_LEN_11x18; buff_len++){
                    buff[buff_len] = ' ';
                }
            }
            SSD1306_GotoXY(1, 28);
            SSD1306_Puts(buff, &Font_11x18, OLED_TEXT);
        }
        //SSD1306_UpdateScreen();
        HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);
        vTaskDelayUntil( &next_wake_time, AM2302_TASK_PERIOD);
    }
}

void ssd1306_task (const void *pvParameters){
    TickType_t next_wake_time;
    next_wake_time = xTaskGetTickCount();
    taskENTER_CRITICAL();
    if(SSD1306_Init()) {
        SSD1306_Fill(OLED_FON);
        SSD1306_GotoXY(1, 2);
        SSD1306_Puts("àáâãäåæçèéê", &Font_11x18, OLED_TEXT);
        SSD1306_GotoXY(1, 20);
        SSD1306_Puts("ëìíîïðñòóôõ", &Font_11x18, OLED_TEXT);
        SSD1306_GotoXY(1, 38);
        SSD1306_Puts("ö÷øùúûüýþÿ", &Font_11x18, OLED_TEXT);
        SSD1306_UpdateScreen();
    }
    taskEXIT_CRITICAL();
    while(1){
//        taskENTER_CRITICAL();
        SSD1306_UpdateScreen();
//        taskEXIT_CRITICAL();
        vTaskDelayUntil( &next_wake_time, SSD1306_TASK_PERIOD);
    } 
}

void rtc_task (const void *pvParameters){
    TickType_t next_wake_time;
    next_wake_time = xTaskGetTickCount();
    uint8_t hour, min, sec;
    
    RTC_HandleTypeDef hrtc;
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    
    while(1){
        SSD1306_DrawFilledRectangle(1, 47, 128, 17, OLED_FON);
        HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
        hour = sTime.Hours;
        min = sTime.Minutes;
        sec = sTime.Seconds;
        
        vTaskDelayUntil( &next_wake_time, RTC_TASK_PERIOD);
    }
}