#include "FreeRTOS.h"
#include "task.h"
#include "user_tasks.h"
#include "am2302.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include "dcts.h"
#include "buttons.h"
#include "stm32f1xx_hal_adc.h"

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
            
            
            SSD1306_GotoXY(1, 27);
            SSD1306_Puts("AM3202 LOST", &Font_11x18, OLED_TEXT);
            SSD1306_GotoXY(1, 45);
            SSD1306_Puts("  CONNECT  ", &Font_11x18, OLED_TEXT);
            taskEXIT_CRITICAL();
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
            taskENTER_CRITICAL();
            SSD1306_GotoXY(1, 27);
            SSD1306_Puts(buff, &Font_11x18, OLED_TEXT);
            taskEXIT_CRITICAL();
            sprintf(buff, "  %s %.1f%s", meas[1].name, hum, meas[1].unit);
            buff_len = strlen(buff);
            if (buff_len < MAX_STR_LEN_11x18){
                for(buff_len; buff_len < MAX_STR_LEN_11x18; buff_len++){
                    buff[buff_len] = ' ';
                }
            }
            taskENTER_CRITICAL();
            SSD1306_GotoXY(1, 45);
            SSD1306_Puts(buff, &Font_11x18, OLED_TEXT);
            taskEXIT_CRITICAL();
        }
        //SSD1306_UpdateScreen();
        HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);
        vTaskDelayUntil( &next_wake_time, AM2302_TASK_PERIOD);
    }
}

void ssd1306_task (const void *pvParameters){
    TickType_t next_wake_time;
    next_wake_time = xTaskGetTickCount();
    uint8_t display_state = display_on;
    taskENTER_CRITICAL();
    if(SSD1306_Init()) {
        SSD1306_Fill(OLED_FON);
        SSD1306_GotoXY(1, 2);
        SSD1306_Puts("  Домашняя", &Font_11x18, OLED_TEXT);
        SSD1306_GotoXY(1, 20);
        SSD1306_Puts("   версия", &Font_11x18, OLED_TEXT);
        SSD1306_GotoXY(1, 38);
        SSD1306_Puts(" термометра", &Font_11x18, OLED_TEXT);
        SSD1306_UpdateScreen();
        vTaskDelay(2000);
        SSD1306_Fill(OLED_FON);
    }
    taskEXIT_CRITICAL();
    while(1){
        if(display_state == display_on){
            taskENTER_CRITICAL();
            SSD1306_UpdateScreen();
            taskEXIT_CRITICAL();
            if(break_pressed >= LONG_PRESS){
                display_state = display_off;
                SSD1306_Fill(OLED_FON);
                SSD1306_UpdateScreen();
            }
        }else if((left_pressed >= SHORT_PRESS)||(right_pressed >= SHORT_PRESS)||(up_pressed >= SHORT_PRESS)||
                 (down_pressed >= SHORT_PRESS)||(ok_pressed >= SHORT_PRESS)||(set_pressed >= SHORT_PRESS)){
            display_state = display_on;
        }
        
        vTaskDelayUntil( &next_wake_time, SSD1306_TASK_PERIOD);
    } 
}

void rtc_task (const void *pvParameters){
    TickType_t next_wake_time;
    next_wake_time = xTaskGetTickCount();
    uint8_t hour, min, sec;
    //char buff[18];
    
    RTC_HandleTypeDef hrtc;
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    
    while(1){
        //SSD1306_DrawFilledRectangle(1, 26, 128, 1, OLED_FON);
        HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
        hour = sTime.Hours;
        min = sTime.Minutes;
        sec = sTime.Seconds;
        
        /*sprintf(buff, "  %2.0f:%2.0f:%2.0f", hour, min, sec);
        taskENTER_CRITICAL();
        SSD1306_GotoXY(1, 5);
        SSD1306_Puts(buff, &Font_11x18, OLED_TEXT);
        taskEXIT_CRITICAL();*/
        vTaskDelayUntil( &next_wake_time, RTC_TASK_PERIOD);
    }
}

void buttons_task (const void *pvParameters){
    TickType_t next_wake_time;
    next_wake_time = xTaskGetTickCount();
    uint16_t up_pressed = 0;
    uint16_t down_pressed = 0;
    uint16_t left_pressed = 0;
    uint16_t right_pressed = 0;
    uint16_t ok_pressed = 0;
    uint16_t set_pressed = 0;
    uint16_t break_pressed = 0;
    while(1){
        
        if(HAL_GPIO_ReadPin(LEFT_PORT, LEFT_PIN)){
            left_pressed += BUTTONS_TASK_PERIOD;
        } else {
            left_pressed = 0;
        }
        
        if(HAL_GPIO_ReadPin(RIGHT_PORT, RIGHT_PIN)){
            right_pressed += BUTTONS_TASK_PERIOD;
        } else {
            right_pressed = 0;
        }
        
        if(HAL_GPIO_ReadPin(UP_PORT, UP_PIN)){
            up_pressed += BUTTONS_TASK_PERIOD;
        } else {
            up_pressed = 0;
        }
        
        if(HAL_GPIO_ReadPin(DOWN_PORT, DOWN_PIN)){
            down_pressed += BUTTONS_TASK_PERIOD;
        } else {
            down_pressed = 0;
        }
        
        if(HAL_GPIO_ReadPin(OK_PORT, OK_PIN)){
            ok_pressed += BUTTONS_TASK_PERIOD;
        } else {
            ok_pressed = 0;
        }
        
        if(HAL_GPIO_ReadPin(SET_PORT, SET_PIN)){
            set_pressed += BUTTONS_TASK_PERIOD;
        } else {
            set_pressed = 0;
        }
        
        if(HAL_GPIO_ReadPin(BREAK_PORT, BREAK_PIN)){
            break_pressed += BUTTONS_TASK_PERIOD;
        } else {
            break_pressed = 0;
        }
        
        vTaskDelayUntil( &next_wake_time, BUTTONS_TASK_PERIOD);
    }
}

void ntc_task (const void *pvParameters){
#define ADC_VREF    3300
#define ADC_RANGE   4095
#define ADC_POL_A1  -0.0000000024
#define ADC_POL_A2  0.0000162934
#define ADC_POL_A3  -0.0569517672
#define ADC_POL_A4  83.7182793156
    ADC_HandleTypeDef hadc1;
    TickType_t next_wake_time;
    next_wake_time = xTaskGetTickCount();
    uint16_t ntc_v_adc;
    float ntc_tmpr;
    while(1){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1,100);
        ntc_v_adc = (uint16_t) HAL_ADC_GetValue(&hadc1); 
        ntc_tmpr = ADC_POL_A1 * ntc_v_adc * ntc_v_adc * ntc_v_adc + ADC_POL_A2 * ntc_v_adc * ntc_v_adc +
            ADC_POL_A3 * ntc_v_adc + ADC_POL_A4;
        vTaskDelayUntil( &next_wake_time, NTC_TASK_PERIOD);
    }
}