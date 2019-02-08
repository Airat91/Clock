#include "dcts.h"
#include "String.h"
#include "FreeRTOS.h"
#include "task.h"

/*========== GLOBAL VARIABLES ==========*/

const uint8_t   id = 0x07;
const char      ver[11] = "1.0";
const char      name[11] = "Portable";
uint8_t         address = 0xFF;
rtc_t rtc = {
    .day = 0, 
    .month = 0, 
    .year = 2000, 
    .hour = 0, 
    .minute = 0, 
    .second = 0
};
int16_t pwr = 0;

meas_t meas[2];

void dcts_init () {
    strcpy (meas[0].name, "T");
    strcpy (meas[0].unit, "АC");
    meas[0].value = 0;
    
    strcpy (meas[1].name, "Ты");
    strcpy (meas[1].unit, "%");
    meas[1].value = 0;
}

void dcts_write_meas_value (uint8_t meas_channel, int32_t value){
    taskENTER_CRITICAL();
    meas[meas_channel].value = value;
    taskEXIT_CRITICAL();
}