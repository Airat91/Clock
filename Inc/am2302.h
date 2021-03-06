// Library for AM2302
// Ver_1.0

/*========== LIBRARY DESCRIPTION ==========
- Library use STM32F3xx_HAL_Driver 
*/

#ifndef am2302_H_
#define am2302_H_
#include "stm32f1xx_hal.h"

typedef struct {
  int16_t hum;
  int16_t tmpr;
  uint8_t paritet;
} am2302_data_t;

//========== VARIABLES ==========

volatile extern uint8_t am2302_timeout;

//========== FUNCTIONS PROTOTYPES ==========

void am2302_init (void);
am2302_data_t am2302_get (void);

#endif /* am2302_H_ */