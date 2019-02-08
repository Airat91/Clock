#include "stdint.h"
#include "stm32f1xx_hal_gpio.h"

#define LEFT_PIN    GPIO_PIN_15
#define LEFT_PORT   GPIOA
#define RIGHT_PIN   GPIO_PIN_3
#define RIGHT_PORT  GPIOB
#define UP_PIN      GPIO_PIN_4
#define UP_PORT     GPIOB
#define DOWN_PIN    GPIO_PIN_5
#define DOWN_PORT   GPIOB
#define OK_PIN      GPIO_PIN_6
#define OK_PORT     GPIOB
#define SET_PIN     GPIO_PIN_7
#define SET_PORT    GPIOB
#define BREAK_PIN   GPIO_PIN_8
#define BREAK_PORT  GPIOB
#define COM_PIN     GPIO_PIN_12
#define COM_PORT    GPIOA

#define SHORT_PRESS 50
#define LONG_PRESS  3000

void buttons_init(void);

extern uint16_t up_pressed;
extern uint16_t down_pressed;
extern uint16_t left_pressed;
extern uint16_t right_pressed;
extern uint16_t ok_pressed;
extern uint16_t set_pressed;
extern uint16_t break_pressed;
