#include "buttons.h"

void buttons_init(void){
    GPIO_InitTypeDef GPIO_InitStruct;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    GPIO_InitStruct.Pin = COM_PIN;
    HAL_GPIO_Init(COM_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    
    GPIO_InitStruct.Pin = LEFT_PIN;
    HAL_GPIO_Init(LEFT_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = RIGHT_PIN;
    HAL_GPIO_Init(RIGHT_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = UP_PIN;
    HAL_GPIO_Init(UP_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DOWN_PIN;
    HAL_GPIO_Init(DOWN_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = OK_PIN;
    HAL_GPIO_Init(OK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SET_PIN;
    HAL_GPIO_Init(SET_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = BREAK_PIN;
    HAL_GPIO_Init(BREAK_PORT, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(COM_PORT, COM_PIN, GPIO_PIN_SET);
}