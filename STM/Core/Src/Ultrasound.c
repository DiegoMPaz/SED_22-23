/*
 * Ultrasound.c
 *
 *  Created on: 18 nov. 2022
 *      Author: Diego
 */

#include "Ultrasound.h"

TIM_HandleTypeDef htim2;

#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT GPIOA


void delayT2 (uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER (&htim2) < delay);
}


void HCSR04_Init(TIM_HandleTypeDef Usound){
	htim2 = Usound;
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}


void HCSR04_Read (void){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delayT2(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);

}
