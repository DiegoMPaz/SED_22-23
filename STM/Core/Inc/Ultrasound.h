/*
 * Ultrasound.h
 *
 *  Created on: 18 jun. 2022
 *      Author: Diego
 */

#ifndef INC_ULTRASOUND_H_
#define INC_ULTRASOUND_H_

#include "stm32f4xx_hal.h"

void HCSR04_Init(TIM_HandleTypeDef Usound);

void HCSR04_Read (void);

void delayT2 (uint16_t delay);

#endif /* INC_ULTRASOUND_H_ */
