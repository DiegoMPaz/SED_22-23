/*
 * T_Sensor.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Diego
 */

#ifndef INC_T_SENSOR_H_
#define INC_T_SENSOR_H_

#include "stm32f4xx_hal.h"

uint8_t DHT11_Start (void);

uint8_t DHT11_Read (void);

#endif /* INC_T_SENSOR_H_ */
