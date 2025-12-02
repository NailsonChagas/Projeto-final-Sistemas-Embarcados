/*
 * led.h
 *
 *  Created on: Dec 2, 2025
 *      Author: nailsonchagas
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32g4xx_hal.h"
#include "cmsis_os.h"

typedef struct led_t_ {
	TickType_t led_time;
	uint16_t led_pin;
	GPIO_TypeDef *led_port;
} led_t;

void led_task(void *param);

#endif /* INC_LED_H_ */
