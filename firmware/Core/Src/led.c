/*
 * led.c
 *
 *  Created on: Dec 2, 2025
 *      Author: nailsonchagas
 */

#include "led.h"

void led_task(void *param)
{
	led_t *led = (led_t*) param;

	TickType_t timer;

	while (1) {
		timer = xTaskGetTickCount();
		HAL_GPIO_TogglePin(led->led_port, led->led_pin);
		vTaskDelayUntil(&timer, led->led_time);
	}
}
