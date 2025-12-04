/*
 * datalogger.h
 *
 *  Created on: Dec 4, 2025
 *      Author: nailsonchagas
 */

#ifndef INC_DATALOGGER_H_
#define INC_DATALOGGER_H_

#include <stdint.h>
#include <string.h>
#include "stm32g4xx_hal.h"
#include "cmsis_os.h"

typedef void (*datalogger_fetch_cb_t)(uint8_t **data, size_t *size);

typedef struct {
	UART_HandleTypeDef *uart_handler;
    uint8_t  *tx_dma_buffer;
    size_t   max_payload_size;
    TickType_t  log_interval_ms;
    datalogger_fetch_cb_t fetch_cb;   // callback de aquisição
} Datalogger;

void datalogger_init(
		Datalogger *datalogger,
		UART_HandleTypeDef *huart,
		size_t max_payload_size,
		TickType_t log_interval_ms,
		datalogger_fetch_cb_t fetch_cb
);
void datalogger_task(void *param);

#endif /* INC_DATALOGGER_H_ */
