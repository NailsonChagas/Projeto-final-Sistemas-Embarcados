/*
 * datalogger.c
 *
 *  Created on: Dec 4, 2025
 *      Author: nailsonchagas
 */

#include "datalogger.h"

void datalogger_init(
		Datalogger *datalogger,
		UART_HandleTypeDef *huart,
		size_t max_payload_size,
		TickType_t log_interval_ms,
		datalogger_fetch_cb_t fetch_cb)
{
    datalogger->uart_handler = huart;

    // +2 para start ('$') e end ('\n')
    datalogger->max_payload_size = max_payload_size + 2;

    datalogger->tx_dma_buffer = (uint8_t*) pvPortMalloc(datalogger->max_payload_size);

    datalogger->log_interval_ms = log_interval_ms;

    datalogger->fetch_cb = fetch_cb;
}


static void datalogger_log(Datalogger *datalogger)
{
	/*
	 * [ '$' | n bytes de dados | '\n' | lixo (não utilizado)
	 */

    uint8_t *data_ptr = NULL;
    size_t data_size = 0;

    // chama a função fornecida pelo usuário
    datalogger->fetch_cb(&data_ptr, &data_size);

    size_t max_payload = datalogger->max_payload_size - 2;

    if (data_size > max_payload) {
        data_size = max_payload;
    }

    uint8_t *buffer = datalogger->tx_dma_buffer;

    buffer[0] = '$';
    memcpy(&buffer[1], data_ptr, data_size);
    buffer[1 + data_size] = '\n';

    HAL_UART_Transmit_DMA(
        datalogger->uart_handler,
        datalogger->tx_dma_buffer,
        datalogger->max_payload_size
    );
}


void datalogger_task(void *param)
{
    Datalogger *datalogger = (Datalogger*) param;
    TickType_t timer;

    while (1) {
        timer = xTaskGetTickCount();

        datalogger_log(datalogger);

        vTaskDelayUntil(&timer, datalogger->log_interval_ms);
    }
}

