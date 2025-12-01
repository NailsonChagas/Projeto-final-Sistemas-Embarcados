/*
 * wave_table.h
 *
 *  Created on: Dec 1, 2025
 *      Author: nailsonchagas
 */

#ifndef INC_WAVE_TABLE_H_
#define INC_WAVE_TABLE_H_

#include "stm32g4xx_hal.h"

#define N_POINTS 100

typedef enum {
    WAVE_SQUARE = 0,
    WAVE_SINE,
    WAVE_RECTIFIED_SINE,
    WAVE_TRIANGLE,
	WAVE_MAX
} WaveformType;

typedef struct {
	volatile WaveformType type;   // qual onda está ativa
	volatile uint16_t index;      // posição atual dentro do array (0–99)
	volatile int8_t amplitude;
    uint8_t max_amplitude;
} WaveformCtrl;

void waveform_init(WaveformCtrl *ctrl, uint8_t max_amplitude);
void waveform_get_sample(WaveformCtrl *ctrl, volatile float *out);
void waveform_next_wave();
void waveform_update_amplitude(WaveformCtrl *ctrl, int16_t delta);

#endif /* INC_WAVE_TABLE_H_ */
