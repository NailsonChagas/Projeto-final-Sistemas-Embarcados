/*
 * wave_table.h
 *
 *  Created on: Dec 1, 2025
 *      Author: nailsonchagas
 */

#ifndef INC_WAVE_TABLE_H_
#define INC_WAVE_TABLE_H_

#include "cmsis_os.h"

#define N_POINTS 100

typedef enum {
    WAVE_SQUARE = 0,
    WAVE_SINE,
    WAVE_RECTIFIED_SINE,
    WAVE_TRIANGLE,
	WAVE_MAX
} WaveformType;

typedef struct {
	WaveformType type;   // qual onda está ativa
    uint16_t index;         // posição atual dentro do array (0–99)
} WaveformCtrl;

void waveform_init(WaveformCtrl *ctrl);
void waveform_get_sample(WaveformCtrl *ctrl, float *out);
void waveform_next_wave();

#endif /* INC_WAVE_TABLE_H_ */
