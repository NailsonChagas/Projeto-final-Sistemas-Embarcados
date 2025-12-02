#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

typedef struct {
	float duty_cycle_cvt;
	uint8_t max_input;

    float kp;
    float ki;
    float kd;

    volatile float e_prev;   // e(k-1)
    volatile float ui_prev;  // uI(k-1)
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd, uint16_t time_counter, uint8_t max_input);
void pid_compute(PIDController *pid, volatile float *reference, volatile float *measurement, volatile uint16_t *out);
void pid_reset(PIDController *pid);

#endif
