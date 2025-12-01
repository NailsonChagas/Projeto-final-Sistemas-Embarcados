#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
	uint16_t duty_cycle_cvt;

    float kp;
    float ki;
    float kd;

    float e_prev;   // e(k-1)
    float ui_prev;  // uI(k-1)
} PID_Controller;

void pid_init(PID_Controller *pid, float kp, float ki, float kd, uint16_t time_counter, uint8_t max_input);
uint16_t pid_compute(PID_Controller *pid, float *reference, float *measurement, uint16_t *out);

#endif
