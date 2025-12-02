#include "pid.h"

void pid_init(PIDController *pid, float kp, float ki, float kd, uint16_t time_counter, uint8_t max_input)
{
	/*
	 * Controlador PID
	 * uP(k) = Kp*e(k)
	 * uI(k) = uI(k-1) + Ki*e(k)
	 * uD(k) = Kd*(e(k) - e(k-1))
	 * uPID = uP(k) + uI(k) + uD(k)
	 **/
	pid->duty_cycle_cvt = (float)time_counter / max_input;
	pid->max_input = max_input;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->e_prev = 0.0f;
    pid->ui_prev = 0.0f;
}

void pid_compute(PIDController *pid, volatile float *reference, volatile float *measurement, volatile uint16_t *out)
{
    float e = *reference - *measurement;
    float ui = pid->ui_prev + pid->ki * e;

    float pid_output = (pid->kp * e) + ui + (pid->kd * (e - pid->e_prev));

    if(pid_output < 0)
    {
    	pid_output = 0;
    }
    else if(pid_output > pid->max_input){
    	pid_output = pid->max_input;
    }

    pid->e_prev = e;
    pid->ui_prev = ui;

    *out = (uint16_t)(pid_output * pid->duty_cycle_cvt);
}

void pid_reset(PIDController *pid)
{
	/*
	 * Como a cada leitura do ADC é feito uma ação de controle
	 * estou bloqueando interrupção do ADC para impedir uma nova ação
	 * de controle enquanto é resetado
	 * */
	HAL_NVIC_DisableIRQ(ADC1_2_IRQn);

	pid->e_prev = 0.0f;
	pid->ui_prev = 0.0f;

	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}
