#include "PID.h"

// Función que iniciliza el controlador
void PIDController_Init(PIDController_t *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;
	pid->i = 0;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

// Función que modificar los parámetros del controlador
void PIDController_SetParams(PIDController_t *pid, PID_Params_t *params){
	// Controller gains
	pid->Kp = params->Kp;
	pid->Ki = params->Ki;
	pid->Kd = params->Kd;
	
	// Derivative low-pass filter
	pid->tau = params->tau;
	
	// Output limits
	pid->limMin = params->limMin;
	pid->limMax = params->limMax;
	
	// Integrator limits
	pid->limMinInt = params->limMinInt;
	pid->limMaxInt = params->limMaxInt;
	
	// Sample time
	pid->T = params->T;
}

// Función que actualiza el estado del controlador
float PIDController_Update(PIDController_t *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;


	/*
	* Proportional
	*/
#ifndef PID_DISCRETO_PROPIO
    float proportional = pid->Kp * error;
#endif


	/*
	* Integral
	*/
#ifdef PID_PSEUDOCONTINUO
	pid->integrator = pid->Ki * pid->T * error + pid->integrator;
#endif
    
#ifndef PID_DISCRETO_PROPIO
	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }
#endif


	/*
	* Derivative (band-limited differentiator)
	*/
#ifdef PID_PSEUDOCONTINUO
	if (pid->i==0){
		pid->differentiator = -pid->Kd * (measurement - pid->prevMeasurement) / (pid->T * PID_DERV_PAS);
		pid->prevMeasurement = measurement;}
	pid->i = (pid->i+1)%PID_DERV_PAS;
#endif

	
	/*
	* Compute output and apply limits
	*/
#ifndef PID_DISCRETO_PROPIO
    pid->out = proportional + pid->integrator + pid->differentiator;
#endif
	
#ifdef PID_DISCRETO_PROPIO
	pid->out += (pid->Kp + pid->Ki * pid->T / 2.0 + pid->Kd / pid->T) * error;
	pid->out += (- pid->Kp + pid->Ki * pid->T / 2.0 - 2.0 * pid->Kd/ pid->T) * pid->prevError;
	pid->out += pid->Kd/pid->T * pid->prevError2;
#endif

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
#ifdef PID_DISCRETO_PROPIO
	pid->prevError2 	 = pid->prevError;
	pid->prevError       = error;
#endif
    // pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}

// Función que resetea el estado del controlador (Cuando no es detectada la esfera sobre la plataforma)
void PIDController_Reset(PIDController_t *pid){
#ifndef PID_DISCRETO_PROPIO
		pid->integrator = 0.0;
#endif
#ifdef PID_DISCRETO_PROPIO
		pid->out = 0.0;
#endif
}
