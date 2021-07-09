#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// Selección del controlador PID
#define PID_PSEUDOCONTINUO
// #define PID_DISCRETO_PROPIO

// Modificación del periodo de cálculo de la derivada
#define PID_DERV_PAS 10 // N veces más lenta que la parte proporcional

typedef struct PIDController{

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;
	int i;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
#ifdef PID_DISCRETO_PROPIO
	float prevError2;
#endif
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController_t;

typedef struct PID_Params{
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;
}PID_Params_t;

void PIDController_Init(PIDController_t *pid);
void PIDController_SetParams(PIDController_t *pid, PID_Params_t *params);
float PIDController_Update(PIDController_t *pid, float setpoint, float measurement);
void PIDController_Reset(PIDController_t *pid);

#endif
