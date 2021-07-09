#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

// Modificación del periodo de cálculo de la derivada
#define LQR_DERV_PAS 10		// N veces más lenta que la parte proporcional

typedef struct SS_System{
	float *R, *U;
	float *X, *K;
	float kr, prevX[2];

	int i, dim_R[2], dim_X[2], dim_K[2];
	float sample_time_s;
}SS_System_t;

typedef struct SS_System_Params{
	float kr;
	int dim_R[2];
	int dim_X[2];
	float sample_time_s;
}SS_System_Params_t;

void SS_Init(SS_System_t *this, SS_System_Params_t *params);
void SS_Set_Mats(SS_System_t *this, float *r, float *u, float *k, float *x);
void SS_Update_Control(SS_System_t *this);
void SS_Estim_States(SS_System_t *this, float *dist);
void SS_Reset(SS_System_t *this);

#endif
