#include "LQR.h"
#include "math.h"
#include <stdlib.h>

// Función que inicializa el espacio de estados
void SS_Init(SS_System_t *this, SS_System_Params_t *params){
	// Se guarda el tiempo de muestreo
	this->sample_time_s = params->sample_time_s;

	// Se almacena la constante de corrección de la salida
	this->kr = params->kr;
	this->i  = 0;
	this->prevX[0] = 0.0; this->prevX[1] = 0.0;
	
	// Se almacenan las dimensiones de las matrices conocidas
	this->dim_X[0] = params->dim_X[0]; this->dim_X[1] = params->dim_X[1];
	this->dim_R[0] = params->dim_R[0]; this->dim_R[1] = params->dim_R[1];
	
	// Cálculo de la dimensión de K
	this->dim_K[0] = this->dim_R[0];
	this->dim_K[1] = this->dim_X[0];
}

// Función que inicializa las matrices del espacio de estados
void SS_Set_Mats(SS_System_t *this, float *r, float *u, float *k, float *x){
	// Se almacenan las matrices
	this->R = r;
	this->U = u;
	this->X = x;
	this->K = k;
}

// Función que actualiza la acción de control
void SS_Update_Control(SS_System_t *this){
	int i, j, k;
	
	// Cálculo de U (Acción del control)
	for (i=0; i<this->dim_R[0]; i++){
		for (j=0; j<this->dim_R[1]; j++){
			// U = R * kr
			this->U[i*this->dim_R[1]+j] = this->R[i*this->dim_R[1]+j]*this->kr;
			// U -= K*X
			for (k=0; k<this->dim_K[1]; k++){
				this->U[i*this->dim_R[1]+j] -= this->K[i*this->dim_K[1]+k]*this->X[k*this->dim_X[1]+j];
			}
		}
	}
}

// Función que actualiza el estado del sistema a partir de la observación de la posición de la esfera
void SS_Estim_States(SS_System_t *this, float *dist){
	// Se estima el estado del sistema
	if (this->i == 0){
		this->X[1] = (dist[0] - this->prevX[0]) / (this->sample_time_s * LQR_DERV_PAS); // X[1] += 7.0 * ang_obj_X * SAMPLE_TIME_S;
		this->prevX[0] = dist[0];
		
		this->X[3] = (dist[1] - this->prevX[1]) / (this->sample_time_s * LQR_DERV_PAS); // X[1] += 7.0 * ang_obj_X * SAMPLE_TIME_S;
		this->prevX[1] = dist[1];
	}
	this->i = (this->i+1)%LQR_DERV_PAS;
	
	this->X[0] = dist[0];
	this->X[2] = dist[1];
}
