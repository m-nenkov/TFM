#ifndef APLICACION_H
#define APLICACION_H

#include "drivers.h"
#include "PID.h"
#include "LQR.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "config.h"

// Definición de tareas del OS
#ifdef DATOS_CAM
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif
void ControlTask(void *argument);
void Toma_DatosTask(void *argument);
void Led_EstadoTask(void *argument);
void Envio_DatosTask(void *argument);
void Recepcion_DatosTask(void *argument);


#endif
