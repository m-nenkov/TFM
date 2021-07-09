#include "aplicacion.h"

// Variables globales
float dist_X = 0, dist_Y = 0;		// Variables para la comunicación de la distancia de la esfera al centro de la plataforma
float setpoint_X = 0, setpoint_Y = 0;	// Varibales para la comunicación del valor de la consigna ( o refencia) a la que se desea llevar la esfera
uint8_t control_activ = 0; 		// Variables indicativa de si el control se encuetra activo o no
uint8_t pData_Cam[2];		// Datos recibidos de la cámara
int8_t pData_Setpoint[2];	// Variable auxiliar para el control de la consigna
uint8_t cambio_modo_setpoint = 0; // Varibale para la gestión del cambio entre los modos de respuesta del sitema (cero, rectángulo e infinito).

// Gestión de la comunicación UART con DMA
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
#ifdef RECEPCION_DATOS
	extern UART_HandleTypeDef huart2;
	extern osThreadId_t recpecion_datosHandle;
#endif

#ifdef DATOS_CAM
	extern UART_HandleTypeDef huart4;
	if (huart == &huart4){
		HAL_UART_Receive_DMA(huart, (uint8_t *)pData_Cam, (uint16_t) sizeof(pData_Cam));
	}
#endif
#ifdef RECEPCION_DATOS
	if (huart == &huart2){
		HAL_UART_Receive_DMA(huart, (uint8_t *)pData_Setpoint, (uint16_t) sizeof(pData_Setpoint));
		osThreadFlagsSet(recpecion_datosHandle, FLAG_ID);
	}
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
}


#ifndef RECEPCION_DATOS
// Gestión de la recepción de comandos por UART
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == USER_BUTTON_PIN){
		if (HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN) ==	 GPIO_PIN_SET){
			cambio_modo_setpoint = 100;
		}
	}
}
#endif

// Tarea encargada del cálculo de la acción de control
void ControlTask(void *argument){
	// Variables externas
	extern TIM_HandleTypeDef htim1, htim2;
	extern osMutexId_t consignaMutexHandle, salidaMutexHandle, estado_controlMutexHandle, accion_controlXQueueHandle;
	extern osMessageQueueId_t distXQueueHandle;
	
	// Inicialización del controlador PWM (Para los servo-motores)
	PWM_Controller_t control_servoX, control_servoY;
	PWM_controller_init(&control_servoX, &htim1, TIM_CHANNEL_1, SERVO_TIMER_PERIOD, MAX_ANG, MIN_ANG);
	PWM_controller_init(&control_servoY, &htim2, TIM_CHANNEL_1, SERVO_TIMER_PERIOD, MAX_ANG, MIN_ANG);

	uint8_t data_estado;
	float dist[2], ang_obj_X=0, ang_obj_Y=0, ang_obj_servo;
	float setpointX = 0, setpointY = 0;

#ifdef CONTROL_PID
	// Inicialización del controlador PID
	PID_Params_t pid_params = { PID_KP, PID_KI, PID_KD,
								PID_TAU,
								PID_LIM_MIN, PID_LIM_MAX,
								PID_LIM_MIN_INT, PID_LIM_MAX_INT,
								SAMPLE_TIME_S };

	PIDController_t pidX, pidY;
	PIDController_Init(&pidX); PIDController_Init(&pidY);
	PIDController_SetParams(&pidX, &pid_params); PIDController_SetParams(&pidY, &pid_params);
#endif
#ifdef CONTROL_LQR
	// Inicialización del controlador LQR
	SS_System_t ss_sys;
	SS_System_Params_t ss_params;

	// Parámetros del controlador
	ss_params.dim_R[0] = DIM_R1; ss_params.dim_R[1] = DIM_R2;
	ss_params.dim_X[0] = DIM_X1; ss_params.dim_X[1] = DIM_X2;
	ss_params.kr = Kr;
	ss_params.sample_time_s = SAMPLE_TIME_S;

	SS_Init(&ss_sys, &ss_params);

	float X[DIM_X1*DIM_X2];
	float K[DIM_K1*DIM_K2] = MAT_K;
	float R[DIM_R1*DIM_R2];
	float U[DIM_R1*DIM_R2] = MAT_R;

	SS_Set_Mats(&ss_sys, (float *)R, (float *) U, (float *)K, (float*) X);
#endif
  /* Infinite loop */
	for(;;){

		// Lee si es necesario control (esfera en plataforma) o no (esfera no detectada);
		osMutexAcquire(estado_controlMutexHandle, osWaitForever);
		data_estado = control_activ;
		osMutexRelease(estado_controlMutexHandle);

		// Si se ha detectado la esfera...
		if (data_estado % 2){
			
			// Lee/accede al dato de la posición de la bola
			osMutexAcquire(salidaMutexHandle, osWaitForever);
			dist[0] = dist_X;
			dist[1] = dist_Y;
			osMutexRelease(salidaMutexHandle);

#ifdef DATOS_CAM
			// Corrección de la distorisón producida por el ángulo de la estructura en la cámara
			ajuste_pos_bola(&ang_obj_X, &dist[0]);
			ajuste_pos_bola(&ang_obj_Y, &dist[1]);
#endif

			// Se adquiere el mutex y actualiza los datos de la consigna
			osMutexAcquire(consignaMutexHandle, osWaitForever);
			setpointX = setpoint_X; setpointY = setpoint_Y;
			osMutexRelease(consignaMutexHandle);

			// Se cálcula la acción de control
#ifdef CONTROL_PID
			ang_obj_X = PIDController_Update(&pidX, setpointX, dist[0]);
			ang_obj_Y = PIDController_Update(&pidY, setpointY, dist[1]);
#endif
#ifdef CONTROL_LQR
			// Se define la consigna
			R[0] = setpointX; R[1] = setpointY;

			// Estimación de los estados
			SS_Estim_States(&ss_sys, dist);

			// Se obtiene la acción de control deseada
			SS_Update_Control(&ss_sys);

			ang_obj_X = U[0]; ang_obj_Y = U[1];

			// Se acota la acción de control calculada a los límites que permiten los servo-motores
			if (ang_obj_Y>MAX_ANG){
				ang_obj_Y = MAX_ANG;
			}
			else if(ang_obj_Y<MIN_ANG){
				ang_obj_Y = MIN_ANG;
			}

			if (ang_obj_X>MAX_ANG){
				ang_obj_X = MAX_ANG;
			}
			else if(ang_obj_X<MIN_ANG){
				ang_obj_X = MIN_ANG;
			}
#endif

#ifdef CALIBRACION_ESTRUCTURA
			ang_obj_X = 0;
			ang_obj_Y = 0;
#endif

#ifdef ENVIO_DATOS
			// Escritura de la acción de control para posterior envío de datos
	#ifdef ENVIO_X
			osMessageQueuePut(accion_controlXQueueHandle, &ang_obj_X, 1, osWaitForever);
			osMessageQueuePut(distXQueueHandle, &dist[0], 1, osWaitForever);
	#endif
	#ifdef ENVIO_Y
			osMessageQueuePut(accion_controlXQueueHandle, &ang_obj_Y, 1, osWaitForever);
			osMessageQueuePut(distXQueueHandle, &dist[1], 1, osWaitForever);
	#endif
	#ifdef ENVIO_XY
			osMessageQueuePut(accion_controlXQueueHandle, &dist[0], 1, osWaitForever);
			osMessageQueuePut(distXQueueHandle, &dist[1], 1, osWaitForever);
	#endif
#endif
		}
		else{ // Se envía a los controladores una señal para anular el error, setpoint = salida
#ifdef CONTROL_PID
			PIDController_Reset(&pidX);
			PIDController_Reset(&pidY);
#endif
#ifdef CONTROL_LQR
			X[1] = 0;
			X[3] = 0;

			SS_Update_Control(&ss_sys); // ang_obj_X = Ux[0];
#endif
		}
		
		// Se calcula el ángulo del servo y se modifica el estado de los actuadores
		calculo_ang_servo(&ang_obj_X, &ang_obj_servo);
		set_PWM(&control_servoX, -(ang_obj_servo - ERROR_ANG_X ));	// Eje X

		calculo_ang_servo(&ang_obj_Y, &ang_obj_servo);
		set_PWM(&control_servoY, (ang_obj_servo - ERROR_ANG_Y));	// Eje Y

		// Espera al siguiente ciclo de ejecución
		osThreadFlagsWait(FLAG_ID, osFlagsWaitAny, osWaitForever);
  }
}

// Lectura de los datos de los sensores
void Toma_DatosTask(void *argument){
	// Variables externas
	extern osMutexId_t salidaMutexHandle, estado_controlMutexHandle;
  
#ifdef DATOS_TFT
	extern I2C_HandleTypeDef hi2c1;

	// Inicialización del controlador ADC
	ADC_Converter_t adc_converter_X, adc_converter_Y;
	ADC_Conv_init(&adc_converter_X, MAX_ADC_VAL_TFT, LONG_X);
	ADC_Conv_init(&adc_converter_Y, MAX_ADC_VAL_TFT, LONG_Y);
	uint8_t pData[5] = {0, 0, 0, 0, 0};
	uint16_t adcvalue_X = 0, adcvalue_Y = 0;
#endif

#ifdef DATOS_CAM
	extern UART_HandleTypeDef huart4;

	// Inicialización del controlador ADC
	ADC_Converter_t pix_converter_X, pix_converter_Y;
	ADC_Conv_init(&pix_converter_X, PIXELS_X, LONG_X);
	ADC_Conv_init(&pix_converter_Y, PIXELS_Y, LONG_Y);
#endif

	// Variables locales
	float dist[2] = {0,0};
	uint8_t bola_detectada = 0;

  for(;;)
  {
	  // Comunicación con el sensor y lectura del dato en bruto
#ifdef DATOS_TFT
	  if (HAL_GPIO_ReadPin(GPIO_RTS_TFT, PIN_RTS_TFT)){
		  HAL_I2C_Master_Receive(&hi2c1, (uint16_t) ADDR_READ_TFT, (uint8_t *) pData, (uint16_t) 5 * sizeof(uint8_t), 100);
	  }
#endif

	  // Procesado del dato
#ifdef DATOS_TFT
	  bola_detectada = pData[0] % 2;
	  if (bola_detectada){
		  procesado_datos_TFT(&adcvalue_X, &adcvalue_Y, pData);
		  convADC2dist(&adc_converter_X, &dist[0], adcvalue_X);
		  convADC2dist(&adc_converter_Y, &dist[1], adcvalue_Y);
	  }
#endif

#ifdef DATOS_CAM
	  HAL_UART_Receive_DMA(&huart4, (uint8_t *)pData_Cam, (uint16_t) sizeof(pData_Cam));
	  bola_detectada = ( (pData_Cam[0]<PIXELS_X) && (pData_Cam[1]<PIXELS_X) )? 1:0; 	// La esfera es detectada si su centro se encuentra dentro de la imágen
	  if (bola_detectada){
		  convADC2dist(&pix_converter_X, &dist[0], pData_Cam[0]);
		  convADC2dist(&pix_converter_Y, &dist[1], pData_Cam[1]);
  	  }
#endif

	  // Actualización del estado de la esfera
	  osMutexAcquire(estado_controlMutexHandle, osWaitForever);
	  control_activ = bola_detectada;
	  osMutexRelease(estado_controlMutexHandle);

	  // Escritura de la medida en variable
	  osMutexAcquire(salidaMutexHandle, osWaitForever);
	  dist_X = dist[0];
	  dist_Y = dist[1];
	  osMutexRelease(salidaMutexHandle);

	  // Espera al siguiente ciclo de ejecución
	  osThreadFlagsWait(FLAG_ID, osFlagsWaitAny, osWaitForever);
  }
}

// Control del LED
void Led_EstadoTask(void *argument){
	// Variables externas
	extern osMutexId_t estado_controlMutexHandle;
	
	
	// Inicialización del controlador
	LED_Controller_t led_controller;
	LED_controller_init(&led_controller, LED_GPIO_Port, LED_GPIO_Pin);
	uint8_t data_estado = 0, aux_data=0;

  for(;;)
  {
	  // Lee/accede a la variable que indica bola detectada
	  osMutexAcquire(estado_controlMutexHandle, osWaitForever);
	  data_estado = control_activ;
	  osMutexRelease(estado_controlMutexHandle);

	  // Activa un led o desactiva el led si cambia el estado
	  if (data_estado != aux_data){
		  if (data_estado % 2){
			  encender_led(&led_controller);
		  }
		  else{
			  apagar_led(&led_controller);
		  }
	  }
	  else{
		  aux_data = data_estado;
	  }

	  // Espera al siguiente ciclo de ejecución
	  osThreadFlagsWait(FLAG_ID, osFlagsWaitAny, osWaitForever);
  }
}

// Envío de los datos muestreados y de la acción de control
void Envio_DatosTask(void *argument){
	// Variables externas
	extern UART_HandleTypeDef huart2;
	extern osMessageQueueId_t accion_controlXQueueHandle, distXQueueHandle;

#ifndef ENVIO_XY
	float ang_obj_f, dist_X_f;
#else
	float dist_X_f, dist_Y_f;
#endif
	int pos = 0, pos_enviada = 0;
	uint16_t datos[SIZE_SEND_DATA];

	for(;;)
	  {
		// Si la posición del buffer no coincide con la enviada
		if (pos != pos_enviada){
			// Se calculan los elementos que hay que enviar
			int size = (pos>pos_enviada)?pos-pos_enviada:SIZE_SEND_DATA-pos_enviada;
			//  Y se envían, alcutualizando la posición del último elemento enviado
			if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*) &datos[pos_enviada], (uint16_t) (size * sizeof(uint16_t))) == HAL_OK){
				pos_enviada = (pos>pos_enviada)?pos:0;
			}
		}

#ifndef ENVIO_XY // Leer línea 162 para entender mejor que se envía en cada momento
		// Acceso / lectura de la acción de control
		osMessageQueueGet(accion_controlXQueueHandle, &ang_obj_f, NULL, osWaitForever);
		float ang = (VUELTA_COMPLETA==VC_DEG)?ang_obj_f:(ang_obj_f*VC_DEG/(VC_RAD));
		datos[pos] = (50 + ang) * 500; // Ángulo objetivo

		// Acceso / lectura de la medida
		osMessageQueueGet(distXQueueHandle, &dist_X_f, NULL, osWaitForever);
		datos[pos+1] = (0.3 + dist_X_f) * 10000; // Distancia de la esfera al centro
#else
		// Acceso / lectura de la medida X
		osMessageQueueGet(accion_controlXQueueHandle, &dist_X_f, NULL, osWaitForever);
		datos[pos] = (0.3 + dist_X_f) * 10000; // Ángulo objetivo

		// Acceso / lectura de la medida Y
		osMessageQueueGet(distXQueueHandle, &dist_Y_f, NULL, osWaitForever);
		datos[pos+1] = (0.3 + dist_Y_f) * 10000; // Distancia de la esfera al centro
#endif

		pos = (pos+2)%SIZE_SEND_DATA;
	  }
}

// Gestión de los comandos recibidos, junto con el cambio de consigna
void Recepcion_DatosTask(void *argument){
	extern osMutexId_t consignaMutexHandle;
	extern UART_HandleTypeDef huart2;

#ifdef RECEPCION_DATOS
	HAL_UART_Receive_DMA(&huart2, pData_Setpoint, sizeof(pData_Setpoint));
#else
	extern osTimerId_t TimerRecepcionDatosHandle;

	int8_t pData_zero[SIZE_BUFFER_ZERO_SETPOINT][2] =  BUFFER_ZERO;
	int8_t pData_rectangulo[SIZE_BUFFER_RECTANGULO_SETPOINT][2] =  BUFFER_RECTANGULO;
	int8_t pData_infinito[SIZE_BUFFER_INFINITO_SETPOINT][2] =  BUFFER_INFINITO;
	int8_t *pData = &(pData_zero[0][0]);

	uint8_t size_buffer = SIZE_BUFFER_ZERO_SETPOINT;
	uint8_t modo = 0;
	int i = 0;
#endif
	for(;;)
	{
		// Espera a dato listo
		osThreadFlagsWait(FLAG_ID, osFlagsWaitAny, osWaitForever);

#ifndef RECEPCION_DATOS
		// Si se detecta que es necesario cambiar la consigna
		if (cambio_modo_setpoint){
			modo = (modo+1)%MAX_SETPOINT_MODES;
			
			switch(modo){
				case 0:	// Modo consigna constante en cero
					pData = &(pData_zero[0][0]);
					size_buffer = SIZE_BUFFER_ZERO_SETPOINT;
					osTimerStart(TimerRecepcionDatosHandle, TIME_SETPOINT_ZERO_S * 1000 / portTICK_PERIOD_MS);
					break;
				case 1: // Modo figura de infinito
					pData = &(pData_rectangulo[0][0]);
					size_buffer = SIZE_BUFFER_RECTANGULO_SETPOINT;
					osTimerStart(TimerRecepcionDatosHandle, TIME_SETPOINT_RECTANGULO_S * 1000 / portTICK_PERIOD_MS);
					break;
				case 2: // Modo figura de rectángulo
					pData = &(pData_infinito[0][0]);
					size_buffer = SIZE_BUFFER_INFINITO_SETPOINT;
					osTimerStart(TimerRecepcionDatosHandle, TIME_SETPOINT_INFINITO_S * 1000 / portTICK_PERIOD_MS);
					break;
				default:
					break;
			}
			
			cambio_modo_setpoint = 0;
			i = 0;
		}
		
		// Se modifica el siguiente valor de la consigna
		pData_Setpoint[0] = pData[i*2]; pData_Setpoint[1] = pData[i*2 + 1];
		i = (i+1)%size_buffer;
#endif

		// Adquiere el mutex y actualiza los datos
		osMutexAcquire(consignaMutexHandle, osWaitForever);
		setpoint_X = pData_Setpoint[0] / 100.0; setpoint_Y = pData_Setpoint[1] / 100.0;
		osMutexRelease(consignaMutexHandle);
	}
}
