#include "drivers.h"
#include "math.h"

// Variables globales
float tf_ang_platf_serv[GRADO_TF_ANG_PLATF_SERVO + 1] = TF_ANG_PLATF_SERVO;

// Controlador del PWM
void PWM_controller_init(PWM_Controller_t *this, TIM_HandleTypeDef *tim, uint32_t tim_channel, uint32_t timer_period, float ang_max, float ang_min){
	// Inicialización de  varibales
	this->timer = tim;
	this->timer_channel = tim_channel;
	this->servo_timer_period = timer_period;
	this->max_ang = ang_max;
	this->min_ang = ang_min;
}

// Función que modifica la señal PWM en función del ángulo obtjetivo de un servo-motor  
void set_PWM(PWM_Controller_t *this, float ang){
	// Normalización del ángulo a menos de una revolución
	while (ang > VUELTA_COMPLETA){
		ang = ang - VUELTA_COMPLETA;
	}
	while (ang < 0){
		ang = ang + VUELTA_COMPLETA;
	}

	// Normalizar el ángulo a +- 90º
	if ((ang > VUELTA_COMPLETA/4) && (ang < VUELTA_COMPLETA*(3.0/4.0))){ // Ángulo en el segundo o tercer cuadrante cuadrante
		ang = ang - VUELTA_COMPLETA/2;
	}
	else if (ang > VUELTA_COMPLETA/4){ // Ángulo en el cuarto cuadrante
		ang = ang - VUELTA_COMPLETA;
	}

	// Se limita al ángulo que permite girar el servomotor
	if (ang > this->max_ang){
		ang = this->max_ang;
	}
	else if (ang <  this->min_ang){
		ang = this->min_ang;
	}

	// Cálculo del ciclo de trabajo
	float ang_norm_ms = (((ang > VUELTA_COMPLETA/2)? ang - VUELTA_COMPLETA:ang) / (VUELTA_COMPLETA/3));
	float DutyCicle =  (1.5 + ang_norm_ms) / 20 * 100;

	// Setup del timer que gestiona el PWM
	float PulseValue = ((float)(this->servo_timer_period + 1) * DutyCicle) / 100 - 1;
	__HAL_TIM_SET_COMPARE(this->timer, this->timer_channel, PulseValue);
}

// Controlador del ADC
void ADC_Conv_init(ADC_Converter_t *this, uint32_t adc_max_val, float max_range){
	// Inicialización de la escrutura
	this->ADC_MAX_Val = adc_max_val;
	this->range = max_range;
}

// Función que trasnforma el valor obtenido de un ADC a un rango determinado
void convADC2dist(ADC_Converter_t *this, float *dist, uint16_t adc_value){
	// Distancia al eje X
	float norm_dist = ((float)adc_value / this->ADC_MAX_Val) * this->range;
	*dist = norm_dist - (float)(this->range/2); // Distancia al eje X

}

// Función que obtiene el ángulo del servo-motor necesario para una determinada inclinación de la plataforma
void calculo_ang_servo(const float *ang_platf, float *ang_servo){
	int i;
	
	// Se obtiene el ángulo en grados sexagesimales
	float ang_aux = *ang_platf * ((VUELTA_COMPLETA == VC_DEG)? 1: VC_DEG/VC_RAD);
	
	// Realizamos la operación de trasnformación
	*ang_servo = tf_ang_platf_serv[GRADO_TF_ANG_PLATF_SERVO];
	for (i=0; i<GRADO_TF_ANG_PLATF_SERVO; i++){
		*ang_servo += tf_ang_platf_serv[i] * pow(ang_aux, GRADO_TF_ANG_PLATF_SERVO-i);
	}
	
	// Transformamos el ángulo obtenido al sistema en el que estemos trabajando
	*ang_servo *= ((VUELTA_COMPLETA == VC_DEG)? 1: VC_RAD/VC_DEG);
}

// Función que corrige la posición de la esfera en función del ángulo de inclinación de la plataforma (Sólo usado en la cámara)
void ajuste_pos_bola(const float *ang, float *dist){
	
	float ang_aux;
	
	ang_aux = (VUELTA_COMPLETA == VC_RAD)? *ang:(*ang / VC_DEG * VC_RAD);
	*dist /= cos(ang_aux); // se ha cambiado

}

// Función que transfroma la trama obtenida por el controlador de la platalla resistiva a un valor ADC de 12 bits 
void procesado_datos_TFT(uint16_t *DatosX, uint16_t *DatosY, const uint8_t* pData){
	// Los datos se reciben en paquetes de 1 byte
	// Parma más información leer datasheet de AR1000 Series Resistive Touch Screen Controller
	// Datos en eje X
	uint16_t Datos_High =  pData[2] << 7;	// Primer byte de datos
	uint16_t Datos_Low = pData[1];			// Segundo byte de datos
	*DatosX = Datos_High | Datos_Low;		// Unión de ambos datos

	// Datos en eje X
	Datos_High =  pData[4] << 7;
	Datos_Low = pData[3];
	*DatosY = Datos_High | Datos_Low;
}

// Función que transfroma la trama obtenida por la cámara a un valor entero
void procesado_datos_CAM(uint8_t *DatosX, uint8_t *DatosY, const uint8_t *pData){
	// Se reciben dos bytes, el primer byte es el pixel Y
	// Para más información observar el protocolo de comunicación del fichero main de la cámara
	//	y el segundo es el pixel X
	// Datos de X
	*DatosX = pData[1]; // DatosX corresponde con el segundo byte de la trama(2 bytes)
	// Datos de Y
	*DatosY = pData[0]; // DatosY corresponde con el primer byte de la trama(2 bytes)
}

// Función privada para gestión de la incicialización Hardware del LED_Controller_t
void _init_LED(LED_Controller_t *this){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(this->Gpio, this->pin, GPIO_PIN_RESET);

	/*Configure GPIO pins */
	GPIO_InitStruct.Pin = this->pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(this->Gpio, &GPIO_InitStruct);
}

// Controlador de leds
void LED_controller_init(LED_Controller_t *this, GPIO_TypeDef *GPIOX, uint16_t GPIO_PIN){
	// Se guardan las variables
	this->pin = GPIO_PIN;
	this->Gpio = GPIOX;
	
	// Inicialización Hardware
	_init_LED(this);
}

// Función que activa el LED
void encender_led(LED_Controller_t *this){
	HAL_GPIO_WritePin(this->Gpio, this->pin, GPIO_PIN_SET);
}

// Función que descativa el LED
void apagar_led(LED_Controller_t *this){
	HAL_GPIO_WritePin(this->Gpio, this->pin, GPIO_PIN_RESET);
}

// Función que modifica el estado del LED
void toggle_led(LED_Controller_t *this){
	HAL_GPIO_TogglePin(this->Gpio, this->pin);
}


