#ifndef DRIVERS_H
#define DRIVERS_H

#include "stm32l4xx_hal.h"
#include "config.h"

// Controlador PWM
typedef struct PWM_controller{
	TIM_HandleTypeDef *timer;
	uint32_t timer_channel, servo_timer_period;
	float max_ang, min_ang;
}PWM_Controller_t;

// Gestión del controlador PWM
void PWM_controller_init(PWM_Controller_t *this, TIM_HandleTypeDef *tim, uint32_t timer_channel, uint32_t servo_timer_period, float max_ang, float min_ang);
void set_PWM(PWM_Controller_t *this, float ang);

// Conversor (ADC) a rango determinado
typedef struct ADC_converter{
	uint32_t ADC_MAX_Val;
	float range;
} ADC_Converter_t;

// Gestión del conversor
void ADC_Conv_init(ADC_Converter_t *this, uint32_t adc_max_val, float max_range);
void convADC2dist(ADC_Converter_t *this, float *dist, uint16_t adc_value);

// Se cáclula el ángulo en el servo-motor necesario para obetener un ángulo de inclinación en la plataforma
void calculo_ang_servo(const float *ang_platf, float *ang_servo);

// Ajuste posición de la esfera en función del ángulo de inclinación de la plataforma
void ajuste_pos_bola(const float *ang, float *dist);

// Procesado de la trama recibida para separar datos en X e Y. 
void procesado_datos_TFT(uint16_t *DatosX, uint16_t *DatosY, const uint8_t *pData);
void procesado_datos_CAM(uint8_t *DatosX, uint8_t *DatosY, const uint8_t *pData);

// Controlador LEDs
typedef struct LED_Controller{
	GPIO_TypeDef *Gpio;
	uint16_t pin;
} LED_Controller_t;

// Control de un GPIO
void LED_controller_init(LED_Controller_t *this, GPIO_TypeDef *GPIOX, uint16_t GPIO_PIN);
void encender_led(LED_Controller_t *this);
void apagar_led(LED_Controller_t *this);
void toggle_led(LED_Controller_t *this);

#endif
