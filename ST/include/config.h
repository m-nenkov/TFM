#ifndef CONFIG_H
#define CONFIG_H

// CONFIGURACIÓN DE PROGRAMA

// Selección de sensor
// #define DATOS_TFT
#define DATOS_CAM

// Selección de controlador
#define CONTROL_PID
// #define CONTROL_LQR

// Configuración del envío y recepción de datos
// #define RECEPCION_DATOS

#define ENVIO_DATOS
#ifdef ENVIO_DATOS
	#define ENVIO_UART
	
	#define ENVIO_X
	// #define ENVIO_Y
	// #define ENVIO_XY
#endif


// Modo de calibración de los ángulos de la estructura
// #define CALIBRACION_ESTRUCTURA

// Unidades angulares
#define VC_DEG 360.0
#define VC_RAD (2.0*3.1415926)
#define VUELTA_COMPLETA VC_RAD // VC_DEG

// Controlador TFT
#define ADDR_READ_TFT 0x9B
#define ADDR_WRITE_TFT 0x9A
#define GPIO_RTS_TFT GPIOC
#define PIN_RTS_TFT GPIO_PIN_0

// Controlador Cámara
#define GPIO_RTS_CAM GPIOC
#define PIN_RTS_CAM GPIO_PIN_0

// Tiempo de muestreo
#define SAMPLE_TIME_S (float)(5.0/1000.0) // Periodo de ejecución de la tarea de control (y de muestreo de la cámara)

// PID 
#define PID_LIM_MAX (float) VUELTA_COMPLETA/12 // VUELTA_COMPLETA/12 // Aprox. 20º - 30º
#define PID_LIM_MIN (float) -PID_LIM_MAX
#define PID_LIM_MIN_INT PID_LIM_MIN/2
#define PID_LIM_MAX_INT PID_LIM_MAX/2

// Parámetros PID
#define PID_KP (float)(1.6)  // PID4-8: (138.0) 8-12: (250.0663) 12-17: (193) 18:(270.0663) 21-22: (0.64) Para cam: 160.0663 0.4
#define PID_KI (float)(0.0)
#define PID_KD (float) (0.6)// (58.0)  // PID4-8: (40.0) 8-12: (57.9335) 12-17: (41) 18:(57.9335) 21-22:(0.15) Para cam: 57.0 2.2
#define PID_TAU (float)(0.0) // (0.00159) 		// Cte de tiempo del filto paso bajo de la componente derivativa

// LQR
#define DIM_C1 2
#define DIM_C2 4
#define DIM_C {DIM_C1, DIM_C2}
#define MAT_C {0.0,7.0,0.0,0.0, 0.0,0.0,0.0,7.0}

#define DIM_X1 4
#define DIM_X2 1
#define DIM_X {DIM_X1, DIM_X2}
#define MAT_X {0,0,0,0}

#define DIM_K1 2
#define DIM_K2 4
#define DIM_K {DIM_K1, DIM_K2}
#define MAT_K {1,0.5354,0.0,0.0, 0.0,0.0,1,0.5345}

#define DIM_R1 2
#define DIM_R2 1
#define DIM_R {DIM_R1, DIM_R2}
#define MAT_R {1,1}

#define Kr 1

// Servo-motores
#define MAX_ANG PID_LIM_MAX // VUELTA_COMPLETA/12
#define MIN_ANG -MAX_ANG
#define SERVO_TIMER_PERIOD (uint16_t)(16000 - 1)

// Otros parámetros
#define ADC_RES 12 // bits
#define MAX_ADC_VAL_TFT 4096.0

// Tamaó de la imágen de la cámara
#define PIXELS_Y 120.0 //  160.0 También puede ser superior pero la velocidad del muestro disminuye
#define PIXELS_X 80.0 //  120.0


// Tamaño de la estructura
// Para TFT
#ifdef DATOS_TFT
#define LONG_Y  (19.0 / 100.0)
#define LONG_X  (25.0 / 100.0)
#endif
// Para cámara
#ifdef DATOS_CAM
#define LONG_Y  (21.0 / 100.0)
#define LONG_X  (16.0 / 100.0)
#endif

// Correcciones del ángulo de la estructura
//Para TFT
#ifdef DATOS_TFT
#define ERROR_ANG_X -VUELTA_COMPLETA/142
#define ERROR_ANG_Y VUELTA_COMPLETA/70
#endif
// Para cámara
#ifdef DATOS_CAM
#define ERROR_ANG_X VUELTA_COMPLETA/393
#define ERROR_ANG_Y VUELTA_COMPLETA/50
#endif

// Tamaño del buffer para envio de datos
#define SIZE_SEND_DATA 10

// Relación ang servo con estrcutura
#define GRADO_TF_ANG_PLATF_SERVO 3
#define TF_ANG_PLATF_SERVO {0.0009, 0.0072, 2.2486, -0.0953}; // En rad: {-0.0588, -0.0184, 0.4360, -0.0006}; 

//  Señal de consigna
#ifdef RECEPCION_DATOS
	#define TIME_SETPOINT_S 1
#else
	#define MAX_SIZE_BUFFER_SETPOINT 8
    #define MAX_SETPOINT_MODES 3

	#define TIME_SETPOINT_ZERO_S 3 // segundos
	#define SIZE_BUFFER_ZERO_SETPOINT 1

	#define TIME_SETPOINT_INFINITO_S 0.5 // segundos
	#define SIZE_BUFFER_INFINITO_SETPOINT 8

#define TIME_SETPOINT_RECTANGULO_S 2.5 // segundos
	#define SIZE_BUFFER_RECTANGULO_SETPOINT 4
#endif

// Recorido de la de la esfera en los modos de demostración
#ifdef DATOS_TFT
	#define MAX_DISTX 5.0 // (cm) La máxima distancia a la que se lleva la bola al hacer las figuras
	#define MAX_DISTY 3.5
#else 
	#ifdef DATOS_CAM // Para la cámara es más pequeña debido al tamaño de su plataforma
		#define MAX_DISTX 3.0
		#define MAX_DISTY 3.0
	#endif
#endif

#define BUFFER_ZERO {{0.0,0.0}}
#define BUFFER_INFINITO {{0,0}, {-MAX_DISTX,0}, {-MAX_DISTX*1.3,-MAX_DISTY*1.3}, {0,-MAX_DISTY}, {0,0}, {0,MAX_DISTY}, {MAX_DISTX*1.3, MAX_DISTY*1.3}, {MAX_DISTX,0}}
#define BUFFER_RECTANGULO {{MAX_DISTX,MAX_DISTY},{MAX_DISTX,-MAX_DISTY},{-MAX_DISTX,-MAX_DISTY},{-MAX_DISTX,MAX_DISTY}}


// SISTEMA OPERATIVO
#define SYS_FREQ_HZ configTICK_RATE_HZ

// Definición de los tiempo de ejecución de las tareas
#define MS_TAREA_LED 200 // ms
#define MS_TAREA_CONTROL SAMPLE_TIME_S*1000 // ms = s * 1000
#define MS_TAREA_TOMA_DATOS MS_TAREA_CONTROL // ms
#define MS_TAREA_ENVIO_DATOS MS_TAREA_CONTROL*3 // ms

#define TICKS_TAREA_LED (uint32_t)( MS_TAREA_LED * SYS_FREQ_HZ / 1000 )
#define TICKS_TAREA_CONTROL (uint32_t)( MS_TAREA_CONTROL * SYS_FREQ_HZ / 1000 )
#define TICKS_TAREA_TOMA_DATOS (uint32_t)( MS_TAREA_TOMA_DATOS * SYS_FREQ_HZ / 1000 )
#define TICKS_TAREA_ENVIO_DATOS (uint32_t)( MS_TAREA_ENVIO_DATOS * SYS_FREQ_HZ / 1000 )
#define TICKS_TAREA_RECEPCION_DATOS (uint32_t)( TIME_SETPOINT_ZERO_S * SYS_FREQ_HZ )

// Definción de variables del LED
#define LED_GPIO_Port GPIOA
#define LED_GPIO_Pin GPIO_PIN_5

// Definición de las flags que activan las tareas
#define FLAG_ID (uint32_t)0x00000005U

// Botón de ususario
#define USER_BUTTON_PORT GPIOC
#define USER_BUTTON_PIN GPIO_PIN_13

#endif