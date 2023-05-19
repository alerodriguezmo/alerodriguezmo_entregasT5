/*
 * extiMain.c
 *
 *  Created on: Mar 24, 2023
 *      Author: alerodriguezmo
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"

/* = = = = = INICIO DE DEFINICIÓN DE LOS ELEMENTOS DEL SISTEMA = = = = =  */

/* = = = Handlers GPIO = = = */
GPIO_Handler_t handlerLED2 		= {0};		 // LED blinky

GPIO_Handler_t handlerSelUni 	= {0};		// Selector que activa el transistor correspondiente al display de unidades
GPIO_Handler_t handlerSelDec 	= {0};		// Selector que activa el transistor correspondiente al display de decenas

GPIO_Handler_t handlerSegA		= {0};		// Handler para el segmento A del display 7 segmentos
GPIO_Handler_t handlerSegB		= {0};		// Handler para el segmento B del display 7 segmentos
GPIO_Handler_t handlerSegC		= {0};		// Handler para el segmento C del display 7 segmentos
GPIO_Handler_t handlerSegD		= {0};		// Handler para el segmento D del display 7 segmentos
GPIO_Handler_t handlerSegE		= {0};		// Handler para el segmento E del display 7 segmentos
GPIO_Handler_t handlerSegF		= {0};		// Handler para el segmento F del display 7 segmentos
GPIO_Handler_t handlerSegG		= {0};		// Handler para el segmento G del display 7 segmentos

GPIO_Handler_t handlerEncoderA		= {0};	// Handler del puerto CLK del encoder
GPIO_Handler_t handlerEncoderB		= {0};  // Handler del puerto DT del encoder
GPIO_Handler_t handlerEncoderSW		= {0};	// Handler del puerto SW del encoder


/* = = = Handlers Timers = = = */
BasicTimer_Handler_t handlerBlinkyTimer 	= {0};		// Timer del LED blinky
BasicTimer_Handler_t handlerDisplayTimer	= {0};		// Timer usado para el display

/* = = = Handlers EXTI = = = */
EXTI_Config_t handlerExtiEncoderA 		= {0};			// EXTI correspondiente al giro del encoder. 1->CCW ; 0->CW
EXTI_Config_t handlerExtiEncoderSW		= {0};			// EXTI correspondiente al botón del encoder

/* = = = Variables auxiliares y contadores = = = */
uint8_t stateB;					// Estado actual del puerto DT del encoder. Esta es la variable que gobierna el giro
uint8_t mainCounter 	= 0;	// Contador principal para el modo número
uint8_t snakeCounter 	= 1;	// Contador que guarda la posición de la culebrita
uint8_t stateDisplay	= 0;	// Auxiliar para usar display de decenas o unidades. 0->Decenas ; 1->Unidades
uint8_t stateButton		= 1;	// Auxiliar para el botón del encoder. Define si se está en modo número o culebrita


/* = = = Cabeceras de las funciones = = = */
void init_Hardware(void); 					// Función para inicializar el hardware
void display_Number(uint8_t number); 		// Función para mostar un número en el display
void display_Snake(uint8_t counterSnake);	// Función para mostrar la culebrita

/* = = = = = FIN DE DEFINICIÓN DE LOS ELEMENTOS DEL SISTEMA = = = = =  */

/* = = = = = INICIO DEL CORE DEL PROGRAMA = = = = =  */
int main(void){

	// Inicialización de todos los elementos del sistema
	init_Hardware();

	while(1){

	}
}
/* = = = = = FIN DEL CORE DEL PROGRAMA = = = = =  */

/* = = = = = INICIO DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */

// Función de inicialización de hardware
void init_Hardware(void){

	/* = = = INICIO DEL LED DE ESTADO (BLINKY) = = = */
	// Configuración del LED2 - PA5
	handlerLED2.pGPIOx								= GPIOA;
	handlerLED2.GPIO_PinConfig.GPIO_PinNumber		= PIN_5;
	handlerLED2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerLED2.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerLED2.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerLED2.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	// Cargando la configuración
	GPIO_Config(&handlerLED2);

	GPIO_WritePin(&handlerLED2, SET); // Se establece que el LED esté encendido por defecto

	// Configuración del TIM2 para que haga un blinky cada 250ms
	handlerBlinkyTimer.ptrTIMx								= TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 250; // Lanza una interrupción cada 250 ms
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable		= 1;

	// Cargando la configuración del TIM2
	BasicTimer_Config(&handlerBlinkyTimer);

	/* = = = FIN DEL LED DE ESTADO (BLINKY) = = = */

	// Configuración del TIM3 para establecer la tasa de refresco del display
	handlerDisplayTimer.ptrTIMx									= TIM5;
	handlerDisplayTimer.TIMx_Config.TIMx_mode					= BTIMER_MODE_UP;
	handlerDisplayTimer.TIMx_Config.TIMx_speed					= BTIMER_SPEED_1ms;
	handlerDisplayTimer.TIMx_Config.TIMx_period					= 10; // Lanza una interrupción cada 10 ms
	handlerDisplayTimer.TIMx_Config.TIMx_interruptEnable		= 1;

	// Cargando la configuración del TIM3
	BasicTimer_Config(&handlerDisplayTimer);


	/* = = = INICIO DE CONFIGURACIÓN DE PINES = = = */

	// Se definen las configuraciones de los pines {PC9, PC8} como salidas que controlan
	// el switcheo de los transistores de unidades y decenas (respectivamente)

	// Parámetros para la configuración del PC9 que controla el transistor de unidades
	handlerSelUni.pGPIOx									= GPIOC;
	handlerSelUni.GPIO_PinConfig.GPIO_PinNumber				= PIN_9;
	handlerSelUni.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSelUni.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_PUSHPULL;
	handlerSelUni.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSelUni.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_FAST;
	handlerSelUni.GPIO_PinConfig.GPIO_PinAltFunMode			= AF0;

	// Parámetros para la configuración del PC8 que controla el transistor de decenas
	handlerSelDec.pGPIOx									= GPIOC;
	handlerSelDec.GPIO_PinConfig.GPIO_PinNumber				= PIN_8;
	handlerSelDec.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSelDec.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_PUSHPULL;
	handlerSelDec.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSelDec.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_FAST;
	handlerSelDec.GPIO_PinConfig.GPIO_PinAltFunMode			= AF0;

	// Se carga la configuración de los pines selectores
	GPIO_Config(&handlerSelUni);
	GPIO_Config(&handlerSelDec);

	// Se definen las configuraciones de los pines {PA11, PB12, PB1, PC5, PC6, PA12, PB2} como salidas que controlan
	// cada uno de los 7 segmentos del display

	// Parámetros para la configuración del PC8 que controla el segmento A
	handlerSegA.pGPIOx									= GPIOA;
	handlerSegA.GPIO_PinConfig.GPIO_PinNumber			= PIN_11;
	handlerSegA.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSegA.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerSegA.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSegA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerSegA.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PB12 que controla el segmento B
	handlerSegB.pGPIOx									= GPIOB;
	handlerSegB.GPIO_PinConfig.GPIO_PinNumber			= PIN_12;
	handlerSegB.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSegB.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerSegB.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSegB.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerSegB.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PD8 que controla el segmento C
	handlerSegC.pGPIOx									= GPIOB;
	handlerSegC.GPIO_PinConfig.GPIO_PinNumber			= PIN_1;
	handlerSegC.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSegC.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerSegC.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSegC.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerSegC.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del C5 que controla el segmento D
	handlerSegD.pGPIOx									= GPIOC;
	handlerSegD.GPIO_PinConfig.GPIO_PinNumber			= PIN_5;
	handlerSegD.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSegD.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerSegD.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSegD.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerSegD.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PB11 que controla el segmento E
	handlerSegE.pGPIOx									= GPIOC;
	handlerSegE.GPIO_PinConfig.GPIO_PinNumber			= PIN_6;
	handlerSegE.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSegE.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerSegE.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSegE.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerSegE.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PB2 que controla el segmento F
	handlerSegF.pGPIOx									= GPIOA;
	handlerSegF.GPIO_PinConfig.GPIO_PinNumber			= PIN_12;
	handlerSegF.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSegF.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerSegF.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSegF.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerSegF.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PB15 que controla el segmento G
	handlerSegG.pGPIOx									= GPIOB;
	handlerSegG.GPIO_PinConfig.GPIO_PinNumber			= PIN_2;
	handlerSegG.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerSegG.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerSegG.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerSegG.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerSegG.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Se carga la configuración de los pines de salida
	GPIO_Config(&handlerSegA);
	GPIO_Config(&handlerSegB);
	GPIO_Config(&handlerSegC);
	GPIO_Config(&handlerSegD);
	GPIO_Config(&handlerSegE);
	GPIO_Config(&handlerSegF);
	GPIO_Config(&handlerSegG);

	// Se definen las configuraciones de los pines {PB15,PB14,PB13} como entradas que representan
	// el estado del encoder

	// Parámetros para la configuración del PC4 que respresenta a CLK
	handlerEncoderA.pGPIOx									= GPIOB;
	handlerEncoderA.GPIO_PinConfig.GPIO_PinNumber			= PIN_15;
	handlerEncoderA.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_IN;
	handlerEncoderA.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerEncoderA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerEncoderA.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PB5 que representa a DT
	handlerEncoderB.pGPIOx									= GPIOB;
	handlerEncoderB.GPIO_PinConfig.GPIO_PinNumber			= PIN_14;
	handlerEncoderB.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_IN;
	handlerEncoderB.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerEncoderB.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerEncoderB.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PA3 que representa a SW
	handlerEncoderSW.pGPIOx									= GPIOB;
	handlerEncoderSW.GPIO_PinConfig.GPIO_PinNumber			= PIN_13;
	handlerEncoderSW.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	handlerEncoderSW.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerEncoderSW.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerEncoderSW.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Se carga la configuración del pin de entrada del DT del encoder
	GPIO_Config(&handlerEncoderB);

	/* = = = FIN DE CONFIGURACIÓN DE PINES = = = */

	/* = = = INICIO DE CONFIGURACIÓN DE EXTI = = = */

	// Se configuran ambas interrupciones para que se lancen con un flanco de bajada y un flanco de subida, respectivamente
	handlerExtiEncoderA.edgeType 		= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEncoderA.pGPIOHandler	= &handlerEncoderA;

	handlerExtiEncoderSW.edgeType 		= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiEncoderSW.pGPIOHandler	= &handlerEncoderSW;

	// Se cargan las configuraciones de las dos exti
	extInt_Config(&handlerExtiEncoderA);
	extInt_Config(&handlerExtiEncoderSW);

	/* = = = FIN DE CONFIGURACIÓN DE EXTI = = = */
}

// Definición de la función para mostrar un número en el display.
// Dependiendo del número, se enciende el segmento correspondiente teniendo en cuenta
// la configuración de ánodo común.
void display_Number(uint8_t number){
	switch(number){
	case 0:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,RESET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 1:{
		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);


		break;
	}
	case 2:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,RESET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,RESET);

		break;
	}
	case 3:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,RESET);

		break;
	}
	case 4:{
		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,RESET);

		break;
	}
	case 5:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,RESET);

		break;
	}
	case 6:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,RESET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,RESET);

		break;
	}
	case 7:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 8:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,RESET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,RESET);


		break;
	}
	case 9:{
		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,RESET);

		break;
		}
		default: break;

		}
}

// Función para mostrar el modo "Culebrita"
// Se asigna cada posible posición de la culebrita con un
// número dentro del contador auxiliar y se enciende el segmento correspondiente teniendo en cuenta
// la configuración de ánodo común.
void display_Snake(uint8_t counterSnake){
	switch(counterSnake){
	case 1:{
		GPIO_WritePin(&handlerSelUni,SET);
		GPIO_WritePin(&handlerSelDec,RESET);

		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 2:{
		GPIO_WritePin(&handlerSelUni,RESET);
		GPIO_WritePin(&handlerSelDec,SET);

		GPIO_WritePin(&handlerSegA,RESET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 3:{
		GPIO_WritePin(&handlerSelUni,RESET);
		GPIO_WritePin(&handlerSelDec,SET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,SET);
		break;
	}
	case 4:{
		GPIO_WritePin(&handlerSelUni,RESET);
		GPIO_WritePin(&handlerSelDec,SET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,RESET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 5:{
		GPIO_WritePin(&handlerSelUni,RESET);
		GPIO_WritePin(&handlerSelDec,SET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 6:{
		GPIO_WritePin(&handlerSelUni,SET);
		GPIO_WritePin(&handlerSelDec,RESET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,RESET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);
		break;
	}
	case 7:{
		GPIO_WritePin(&handlerSelUni,SET);
		GPIO_WritePin(&handlerSelDec,RESET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,RESET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 8:{
		GPIO_WritePin(&handlerSelUni,RESET);
		GPIO_WritePin(&handlerSelDec,SET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);
		break;
	}
	case 9:{
		GPIO_WritePin(&handlerSelUni,RESET);
		GPIO_WritePin(&handlerSelDec,SET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);

		break;
	}
	case 10:{
		GPIO_WritePin(&handlerSelUni,SET);
		GPIO_WritePin(&handlerSelDec,RESET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,RESET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);
		break;
	}
	case 11:{
		GPIO_WritePin(&handlerSelUni,SET);
		GPIO_WritePin(&handlerSelDec,RESET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,SET);
		GPIO_WritePin(&handlerSegC,RESET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);
		break;
	}
	case 12:{
		GPIO_WritePin(&handlerSelUni,SET);
		GPIO_WritePin(&handlerSelDec,RESET);

		GPIO_WritePin(&handlerSegA,SET);
		GPIO_WritePin(&handlerSegB,RESET);
		GPIO_WritePin(&handlerSegC,SET);
		GPIO_WritePin(&handlerSegD,SET);
		GPIO_WritePin(&handlerSegE,SET);
		GPIO_WritePin(&handlerSegF,SET);
		GPIO_WritePin(&handlerSegG,SET);
		break;
	}

	default: break;

	}

}
/* = = = = = FIN DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */

/* = = = = = INICIO DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */

// Callback del timer 2 correspondiente al LED Blinky
void BasicTimer2_Callback(void){
	// Callback del blinky
	GPIOxTooglePin(&handlerLED2);
}

// Callback del timer 5 correspondiente al switcheo de los display 7 segmentos
void BasicTimer5_Callback(void){
	if(stateButton){
		// Si se encuentra el modo número activado:
		if(stateDisplay){
			GPIO_WritePin(&handlerSelUni,RESET);
			GPIO_WritePin(&handlerSelDec,SET);
			display_Number(mainCounter / 10);

			stateDisplay = 0;
		}else{
			GPIO_WritePin(&handlerSelDec,RESET);
			GPIO_WritePin(&handlerSelUni,SET);
			display_Number(mainCounter % 10);
			stateDisplay = 1;
		}
	}
	else{
		// Si se encuentra el modo culebrita activado:
		if(snakeCounter == 0){
			snakeCounter = 12;
		}else if(snakeCounter == 13){
			snakeCounter = 1;
		}
		display_Snake(snakeCounter);
	}
}

// Callback de rotación correspondiente a puerto B15 (encoder)
void callback_extInt15(void){
	stateB = GPIO_ReadPin(&handlerEncoderB);

	// Se debe verificar en qué modo está el sistema: número o culebrita

	// Si se está en modo número:
	if(stateButton){
		//CCW para el modo número
		if(!stateB){
			if(mainCounter == 0){
				mainCounter++;
			}
			mainCounter--;
		}
		//CW para el modo número
		else{
			if(mainCounter == 99){
				mainCounter--;
			}
			mainCounter++;
		}
	}

	// Si se está en modo culebrita
	else{
		//CCW para el modo culebrita
		if(!stateB){
			snakeCounter--;
			if(snakeCounter == 0){
				snakeCounter = 12;
			}
		}
		//CW para el modo culebrita
		else{
			snakeCounter++;
			if(snakeCounter == 13){
				snakeCounter = 1;
			}
		}
	}
}

// Callback correspondiente al cambio de estado entre modo número y culebrita
void callback_extInt13(void){
	if(stateButton){
		stateButton = 0;
	}else{
		stateButton = 1;
	}
}

/* = = = = = FIN DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */
