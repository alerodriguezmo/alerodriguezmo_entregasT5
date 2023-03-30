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
GPIO_Handler_t handlerLED2 = {0};

/* = = = Handlers Timers = = = */
BasicTimer_Handler_t handlerBlinkyTimer = {0};

/* = = = Handlers EXTI = = = */
EXTI_Config_t handlerExti1 = {0};
EXTI_Config_t handlerExti2 = {0};


/* = = = Cabeceras de las funciones = = = */
void init_Hardware(void); // Función para inicializar el hardware

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
void init_Hardware(void){

	/* = = = LED DE ESTADO (BLINKY) = = = */
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


}
/* = = = = = FIN DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */

/* = = = = = INICIO DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerLED2);
}
/* = = = = = FIN DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */
