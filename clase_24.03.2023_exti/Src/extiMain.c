/*
 * extiMain.c
 *
 *  Created on: Mar 24, 2023
 *      Author: ingfisica
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include "GPIOxDriver.h"
#include "BasicTimer.h"

/* Definición de los elementos del sistema */
GPIO_Handler_t handlerLED2 = {0};
BasicTimer_Handler_t handlerBlinkyTimer = {0};
GPIO_Handler_t handlerUserButton = {0};
uint32_t counterExti13 = 0;


// Prototipos de las funciones
void init_Hardware(void);
void EXTI15_10_IRQHandler(void);

// Core del programa
int main(void){

	// Inicialización de todos los elementos del sistema
	init_Hardware();

	while(1){

	}
}

void init_Hardware(void){

	// Configuración del LED2 - PA5
	handlerLED2.pGPIOx								= GPIOA;
	handlerLED2.GPIO_PinConfig.GPIO_PinNumber		= PIN_5;
	handlerLED2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerLED2.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerLED2.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerLED2.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	// Cargando la configuración
	GPIO_Config(&handlerLED2);

	GPIO_WritePin(&handlerLED2, SET);

	// Configuración del TIM2 para que haga un blinky cada 300ms
	handlerBlinkyTimer.ptrTIMx								= TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 300; // Lanza una interrupción cada 300ms
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable		= 1;

	// Cargando la configuración del TIM2
	BasicTimer_Config(&handlerBlinkyTimer);

	// Configuración del exti
	handlerUserButton.pGPIOx								= GPIOC;
	handlerUserButton.GPIO_PinConfig.GPIO_PinNumber			= PIN_13;
	handlerUserButton.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	handlerUserButton.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;


	// Cargando la configuración
	GPIO_Config(&handlerUserButton);

	/* 2. Activando la señal de reloj del SYSCFG*/
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* 3. Configurando el mux 13 para que utilice el puerto C*/
	SYSCFG->EXTICR[3] &= ~(0xF << SYSCFG_EXTICR4_EXTI13_Pos);
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	/* 4. Configurar el perfil del flanco */
	EXTI->FTSR = 0; // Desactivamos todos los posibles flancos de bajada

	EXTI->RTSR = 0; // Llevamos el registro a un valor conocido
	EXTI->RTSR |= EXTI_RTSR_TR13;

	/* 4b. Activar interrupción */
	EXTI->IMR = 0;
	EXTI->IMR |= EXTI_IMR_IM13;

	/* 5a. Desactivar TODAS las interrupciones */
	__disable_irq();

	/* 5.b Matricular la interrupción en el NVIC */
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* 5c. Crear ISR */
	/* 5d. Crear el callback */
	/* 5e. activar las interrupciones */
	__enable_irq();

}

void EXTI15_10_IRQHandler(void){

	if((EXTI->PR & EXTI_PR_PR13) !=0 ){
		EXTI->PR |= EXTI_PR_PR13;	// Limpiar la bandera de exti 13
	}
}

void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerLED2);
}
