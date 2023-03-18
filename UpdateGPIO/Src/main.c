/**
 ******************************************************************************
 * @file           : main.c
 * @author         : alerodriguezmo
 * @brief          : Configuracion basica de un proyecto
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include "GPIOxDriver.h"

// Función TooglePin
void GPIOxTooglePin(GPIO_Handler_t *pPinHandler){
	// Usamos una compuerta XOR en el estado del pin:
	pPinHandler->pGPIOx->MODER ^= (1 << pPinHandler->GPIO_PinConfig.GPIO_PinNumber);

}

// Definición de un elemento
GPIO_Handler_t handlerLed = {0}; // PA5

int main(void)
{
	// Configurando el LED_2 -> PA5
	handlerLed.pGPIOx									= GPIOA;
	handlerLed.GPIO_PinConfig.GPIO_PinNumber			= PIN_5;
	handlerLed.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerLed.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerLed.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_FAST;
	handlerLed.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	// Cargamos la configuración del PinA5
	GPIO_Config(&handlerLed);

	GPIO_WritePin(&handlerLed, SET);

    /* Loop forever */
	while(1){
		GPIOxTooglePin(&handlerLed);

		for(int i = 0; i < 2000000 ; i++){
			__NOP();
		}

	}

	return 0;
}
