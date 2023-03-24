/**
 ******************************************************************************
 * @file           : main.c
 * @author         : alerodriguezmo
 * @brief          : Clase del viernes 10 de marzo del 2023
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

#include <stdint.h>
#include "stm32f411xx_hal.h"

int main(void)
{
	/* Configuración inicial del MCU */

	// Activación de la señal de reloj:
	RCC->AHB1ENR &= ~(1 << 0);		// Limpieza del bit
	RCC->AHB1ENR |= (1 << 0);		// Escritura para activar la señal de reloj

	/* Configurando el pin A5 como salida */

	// Configurando en el modo salida
	GPIOA->MODER &= ~(0b11 << 10);	// Limpieza
	GPIOA->MODER |= (0b01 << 10);	// Escritura

	// Configurando el tipo de salida
	GPIOA->OTYPER &= ~(1 << 5);		// Limpieza y configuración

	// Configurando la velocidad
	GPIOA->OSPEER &= ~(0b11 << 10);	//Limpieza
	GPIOA->OSPEER |= (0b11 << 10);	// Escritura

	// Configurando pull-up/pull-down
	GPIOA->PUPDR &= ~(0b11 << 10);	// No PUPD

	// Dándole salida al pin
	GPIOA->ODR &= ~(1 << 5);		// Limpiamos la salida PAS, apaga el led
	GPIOA->ODR |= (1 << 5);			// LED encendido!


}
