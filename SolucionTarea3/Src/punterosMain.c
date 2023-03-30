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

// Definición de elementos del programa
uint32_t counter 			= 0;
uint32_t auxVariable		= 0;

uint32_t *ptr_Counter;

uint8_t byteVariable;
uint8_t	*ptr_ByteVariable;


int main(void){

	// Trabajando con las variables y los punteros
	counter = 3456789; // Asignar un valor
	auxVariable = counter; // C es pasado por valor

	ptr_Counter = &counter; // Solo recibe posiciones de memoria, se pasa la posición de counter

	*ptr_Counter = 9876543;

	ptr_Counter++; // Movemos una posición de memoria

	*ptr_Counter = 9876543; // Escribimos sobre la nueva posición de memoria

	byteVariable = 234;
	ptr_ByteVariable = &byteVariable;
	*ptr_ByteVariable = 87;

	//ptr_ByteVariable = &counter;

	ptr_Counter = &counter;
	auxVariable = (uint32_t)ptr_Counter;

	ptr_ByteVariable = (uint8_t*)auxVariable;

	while(1){

	}
}
