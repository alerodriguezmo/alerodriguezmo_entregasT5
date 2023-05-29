/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : PLLDriver.c
 * @brief    : Archivo de fuente del driver del periférico PLL
 * **************************************************************************************************
 */

#include "PLLDriver.h"

void configPLL(uint8_t frequency){
	// HSI clock selected as PLL and PLLI2S clock entry
	RCC -> PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);

	switch (frequency){
	case 0:
		/* Division factor for the main PLL (PLL) input clock
		 * VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
		 * VCO input frequency ranges from 1 to 2 MHz.*
		 */
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // Limpiamos
		/*Ponemos PLLM en 8 */
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLM_3);

		/* Main PLL (PLL) multiplication factor for VCO
		 * VCO output frequency = VCO input frequency × PLLN with 50 ≤ PLLN ≤ 432
		 * VCO output frequency is between 100 and 432 MHz
		 * */
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); // Limpiamos
		/* Ponemos PLLN en 80 */
//		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_0);
//		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_1);
//		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_2);
//		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_3);
//		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_6);

		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_4);
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_6);

		/* Main PLL (PLL) division factor for main system clock
		 * PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8
		 * NOT to exceed 100 MHz on this domain
		 * */
		/* Ponemos el PLLP en 2 */
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);

		/* Latencia del FLASH_ACR para que se demore y pueda hacer el registro */
		// 2 Wait states segun la tabla
		FLASH -> ACR &= ~(FLASH_ACR_LATENCY); // Limpiamos
		FLASH -> ACR |= (FLASH_ACR_LATENCY_2WS);
		break;
	case 1:
		/* Division factor for the main PLL (PLL) input clock
		 * VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
		 * VCO input frequency ranges from 1 to 2 MHz.*
		 */
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // Limpiamos
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLM_3); // Ponemos un 8 en el PLLM

		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); // Limpiamos
		/* Ponemos PLLN en 100 */
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_2);
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_5);
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_6);

		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP); // Limpiamos

		/* Latencia del FLASH_ACR para que se demore y pueda hacer el registro */
		// 3 Wait states
		FLASH -> ACR &= ~(FLASH_ACR_LATENCY); // Limpiamos
		FLASH -> ACR |= (FLASH_ACR_LATENCY_1WS);
		FLASH -> ACR |= (FLASH_ACR_LATENCY_2WS);

		/* Configuramos el MC01 (PIN A8 como funcion alternativa 00) */

		// Seleccionamos la senal PLL
		RCC -> CFGR |= RCC_CFGR_MCO1_0;
		RCC -> CFGR |= RCC_CFGR_MCO1_1;

		// Utilizamos un prescaler para poder ver la senal en el osciloscopio
		RCC -> CFGR |= RCC_CFGR_MCO1PRE_0;
		RCC -> CFGR |= RCC_CFGR_MCO1PRE_1;
		RCC -> CFGR |= RCC_CFGR_MCO1PRE_2;
		break;
	default:
		/* Division factor for the main PLL (PLL) input clock
		 * VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
		 * VCO input frequency ranges from 1 to 2 MHz.*
		 */
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // Limpiamos
		/*Ponemos PLLM en 8 */
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLM_3);

		/* Main PLL (PLL) multiplication factor for VCO
		 * VCO output frequency = VCO input frequency × PLLN with 50 ≤ PLLN ≤ 432
		 * VCO output frequency is between 100 and 432 MHz
		 * */
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); // Limpiamos
		/* Ponemos PLLN en 80 */
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_5);
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_7);

		/* Main PLL (PLL) division factor for main system clock
		 * PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8
		 * NOT to exceed 100 MHz on this domain
		 * */
		/* Ponemos el PLLP en 2 */
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);

		/* Latencia del FLASH_ACR para que se demore y pueda hacer el registro */
		// 2 Wait states segun la tabla
		FLASH -> ACR &= ~(FLASH_ACR_LATENCY); // Limpiamos
		FLASH -> ACR |= (FLASH_ACR_LATENCY_2WS);
		break;
	}


	// Encendemos el PLL
	RCC->CR |= RCC_CR_PLLON;

	// Esperamos que el PLL se cierre (estabilizacion)
	while (!(RCC->CR & RCC_CR_PLLRDY)){
		__NOP();
	}

	// Cambiamos el CPU Clock source cambiando los SW bits (System Clock Switch)
	/* System Clock Switch para PLL */
	RCC -> CFGR &= ~(RCC_CFGR_SW); // Limpiamos
	RCC -> CFGR |= (RCC_CFGR_SW_1);

}

/* Funcion para leer el registro con la informacion de la Frecuencia del micro */
int getConfigPLL(void){
	uint32_t frequencyValue =0;
	uint32_t RCCPLLn = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);

	if(RCC->CFGR | (RCC_CR_PLLON)){
		frequencyValue= (RCCPLLn*1000000);
	}else if(RCC->CFGR &= ~(RCC_CR_PLLON)){
		frequencyValue = (RCC->CFGR & RCC_CFGR_SW_HSI);
	}
	return frequencyValue;
}
