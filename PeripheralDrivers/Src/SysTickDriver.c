/*
 *************************************************************************
 * @file		: SysTickDriver.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Archivo de fuente del driver del SysTick
 *
 *************************************************************************
 */


#include <stm32f4xx.h>
#include "SysTickDriver.h"

uint64_t ticks 			= 0;
uint64_t ticks_start 	= 0;
uint64_t ticks_counting = 0;

void config_SysTick_ms(uint8_t systemClock){
	//Se resetea el contador
	ticks = 0;
	//Cargando el valor del limite de incrementos que representan 1ms
	switch(systemClock){
	//Clock HSI->16MHz
	case 0:{
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;
	}
	//HSE Clock
	case 1:{
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;
	}
	//PLL Clock at 100MHz
	case 2:{
		SysTick->LOAD = SYSTICK_LOAD_VALUE_100MHz_1ms;
		break;
	}
	//PLL Clock at 80MHz
	case 3:{
		SysTick->LOAD = 80000;
		break;
	}
	default:
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;
	}

	//Se limpia el valor actual del SysTick
	SysTick->VAL = 0;

	//SE CONFIGURA RELOJ INTERNO COMO RELOJ PARA EL TIMER
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	//Se desactivan las interrupciones globales
	__disable_irq();
	//Se matricula la interrupción en el NVIC
	NVIC_EnableIRQ(SysTick_IRQn);
	//Se activa la interrupción debida al conteo a cero del SysTick
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	//Se activa el timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	//Se activa interrupciones globales again
	__enable_irq();
}

uint64_t getTicks_ms(void){
	return ticks;
}

void delay_ms(uint32_t wait_time_ms){
	//Primer valor de tiempo para comparar
	ticks_start = getTicks_ms();
	//Segundo valor de tiempo para comparar
	ticks_counting = getTicks_ms();
	//Actualiza el valor de ticks counting hasta que llegue al wait time
	while(ticks_counting < (ticks_start +(uint64_t)wait_time_ms)){
		ticks_counting = getTicks_ms();
	}
}

void SysTickHandler(void){
	//Se verifica si la interrupción se lanzó
	if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk){
		//Se limpia la bandera
		SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
		//Incrementa en 1 el contador
		ticks++;
	}
}
