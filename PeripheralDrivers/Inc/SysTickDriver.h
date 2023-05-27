/*
 *************************************************************************
 * @file		: SysTickDriver.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Archivo de cabecera del driver del SysTick
 *
 *************************************************************************
 */

#ifndef 	INC_SYSTICKDRIVER_H_
#define 	INC_SYSTICKDRIVER_H_

#include 	<stm32f4xx.h>

#define 	SYSTICK_LOAD_VALUE_16MHz_1ms 	16000
#define 	SYSTICK_LOAD_VALUE_100MHz_1ms 	100000

void 		config_SysTick_ms(uint8_t systemClock);
uint64_t 	getTicks_ms(void);
void 		delay_ms(uint32_t wait_time_ms);


#endif /* INC_SYSTICKDRIVER_H_ */
