/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : SysTick.h
 * @brief    : Archivo de cabecera del driver del SysTick
 * **************************************************************************************************
 */

#ifndef SYSTICKDRIVER_H_
#define SYSTICKDRIVER_H_

#include <stm32f4xx.h>

#define SYSTICK_LOAD_VALUE_16MHz_1ms     16000   // Numero de ciclos en un milisegundo
#define SYSTICK_LOAD_VALUE_100MHz_1ms    100000  // Numero de ciclos en un milisegundo
#define SYSTICK_LOAD_VALUE_80MHz_1ms     80000   // Numero de ciclos en un milisegundo

void config_SysTick_ms(uint8_t systemClock);
uint64_t getTicks_ms(void);
void delay_ms(uint32_t wait_time_ms);

#endif /* SYSTICKDRIVER_H_ */
