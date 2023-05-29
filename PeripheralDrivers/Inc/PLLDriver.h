/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : PLLDriver.h
 * @brief    : Archivo de cabecera del driver del periférico PLL
 * **************************************************************************************************
 */

#ifndef PLLDRIVER_H_
#define PLLDRIVER_H_

#include "stm32f4xx.h"

#define FREQUENCY_80_MHz	0
#define FREQUENCY_100_MHz	1

void configPLL(uint8_t frequency);
int getConfigPLL(void);

#endif /* PLLDRIVER_H_ */
