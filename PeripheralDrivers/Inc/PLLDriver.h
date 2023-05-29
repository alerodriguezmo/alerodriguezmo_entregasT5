/*
 * PLLDriver.h
 *
 *  Created on: May 20, 2023
 *      Author: majo
 */

#ifndef PLLDRIVER_H_
#define PLLDRIVER_H_

#include "stm32f4xx.h"

#define FREQUENCY_80_MHz	0
#define FREQUENCY_100_MHz	1

void configPLL(uint8_t frequency);
int getConfigPLL(void);

#endif /* PLLDRIVER_H_ */
