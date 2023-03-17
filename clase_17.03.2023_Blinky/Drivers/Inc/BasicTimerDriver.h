/*
 * BasicTimerDriver.h
 *
 *  Created on: Mar 17, 2023
 *      Author: alerodriguezmo
 */

#ifndef INC_BASICTIMERDRIVER_H_
#define INC_BASICTIMERDRIVER_H_

#include "stm32f4xx.h"

#define BTIMER_MODE_UP		0
#define BTIMER_MODE_DOWN	1

#define BTIMER_SPEED_10us	160
#define BTIMER_SPEED_100us	1600
#define BTIMER_SPEED_1ms	16000



/* Estructura que contiene la configuración mínima necesaria para el manejo del Timer.*/
typedef struct
{
	uint8_t		TIMx_mode; 		// Up or dowm
	uint32_t	TIMx_speed;		// A qué velocidad se incrementa el Timer
	uint32_t	TIMx_period;	// Valor en ms del periodo del Timer
	uint8_t		TIMx_interruptEnable;	// Activa o desactiva el modo interrupción del timer.
}BasicTimer_Config_t;

/* Handler para el Timer*/
typedef struct
{
	TIM_TypeDef			*ptrTIMx;
	BasicTimer_Config_t	TIMx_Config;
}BasicTimer_Handler_t;

void BasicTimer_Config(BasicTimer_Handler_t *ptrBTimerHandler);
void BasicTimerX_Callback(void); /* Esta función debe ser sobre-escrita en el main para que el sistema funcione*/

#endif /* INC_BASICTIMERDRIVER_H_ */
