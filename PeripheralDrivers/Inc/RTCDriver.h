/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : RTCDriver.h
 * @brief    : Archivo de cabecera del driver del periférico RTC (Real Time Clock)
 * **************************************************************************************************
 */

#ifndef RTCDRIVER_H_
#define RTCDRIVER_H_

#include <stm32f4xx.h>

#define TIME_NOTATION_AM_OR_24	0
#define TIME_NOTATION_PM		1

#define  TIME_FORMAT_24_HOUR	0
#define  TIME_FORMAT_AM_OR_PM	1

#define MONDAY					1
#define TUESDAY 				2
#define WEDNESDAY				3
#define THURSDAY				4
#define FRIDAY					5
#define SATURDAY				6
#define SUNDAY					7

typedef struct{ /**Configuración RTC**/
uint16_t RTC_Hours; //Configuración de las horas
uint16_t RTC_Minutes; //Configuración de los minutos
uint16_t RTC_Seconds; //Configuración de los segundos
uint16_t RTC_Year; //Configuración del año
uint16_t RTC_WeekDay; //Configuración del día de la semana
uint16_t RTC_TimeFormat; // Configuración del formato
uint16_t RTC_TimeNotation; //Configuración de la notación
uint16_t RTC_Month; //Configuración del mes
uint16_t RTC_ValueDay; //Configuración valor día

}RTC_Config_t;

typedef struct{
	RTC_TypeDef 	*ptrRTC; //Dirección
	RTC_Config_t	RTC_Config; //Configuración

}RTC_Handler_t;

//Cabecera función
void RTC_Config(RTC_Handler_t *ptrRtcHandler);
void *read_date(void);

#endif /* RTCDRIVER_H_ */
