/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : RTCDriver.h
 * @brief    : Archivo de cabecera del driver del periférico RTC
 * **************************************************************************************************
 */


#ifndef RTCDRIVER_H_
#define RTCDRIVER_H_

#include <stm32f4xx.h>


#define TIME_NOTATION_AM_OR_24	0
#define TIME_NOTATION_PM		1

#define  FORMAT_24	0
#define  FORMAT_12	1

#define MONDAY					1
#define TUESDAY 				2
#define WEDNESDAY				3
#define THURSDAY				4
#define FRIDAY					5
#define SATURDAY				6
#define SUNDAY					7

typedef struct{ /**Estructura principal RTC**/
uint16_t RTC_Seconds; 		//Segundos
uint16_t RTC_Minutes; 		//Minutos
uint16_t RTC_Hours;			//Horas
uint16_t RTC_Month; 		//Mes
uint16_t RTC_Year;			//Año
uint16_t RTC_WDay; 			//Dia de la semana
uint16_t RTC_DayValue; 		//Configuración valor día
uint8_t TRMod;
uint8_t DRMod;

}RTC_Config_t;


//Cabeceras de funciones
void RTC_Config(RTC_Config_t *RTCConfig);
void RTC_read(uint8_t *arraySaveValues);
uint8_t RTC_BcdToDec(uint16_t BCD_Value);

#endif /* RTCDRIVER_H_ */
