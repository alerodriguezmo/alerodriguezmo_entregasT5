/*
 *************************************************************************
 * @file		: PLLDriver.h
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Archivo de cabecera del driver del periférico PLL
 *
 *************************************************************************
 */

#ifndef INC_PLLDRIVER_H_
#define INC_PLLDRIVER_H_

typedef struct
{
	uint8_t PLL_src;	//Selección de señal de referencia
	uint8_t PLL_pllm;	//Factor de división para el input clock, 2<=PLLM<=63
	uint8_t PLL_plln;	//Factor de multiplicación para el VCO, 50<=PLLN<=432
	uint8_t PLL_pllp;	//Factor de division para el VCO_outputfreq. No debe superar los 100MHz
	uint8_t PLL_pllq;	//Factor de división para el USB

}PLL_Config_t;

//Estructura que contiene la direccion del puerto que se utiliza y la configuracion especifica del pin

void PLL_Config(PLL_Config_t *PLLConfig); //Func. para configurar el PLL con la input del usuario
int PLL_GetConfig(PLL_Config_t *PLLConfig);
//Func. para obtener la config. del PLL

#endif /* INC_PLLDRIVER_H_ */
