/*
 *************************************************************************
 * @file		: PLLDriver.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Archivo de fuente del driver del periférico PLL
 *
 *************************************************************************
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <PLLDriver.h>

void PLL_Config(PLL_Config_t *PLLConfig){
	//1. Se configuran los parametros del PLL
	//1.1 Se configura la señal que se usará de referencia
	if(PLLConfig->PLL_src == 1){//HSI es 1
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);//Se selecciona el HSI como señal de referencia (bit 22=0)
	}else{
		RCC->PLLCFGR |= (1<<RCC_PLLCFGR_PLLSRC_Pos);//Se selecciona el HSE como señal de referencia  (bit 22=1)
	}
	//Se activa el power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	//1.6 Se configura el PWR_CR para que el micro para que se regulen bien los voltajes en las frecuencias
	PWR->CR |=  (0b10<<PWR_CR_VOS_Pos);
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR |= FLASH_ACR_ICEN;
	FLASH->ACR |= FLASH_ACR_DCEN;

	//1.7 Se le configuran los wait states al micro para que la memoria flash le pueda seguir el ritmo.
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS;

	//1.2 Se configura el PLLM, factor de división para la frecuencia de la señal de referencia para configurar la VCOInput freq
	//    debe estar en [1,2]MHz, se recomienda 2
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM; //Se limpia todo el pllm
	RCC->PLLCFGR |= (PLLConfig->PLL_pllm<<RCC_PLLCFGR_PLLM_Pos);   //Se escribe el valor deseado

	//1.3 Se configura el PLLN, factor de multiplicación para la frecuencia de salida del VCO. La salida debe pertenecer a [100,432]MHz.
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN; 		//Se limpia el plln
	RCC->PLLCFGR |= (PLLConfig->PLL_plln<<RCC_PLLCFGR_PLLN_Pos);  //Se escribe el valor en el plln

	//1.4 Se configura el PLLP Factor de división para la frecuencia de salida del VCO, da el valor de salida del PLL, <100MHz
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;//Se limpia
	switch(PLLConfig->PLL_pllp){//Se escribe dependiendo el utilizado
		case 2:{
			RCC->PLLCFGR |= (0b00<<RCC_PLLCFGR_PLLP_Pos);
			break;
		}
		case 4:{
			RCC->PLLCFGR |= (0b01<<RCC_PLLCFGR_PLLP_Pos);
			break;
		}
		case 6:{
			RCC->PLLCFGR |= (0b10<<RCC_PLLCFGR_PLLP_Pos);
			break;
		}
		case 8:{
			RCC->PLLCFGR |= (0b11<<RCC_PLLCFGR_PLLP_Pos);
			break;
		}
		default:{
			break;}
	}

	RCC->CFGR &= ~RCC_CFGR_HPRE;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Con esto para el bus AHB dividimos por 1, queda en 80 MHz, o lo mismo del PLLFreqMHz

	//1.5 Se configura el prescaler del APB1 para que no exceda los 50MHz
	RCC->CFGR &=~RCC_CFGR_PPRE1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;//se divide por dos, puesto que es máx 100MHz lo que puede dar el reloj

	//Para APB2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	//Para que se pueda usar el MCO1
	RCC->CFGR |= RCC_CFGR_MCO1; // Con este para el MCO1 uso la PLL
	RCC->CFGR &= ~RCC_CFGR_MCO1PRE;
	RCC->CFGR |= RCC_CFGR_MCO1PRE; // Con esta macro, divido los 80MHz por 5, para tener 16MHz en el pin MCO1




	//2. Se habilita el PLL
	RCC->CR |= RCC_CR_PLLON;
	//3. Se espera hasta que el PLL esté listo
	while( !(RCC->CR & RCC_CR_PLLRDY)){
		__NOP();
	}
	//4. Se selecciona al PLL como reloj principal

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_1;
}

int PLL_GetConfig(PLL_Config_t *PLLConfig){

	int freq = 0;
	if(PLLConfig->PLL_src == 1){
		freq = PLLConfig->PLL_plln*16/(PLLConfig->PLL_pllm*PLLConfig->PLL_pllp);
		return freq;
	}else{
		return freq;
	}
}


