/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : RTCDriver.c
 * @brief    : Archivo de fuente del driver del periférico RTC
 * **************************************************************************************************
 */
#include <RTCDriver.h>

void RTC_Config(RTC_Config_t *RTCConfig) {

	RCC->APB1ENR |= RCC_APB1ENR_PWREN; 			// Se habilita señal de reloj para el periférico

	PWR->CR  |= PWR_CR_DBP;						// Se deshabilita la protección del write

	RCC->BDCR |= RCC_BDCR_RTCEN;		  		// Se le habilita el RTC clock
	RCC->BDCR |= RCC_BDCR_LSEON;		  		// Se habilita el LSE
	while(!(RCC->BDCR & RCC_BDCR_LSERDY)){
		__NOP() ;
	}

	RCC->BDCR |= RCC_BDCR_RTCSEL_0;		 		// Se selecciona el LSE como source clock

	RTC->WPR = (0xCA << RTC_WPR_KEY_Pos); // Se quita la proteccion contra escritura en todos los registros RTC
	RTC->WPR = (0x53 << RTC_WPR_KEY_Pos); // Key unlock write protection

	RTC->ISR |= RTC_ISR_INIT;			  // Se entra en el modo inicializacion
	while (!(RTC->ISR & RTC_ISR_INITF)){  //Se espera hasta que el modo inic empiece
		__NOP() ;
	}
	
	//Se ajustan los prescalers para el LSE
	RTC->PRER |= RTC_PRER_PREDIV_A;
	RTC->PRER |= 0xFF << 0 ;

	//Se cargan los datos iniciales y se configura el formato
    RTC->CR |= RTC_CR_BYPSHAD;//Para poder acceder a los registros

	if(RTCConfig->TRMod){
		RTC->TR = 0;//Se limpian
	}if(RTCConfig->DRMod){
		RTC->DR =0;//Se limpian
	}

	RTC->DR = 0;

	RTC->DR |= ((RTCConfig->RTC_Year) / 10) << RTC_DR_YT_Pos;  		// decena del año en BCD
	RTC->DR |= ((RTCConfig->RTC_Year) % 10) << RTC_DR_YU_Pos;  		// unidad del año BCD
	RTC->DR |= ((RTCConfig->RTC_WDay)) << RTC_DR_WDU_Pos;   		// dia de la semana en BCD
	RTC->DR |= ((RTCConfig->RTC_Month) / 10) << RTC_DR_MT_Pos;  		// decena del mes en BCD
	RTC->DR |= ((RTCConfig->RTC_Month) % 10) << RTC_DR_MU_Pos;  		// unidad del mes en BCD
	RTC->DR |= ((RTCConfig->RTC_DayValue) / 10) << RTC_DR_DT_Pos;   	// decena de dia del mes en BCD
	RTC->DR |= ((RTCConfig->RTC_DayValue) % 10) << RTC_DR_DU_Pos;    // unidad del dia del mes en BCD
	RTC->CR |= ((0) << RTC_CR_FMT_Pos);  	// 0: formato 24h
	RTC->TR |= ((RTCConfig->RTC_Hours) / 10) << RTC_TR_HT_Pos;  		// decena de la hora en BCD
	RTC->TR |= ((RTCConfig->RTC_Hours) % 10) << RTC_TR_HU_Pos;  		// unidad de la hora en BCD
	RTC->TR |= ((RTCConfig->RTC_Minutes) / 10) << RTC_TR_MNT_Pos;  	// decena de los minutos en BCD
	RTC->TR |= ((RTCConfig->RTC_Minutes) % 10) << RTC_TR_MNU_Pos;  	// unidad de los minutos en BCD
	RTC->TR |= ((RTCConfig->RTC_Seconds) / 10) << RTC_TR_ST_Pos;   	// decena de los segundos en BCD
	RTC->TR |= ((RTCConfig->RTC_Seconds) % 10) << RTC_TR_SU_Pos;   	// unidad de los segundos en BCD
	RTC->TR |= ((0) << RTC_TR_PM_Pos);  	// 0:formato 24h
	
	//Salir del modo inicializacion
	RCC->BDCR |= RCC_BDCR_RTCEN;//Se habilita el reloj
	RTC->ISR &= ~RTC_ISR_INIT;  //Se baja la flag para salir de inicialización
	PWR->CR &= ~ PWR_CR_DBP;	//Se habilita la protección
	RTC->CR &= ~RTC_CR_BYPSHAD; //Se deshabilita la modificacion de los shadow registers
	RTC->WPR = (0xFF); 			// Se bloquea el write protection
}

//Definición de las variables necesarias para  el RTC
uint8_t RTC_BcdToDec(uint16_t BCD_Value){
    return((BCD_Value/16*10) + (BCD_Value%16));
}

void RTC_read(uint8_t *arraySaveValues){

    //while((RTC->ISR & RTC_ISR_RSF)){

	 uint8_t RTC_Hours     = 0;
	 uint8_t RTC_Minutes = 0;
	 uint8_t RTC_Seconds = 0;

	// uint8_t weekday = 0;

	 uint8_t RTC_year = 0;
	 uint8_t RTC_Month = 0;
	 uint8_t RTC_Day = 0;

	 uint32_t RTC_Time = 0;
	 RTC_Time = RTC->TR;

	 uint32_t RTC_Date = 0;
	 RTC_Date = RTC->DR;

	 RTC_Hours   = RTC_BcdToDec(((RTC_Time & 0x3F0000) >> 16));
	 RTC_Minutes = RTC_BcdToDec(((RTC_Time & 0x007F00) >> 8));
	 RTC_Seconds = RTC_BcdToDec((RTC_Time  & 0x7F));
	 RTC_year   =  RTC_BcdToDec(((RTC_Date & 0xFF0000) >> 16));
	 RTC_Month  =  RTC_BcdToDec(((RTC_Date & 0x1F00)   >> 8));
	 RTC_Day    =  RTC_BcdToDec((RTC_Date  & 0x3F));


	arraySaveValues[0] = RTC_Seconds;
	arraySaveValues[1] = RTC_Minutes;
	arraySaveValues[2] = RTC_Hours;
	arraySaveValues[4] = RTC_Day;
	arraySaveValues[5] = RTC_Month;
	arraySaveValues[6] = RTC_year;
//    }
}
