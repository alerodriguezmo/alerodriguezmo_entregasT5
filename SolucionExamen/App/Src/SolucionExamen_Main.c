/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : SolucionExamen_Main.c
 * @brief    : EXAMEN FINAL - TALLER V (2023-01)
 * **************************************************************************************************
 */

/*	-	-	-	Importación de drivers y elementos necesarios	-	-	-	*/

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "SysTickDriver.h"
#include "I2CDriver.h"
#include "PwmDriver.h"
#include "PLLDriver.h"
#include "RTCDriver.h"

#include "arm_math.h"

/*	-	-	-	Definición de las macros a utilizar	-	-	-	*/

// Macros para la implementación del acelerómetro
#define ACCEL_ADDRESS          	 0x1D
#define ACCEL_XOUT_L             50
#define ACCEL_XOUT_H             51
#define ACCEL_YOUT_L             52
#define ACCEL_YOUT_H             53
#define ACCEL_ZOUT_L             54
#define ACCEL_ZOUT_H             55

#define BW_RATE                  44
#define POWER_CTL                45
#define WHO_AM_I                 0

// Macros para la implementación de la FFT
#define FFT_SIZE 				 1024

/*	-	-	-	Definición de handlers	-	-	-	*/

// Handlers de la comunicacion serial
GPIO_Handler_t handlerPinTX 			= {0};
GPIO_Handler_t handlerPinRX 			= {0};

USART_Handler_t handlerCommTerminal 	= {0};

// Handler del led de estado (Blinky)
GPIO_Handler_t handlerLEDBlinky = {0};
GPIO_Handler_t handlerMCO		= {0};

// Handlers de los timers
BasicTimer_Handler_t handlerBlinkyTimer = {0};
BasicTimer_Handler_t handlerSampling	= {0};

// Handlers del protocolo I2C
GPIO_Handler_t handlerI2cSDA = {0};
GPIO_Handler_t handlerI2cSCL = {0};

I2C_Handler_t handlerAccelerometer = {0};

// Handler del RTC
RTC_Config_t rtcConfig = {0};

/*	-	-	-	Definición de variables	-	-	-	*/

// Variables de la comunicación serial por comandos
uint8_t rxData 					= 0;
uint8_t counterReception		= 0;
bool stringComplete 			= false;
int firstParameter				= 0;
int secondParameter				= 0;
int thirdParameter				= 0;
char bufferReception[64]		= {0};
char cmd[64]					= {0};
char bufferData[64]				= {0};

// Variables para la toma de datos con el acelerómetro
uint8_t AccelX_low 		= 0;
uint8_t AccelX_high 	= 0;
int16_t AccelX 			= 0;
uint8_t AccelY_low 		= 0;
uint8_t AccelY_high 	= 0;
int16_t AccelY 			= 0;
uint8_t AccelZ_low		= 0;
uint8_t AccelZ_high 	= 0;
int16_t AccelZ 			= 0;

uint8_t i2cBuffer 		= {0};
uint32_t dataCounter 	= 0;
uint8_t flagSampling	= 0;
uint8_t samplingEnable	= 0;

float x,y,z;
float32_t dataAccelX[600] = {0};
float32_t dataAccelY[600] = {0};
float32_t dataAccelZ[600] = {0};

// Variables para la implementación de la FFT
arm_status statusInitFFT = ARM_MATH_ARGUMENT_ERROR;

// Variables para el RTC
uint8_t calendar[6] = {0};

// Variables para el SysTick
uint8_t systemTicks = 0;
uint8_t systemTicksStart = 0;
uint8_t systemTicksEnd = 0;

// Variables auxiliares
uint8_t flag_sampleAccel = 0;

/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void initSystem(void);
void tuneMCU(void);
void parseCommands(char *ptrBufferReception);
void sampleAccel(void);
/*	=	=	=	INICIO DEL MAIN	=	=	=	*/
int main (void){
	// Se afina el micro y se establecen los parámetros de operación correctos
	tuneMCU();

	// Se inicializan todos los sistemas
	initSystem();

	// Se establece la frecuencia del micro en 100MHz
	configPLL(FREQUENCY_100_MHz);

	// Se imprime un mensaje de inicio por la terminal serial
	writeMsg(&handlerCommTerminal, "EXAMEN FINAL - Taller V (2023-01) \n"
			"Alejandro Rodriguez Montes \n"
			"Escriba el comando 'help @' para ver los comandos disponibles \n");

	// Encendemos el acelerómetro, muestreando a 1.6KHz
	i2c_writeSingleRegister(&handlerAccelerometer, BW_RATE , 0xF);
	i2c_writeSingleRegister(&handlerAccelerometer, POWER_CTL , 0b00001000);

	while(1){

		/*	-	-	-	Comunicación por comandos	-	-	-	*/
		if (rxData != '\0'){
			bufferReception[counterReception] = rxData;
			counterReception++;

			// Si el caracter que llega representa un cambio de línea, se levanta una
			// bandera para el loop main
			if (rxData == '@'){
				stringComplete = true;

				// Se agrega el caracter nulo al final del string
				bufferReception[counterReception] = '\0';

				counterReception = 0;
			}

			// Para que no vuelva a entrar. Solo cambia debido a la interrupción
			rxData = '\0';
		}

		// Hacemos un análisis de la cadena de datos obtenida
		else if (stringComplete){
			parseCommands(bufferReception);
			stringComplete = false;
		}
	}
	return(0);
}
/*	=	=	=	FIN DEL MAIN	=	=	=	*/

/*	=	=	=	INICIO DE LA DEFINICIÓN DE FUNCIONES	=	=	=	*/

/*	-	-	-	Función que afina el MCU y establece parámetros de operación	-	-	-	*/
void tuneMCU(void){
	// Se afina el micro para que el HSI quede lo más cercano posible a 100MHz.
	// Por tanteo se llegó a que un trim down de 5 daba los mejores resultados.
	// Más adelante se implementa una función que permite establecer un trim definido por el usuario.
	RCC->CR &= ~(RCC_CR_HSITRIM);
	RCC->CR |= 13 << RCC_CR_HSITRIM_Pos;

	// Inicialización del LSE

	// Se establece el escalado del APB1 para que reciba los 50MHz que tiene de frequencia máxima
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	// Se enciende la señal de reloj del APB1
	RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN);
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// Se deshabilita la protección de escritura
	PWR->CR &= ~(PWR_CR_DBP);
	PWR->CR |= PWR_CR_DBP;
	// Se enciende el LSE
	RCC->BDCR |= RCC_BDCR_LSEON;
	// Se espera hasta que el LSE este listo
	while(!(RCC->BDCR & RCC_BDCR_LSERDY)){
		__NOP();
	}
	// Se habilita la protección de escritura
	PWR->CR &= ~(PWR_CR_DBP);

	// Se establece la señal por defecto del MCO1 como el PLL y el preescalado en 5.

	// Selección del PLL
	RCC->CFGR &= ~(RCC_CFGR_MCO1);
	RCC->CFGR |= 0b00 << RCC_CFGR_MCO1_Pos;

	// Selección del preescalado en 5
	RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
	RCC->CFGR |= 0b111 << RCC_CFGR_MCO1PRE_Pos;

	// Se activa el coprocesador matematico FPU
	SCB->CPACR |= (0xF << 20);

	// Se configura el systick a 100MHz
	config_SysTick_ms(2);
}

/*	-	-	-	Función que inicializa los elementos del sistema	-	-	-	*/
void initSystem(void){

	/*	-	-	-	Led de estado (Blinky)	-	-	-	*/

	// Configuracion del LED2
	handlerLEDBlinky.pGPIOx                             = GPIOA;
	handlerLEDBlinky.GPIO_PinConfig.GPIO_PinNumber      = PIN_5;
	handlerLEDBlinky.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerLEDBlinky.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerLEDBlinky.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEED_FAST;
	handlerLEDBlinky.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;

	// Se carga la configuración y se enciende por defecto
	GPIO_Config(&handlerLEDBlinky);
	GPIO_WritePin(&handlerLEDBlinky, SET);

	/*	-	-	-	Timers	-	-	-	*/

	// Configuracion del TIM2 para que haga un blinky cada 250ms
	handlerBlinkyTimer.ptrTIMx                               = TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_10us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period               = 25000;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerBlinkyTimer);

	handlerSampling.ptrTIMx                               = TIM4;
	handlerSampling.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerSampling.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_100us;
	handlerSampling.TIMx_Config.TIMx_period               = 50;
	handlerSampling.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerSampling);

	/*	-	-	-	Pin para medir la frecuencia del micro (MCO_1)	-	-	-	*/
	handlerMCO.pGPIOx								= GPIOA;
	handlerMCO.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerMCO.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerMCO.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerMCO.GPIO_PinConfig.GPIO_PinAltFunMode	= AF0;

	// Se carga la configuración
	GPIO_Config(&handlerMCO);


	/*	-	-	-	Comunicación serial	-	-	-	*/
	handlerPinTX.pGPIOx                               = GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber        = PIN_11;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF8;

	// Se carga la configuración
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx                               = GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber        = PIN_12;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF8;

	// Se carga la configuración
	GPIO_Config(&handlerPinRX);

	handlerCommTerminal.ptrUSARTx                       = USART6;
	handlerCommTerminal.USART_Config.USART_baudrate     = USART_BAUDRATE_115200;
	handlerCommTerminal.USART_Config.USART_datasize     = USART_DATASIZE_8BIT;
	handlerCommTerminal.USART_Config.USART_parity       = USART_PARITY_NONE;
	handlerCommTerminal.USART_Config.USART_stopbits     = USART_STOPBIT_1;
	handlerCommTerminal.USART_Config.USART_mode         = USART_MODE_RXTX;
	handlerCommTerminal.USART_Config.USART_enableIntRX  = USART_RX_INTERRUP_ENABLE;
	handlerCommTerminal.USART_Config.USART_enableIntTX  = USART_TX_INTERRUP_DISABLE;
	handlerCommTerminal.USART_Config.USART_frequency    = 100;

	// Se carga la configuración
	USART_Config(&handlerCommTerminal);

	/*	-	-	-	Protocolo I2C	-	-	-	*/

	// Pines del acelerómetro
	handlerI2cSCL.pGPIOx                                      = GPIOB;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinNumber               = PIN_8;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinMode                 = GPIO_MODE_ALTFN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinOPType               = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinSpeed                = GPIO_OSPEED_FAST;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinPuPdControl          = GPIO_PUPDR_PULLUP;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinAltFunMode           = AF4;
	GPIO_Config(&handlerI2cSCL);

	handlerI2cSDA.pGPIOx                                      = GPIOB;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinNumber               = PIN_9;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinMode                 = GPIO_MODE_ALTFN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinOPType               = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinSpeed                = GPIO_OSPEED_FAST;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinPuPdControl          = GPIO_PUPDR_PULLUP;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinAltFunMode           = AF4;
	GPIO_Config(&handlerI2cSDA);

	// Configuración del acelerómetro
	handlerAccelerometer.ptrI2Cx                            = I2C1;
	handlerAccelerometer.modeI2C                            = I2C_MODE_FM;
	handlerAccelerometer.slaveAddress                       = ACCEL_ADDRESS;
	handlerAccelerometer.mainClock							= MAIN_CLOCK_100_MHz_FOR_I2C;
	handlerAccelerometer.maxI2C_FM							= I2C_MAX_RISE_TIME_FM_100MHz;
	handlerAccelerometer.modeI2C_FM							= I2C_MODE_FM_SPEED_400KHz_100MHz;

	i2c_config(&handlerAccelerometer);
}

/*	-	-	-	Función dedicada a los comandos	-	-	-	*/
void parseCommands(char *ptrBufferReception){

	/* Esta función lee la cadena de caracteres a la que apunta el puntero
	 * y almacena en tres elementos diferentes: Un string llamado "cmd", y dos
	 * integers llamados "firstParameter y secondParameter.
	 *
	 * De esta forma, podemos introducir información al micro desde el puerto serial*/
	sscanf(ptrBufferReception, "%s %u %u %u", cmd, &firstParameter, &secondParameter, &thirdParameter);

	// 1) help. Imprime una lista con todos los comandos disponibles
	if(strcmp(cmd, "help") == 0){
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "HELP MENU - LIST OF AVAILABLE COMMANDS:\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "1) help - - - - - - - - - - - - - - - - - Print this menu\n");
		writeMsg(&handlerCommTerminal, "2) setTrimHSI # - - - - - - - - - - - - - Set # as the trim value for the HSI (0 < # < 20)\n");
		writeMsg(&handlerCommTerminal, "This command can be used  by the user to manually calibrate the HSI (MCUs internal clock signal)\n");
		writeMsg(&handlerCommTerminal, "to compensate the frequency changes caused by external factors.\n");
		writeMsg(&handlerCommTerminal, "This trim can take values between 0 and 20, and each unit increase or decrease is equal to 48kHz (approximately)\n");
		writeMsg(&handlerCommTerminal, "The MCU is calibrated by default with a trim of 13, aimed at an HSI frequency of 100MHz.\n");
		writeMsg(&handlerCommTerminal, "It's strongly recommended to keep the trim at 13. Nevertheless, the user can adjust accordingly.\n");
		writeMsg(&handlerCommTerminal, "3) getTrimHSI - - - - - - - - - - - - - - Returns the current value of the HSI trim\n");
		writeMsg(&handlerCommTerminal, "4) setMCO1Channel # - - - - - - - - - - - Set # as the active channel for the MCO1 (PA8). 1->HSI ; 2->LSE ; 3->PLL. Set as 3 by default.\n");
		writeMsg(&handlerCommTerminal, "5) setMCO1PreScaler # - - - - - - - - - - Set # as the division factor for the active channel of MCO1 (PA8). # = {1, 2, 3, 4, 5}. Set as 5 by default\n");
		writeMsg(&handlerCommTerminal, "6) setDate #DD #MM #YY - - - - - - - - -  Set #DD/#MM/#YY as the current day for the RTC.\n");
		writeMsg(&handlerCommTerminal, "7) setTime #hh #mm #ss - - - - - - - - -  Set #hh:#mm:#ss as the current time (in 24h format) for the RTC.\n");
		writeMsg(&handlerCommTerminal, "8) getDate - - - - - - - - - - - - - - -  Returns the current date of the RTC.\n");
		writeMsg(&handlerCommTerminal, "9) getTime - - - - - - - - - - - - - - -  Returns the current time of the RTC (in 24h format).\n");
		writeMsg(&handlerCommTerminal, "10) getDateTime - - - - - - - - - - - - - Returns the current date and time of the RTC (in 24h format).\n");
		writeMsg(&handlerCommTerminal, "9) sampleAccel - - - - - - - - - - - - -  Sample and store 6 seconds worth acceleration values at 200Hz. Needed before using fireUpFFT\n");
		writeMsg(&handlerCommTerminal, "10) fireUpFFT - - - - - - - - - - - - - - Use a Fast Fourier Transform to get a frequency using data from sampleAccel.\n");
		writeMsg(&handlerCommTerminal, "11) showAccel - - - - - - - - - - - - - - Shows a 20 value sample for the acceleration data\n");
		writeMsg(&handlerCommTerminal, "12) EMPTY\n");
		writeMsg(&handlerCommTerminal, "13) EMPTY\n");
		writeMsg(&handlerCommTerminal, "\n");
	}

	// 2) trimHSI #. Le permite al usuario ajustar la frecuencia del HSI.
	else if(strcmp(cmd, "setTrimHSI") == 0){
		// Por si el usuario ingresa un número fuera del rango establecido
		if(firstParameter < 0){
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Wrong input. Remember that trim values\n");
			writeMsg(&handlerCommTerminal, "can only be between 0 and 20\n");
			writeMsg(&handlerCommTerminal, "\n");
		}
		else if(firstParameter > 20){
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Wrong input. Remember that trim values\n");
			writeMsg(&handlerCommTerminal, "can only be between 0 and 20\n");
			writeMsg(&handlerCommTerminal, "\n");
		}
		// Se hace el ajuste si todo está correcto
		else{
			RCC->CR &= ~(RCC_CR_HSITRIM);
			RCC->CR |= firstParameter << RCC_CR_HSITRIM_Pos;
			sprintf(bufferData, "Trim set at %u successfully. Default is 13.\n", firstParameter);
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, bufferData);
			writeMsg(&handlerCommTerminal, "\n");

			// Se limpia el parametro
			firstParameter = 0;
		}
	}

	// 3) getTrimHSI. Le permite al usuario conocer el valor actual del trim del HSI
	else if(strcmp(cmd,"getTrimHSI") == 0){
		sprintf(bufferData, "The current trim value is %u. Default is 13.\n", (int)RCC->CR >>3 & 0b11111);
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, bufferData);
		writeMsg(&handlerCommTerminal, "\n");
	}

	// 4) setMCO1Channel #. Le permite al usuario establecer el canal activo para el MCO1 (PA8)
	else if(strcmp(cmd,"setMCO1Channel") == 0){
		if(firstParameter == 1){
			RCC->CFGR &= ~(RCC_CFGR_MCO1);
			RCC->CFGR |= 0b00 << RCC_CFGR_MCO1_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Active channel for the MCO1 (PA8) successfully set as HSI\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else if(firstParameter == 2){
			// Se activa el LSE
			RCC->BDCR &= ~(RCC_BDCR_LSEON);
			RCC->BDCR |= 0b1 << RCC_BDCR_LSEON_Pos;
			// Se selecciona el LSE
			RCC->CFGR &= ~(RCC_CFGR_MCO1);
			RCC->CFGR |= 0b01 << RCC_CFGR_MCO1_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Active channel for the MCO1 (PA8) successfully set as LSE\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else if(firstParameter == 3){
			RCC->CFGR &= ~(RCC_CFGR_MCO1);
			RCC->CFGR |= 0b11 << RCC_CFGR_MCO1_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Active channel for the MCO1 (PA8) successfully set as PLL\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else{
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Channel selection not valid.\n");
			writeMsg(&handlerCommTerminal, "\n");
		}
	}

	// 5) setMCO1PreScaler # Le permite al usuario establecer el pre escalado del canal activo en el MCO1 (PA8)
	else if(strcmp(cmd,"setMCO1PreScaler") == 0){
		if(firstParameter == 1){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b0 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Division factor for the active channel of the MCO1 (PA8) successfully set as 1\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else if(firstParameter == 2){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b100 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Division factor for the active channel of the MCO1 (PA8) successfully set as 2\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else if(firstParameter == 3){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b101 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Division factor for the active channel of the MCO1 (PA8) successfully set as 3\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else if(firstParameter == 4){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b110 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Division factor for the active channel of the MCO1 (PA8) successfully set as 4\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else if(firstParameter == 5){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b111 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Division factor for the active channel of the MCO1 (PA8) successfully set as 5\n");
			writeMsg(&handlerCommTerminal, "\n");
			// Se limpia el parametro
			firstParameter = 0;
		}
		else{
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Division factor selection not valid.\n");
			writeMsg(&handlerCommTerminal, "\n");
		}

	}
	// 8) setDate #dd #mm #yy. Permite establecer la fecha del RTC
	else if(strcmp(cmd,"setDate") == 0){
		// Se evalua el formato para el año
		if(thirdParameter >= 0 && thirdParameter <= 99){
			// Formato correcto para el año. Se evalúa el formato para cada mes.

			if(secondParameter == 1){
				// Formato correcto para enero. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 31){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 31 for the month selected (January).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 2){
				// Formato correcto para febrero. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 28){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 28 for the month selected (Feburary).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 3){
				// Formato correcto para marzo. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 31){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 31 for the month selected (March).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 4){
				// Formato correcto para abril. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 30){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 30 for the month selected (April).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 5){
				// Formato correcto para mayo. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 31){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 31 for the month selected (May).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 6){
				// Formato correcto para junio. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 30){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 30 for the month selected (June).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 7){
				// Formato correcto para julio. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 31){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 31 for the month selected (July).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 8){
				// Formato correcto para agosto. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 31){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 31 for the month selected (August).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 9){
				// Formato correcto para septiembre. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 30){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 30 for the month selected (September).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 10){
				// Formato correcto para octubre. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 31){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 31 for the month selected (October).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 11){
				// Formato correcto para noviembre. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 30){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 30 for the month selected (November).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else if(secondParameter == 12){
				// Formato correcto para diciembre. Se evalúa el formato de los días
				if(firstParameter >= 1 && firstParameter <= 31){
					// Formato válido para configuración. Se procede a configurar.
					rtcConfig.RTC_DayValue = firstParameter;
					rtcConfig.RTC_Month = secondParameter;
					rtcConfig.RTC_Year = thirdParameter;
					rtcConfig.DRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.DRMod = 0;

					sprintf(bufferData,"Date successfully set as %u/%u/%u\n", firstParameter, secondParameter, thirdParameter);
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, bufferData);
					writeMsg(&handlerCommTerminal, "\n");

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los días
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid day format. #DD ranges between 1 and 31 for the month selected (December).\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else{
				// Formato inválido para los meses
				writeMsg(&handlerCommTerminal, "\n");
				writeMsg(&handlerCommTerminal, "Invalid month format. #MM ranges between 1 and 12 (January through December).\n");
				writeMsg(&handlerCommTerminal, "\n");
			}

		}
		else{
			// Formato inválido para los años
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Invalid year format. #YY ranges between 00 and 99 and represents years between 2000 and 2099.\n");
			writeMsg(&handlerCommTerminal, "\n");
		}
	}

	// 7) setTime. Permite establecer la hora para el RTC en formato militar
	else if(strcmp(cmd,"setTime") == 0){
		// Verificación para el formato de las horas
		if(firstParameter >= 0 && firstParameter <= 23 ){
			// Formato válido para las horas
			// Verificación del formato de minutos
			if(secondParameter >= 0 && secondParameter <= 59){
				//Formato válido para los minutos
				// Verificación del formato de los segundos
				if(thirdParameter >= 0 && thirdParameter <= 59){
					// Formato válido para los segundos
					// Se realiza la configuración
					rtcConfig.RTC_Hours = firstParameter;
					rtcConfig.RTC_Minutes = secondParameter;
					rtcConfig.RTC_Seconds = thirdParameter;
					rtcConfig.TRMod = 1;
					RTC_Config(&rtcConfig);
					rtcConfig.TRMod = 0;
					sprintf(bufferData, "Time successfully set as %u:%u:%u (24h format)\n",firstParameter,secondParameter,thirdParameter);
					writeMsg(&handlerCommTerminal, bufferData);

					// Se reinician los parámetros
					firstParameter = 0;
					secondParameter = 0;
					thirdParameter = 0;
				}
				else{
					// Formato inválido para los segundos
					writeMsg(&handlerCommTerminal, "\n");
					writeMsg(&handlerCommTerminal, "Invalid seconds format. #ss ranges between 0 and 59.\n");
					writeMsg(&handlerCommTerminal, "\n");
				}
			}
			else{
				// Formato inválido para los minutos
				writeMsg(&handlerCommTerminal, "\n");
				writeMsg(&handlerCommTerminal, "Invalid minutes format. #mm ranges between 0 and 59.\n");
				writeMsg(&handlerCommTerminal, "\n");
			}
		}
		else{
			// Formato inválido para las horas
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Invalid hours format. #hh ranges between 0 and 24.\n");
			writeMsg(&handlerCommTerminal, "\n");
		}
	}
	// 8) getDate. Entrega la fecha actual del RTC
	else if(strcmp(cmd,"getDate") == 0){
			RTC_read(calendar);
			sprintf(bufferData,"Current date set as: %u/%u/%u\n",calendar[4],calendar[5],calendar[6]);
			writeMsg(&handlerCommTerminal,bufferData);
	}

	// 9) getTime. Entrega la hora actual del RTC
	else if(strcmp(cmd,"getTime") == 0){
			RTC_read(calendar);
			sprintf(bufferData,"Current time (in 24h format) set as: %u:%u:%u\n",calendar[2],calendar[1],calendar[0]);
			writeMsg(&handlerCommTerminal,bufferData);
		}

	// 10) getDateTime. Entrega la hora y fecha actuales del RTC
	else if(strcmp(cmd,"getDateTime") == 0){
		RTC_read(calendar);
		sprintf(bufferData,"Current date and time (in 24h format) set as: %u/%u/%u  %u:%u:%u\n",calendar[4],calendar[5],calendar[6],calendar[2],calendar[1],calendar[0]);
		writeMsg(&handlerCommTerminal,bufferData);
	}
	// 8) sampleAccel. Toma 600 datos de aceleración
	else if(strcmp(cmd,"sampleAccel") == 0){
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "Sampling acceleration...\n");
		writeMsg(&handlerCommTerminal, "\n");
		// Se habilita el muestreo
		samplingEnable = 1;

		// Muestreo a 200Hz por 3 segundos
		while(dataCounter < 600){
			if(flagSampling){
				// Se muestrea
				sampleAccel();
				// Se almacenan los datos
				dataAccelX[dataCounter] = x;
				dataAccelY[dataCounter] = y;
				dataAccelZ[dataCounter] = z;

				dataCounter++;
				flagSampling = 0;
				if(dataCounter == 600){
					break;
				}
		    }
	    }
		samplingEnable = 0;
		flag_sampleAccel = 1;
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "Data acquired and stored successfully!\n");
		writeMsg(&handlerCommTerminal, "Command 'fireUpFFT' unlocked and ready to use.\n");
		writeMsg(&handlerCommTerminal, "\n");
     }
	// 9) fireUpFFT. Permite al usuario calcular la frecuencia fundamental detectada por el acelerómetro
	else if(strcmp(cmd,"fireUpFFT") == 0){
		if(flag_sampleAccel){
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "Making Fourier proud...\n");
			writeMsg(&handlerCommTerminal, "\n");

			// Se definen los buffer de entrada y de salida
			float32_t outputX[1024] = {0}, outputY[1024] = {0},outputZ[1024] = {0};

			// Se definen las variables para obtener los máximos de cada eje, el máximo absoluto y la frecuencia predominante
			float32_t	maxValueX = 0, maxValueY = 0, maxValueZ = 0;
			uint32_t 	maxIndexX = 0, maxIndexY = 0, maxIndexZ = 0, maxIndex = 0;
			uint16_t 	dominantFreq = 420;

			// Se castean los valores obtenidos para el acelerómetro
//			for(int i; i < 600; i++){
//				inputX[i] = dataAccelX[i];
//				inputY[i] = dataAccelY[i];
//				inputZ[i] = dataAccelZ[i];
//			}

			// Se inicializa la transformada
			arm_rfft_fast_instance_f32 fftInstance;
			statusInitFFT = arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE);


			if(statusInitFFT == ARM_MATH_SUCCESS){
				// Se ejecuta la transformada para cada uno de los ejes
							arm_rfft_fast_f32(&fftInstance, dataAccelX, outputX, 0);
							arm_rfft_fast_f32(&fftInstance, dataAccelY, outputY, 0);
							arm_rfft_fast_f32(&fftInstance, dataAccelZ, outputZ, 0);

							// Se saca la magnitud para cada transformada
							arm_cmplx_mag_f32(outputX, outputX, FFT_SIZE / 2);
							arm_cmplx_mag_f32(outputY, outputY, FFT_SIZE / 2);
							arm_cmplx_mag_f32(outputZ, outputZ, FFT_SIZE / 2);

							// Se obtienen los máximos
							arm_max_f32(outputX, FFT_SIZE / 2, &maxValueX, &maxIndexX);
							arm_max_f32(outputY, FFT_SIZE / 2, &maxValueY, &maxIndexY);
							arm_max_f32(outputZ, FFT_SIZE / 2, &maxValueZ, &maxIndexZ);

							// Se saca el máximo absoluto

							// Se establece X por defecto
							maxIndex = maxIndexX;
							if(maxValueY > maxValueX){
								//Y le gana a X
								maxIndex = maxIndexY;
							}
							else if(maxValueZ > maxValueX){
								//Z le gana a X
								maxIndex = maxIndexY;
							}

							// Se calcula la frecuencia dominante
							dominantFreq = (maxIndex*200) / FFT_SIZE;
							sprintf(bufferData, "The dominant frequency in the data is %u", dominantFreq);
							delay_ms(1500);

							writeMsg(&handlerCommTerminal, "\n");
							writeMsg(&handlerCommTerminal, "FFT executed successfully!\n");
							writeMsg(&handlerCommTerminal, "\n");
							writeMsg(&handlerCommTerminal, bufferData);
							writeMsg(&handlerCommTerminal, "\n");


			}else{
				writeMsg(&handlerCommTerminal, "\n");
				writeMsg(&handlerCommTerminal, "FFT not initialized\n");
				writeMsg(&handlerCommTerminal, "\n");
			}
		}

		else{
			writeMsg(&handlerCommTerminal, "\n");
			writeMsg(&handlerCommTerminal, "No data available to perform a FFT\n");
			writeMsg(&handlerCommTerminal, "Execute command 'sampleAccel' to unlock this command.\n");
			writeMsg(&handlerCommTerminal, "\n");
		}
	}
	// 10) showAccel
	else if(strcmp(cmd, "showAccel") == 0){
		for (int i = 0;i<20; i++){
			sprintf(bufferData, "Accel = x %.5f; y %.5f; z %.5f ; #Datum %d \n", dataAccelX[i], dataAccelY[i], dataAccelZ[i], i+1);
            writeMsg(&handlerCommTerminal, bufferData);
		}
	}
	else{
		// Se imprime el mensaje "Wrong CMD" si la escritura no corresponde a los CMD implementados
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "Wrong CMD\n");
		writeMsg(&handlerCommTerminal, "\n");
	}
}

// Función que retorna los valores de la acceleración en los 3 ejes del acelerómetro
void sampleAccel(void){
	AccelX_low =  i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_L);
	AccelX_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_H);
	AccelX = AccelX_high << 8 | AccelX_low;
	AccelY_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_L);
	AccelY_high = i2c_readSingleRegister(&handlerAccelerometer,ACCEL_YOUT_H);
	AccelY = AccelY_high << 8 | AccelY_low;
	AccelZ_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_L);
	AccelZ_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_H);
	AccelZ = AccelZ_high << 8 | AccelZ_low;

	// Conversión a m/s²
	x = (AccelX/256.f)*9.78;
	y = (AccelY/256.f)*9.78;
	z = (AccelZ/256.f)*9.78;
}

/*	=	=	=	FIN DE LA DEFINICIÓN DE FUNCIONES	=	=	=	*/

/*	=	=	=	INICIO DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/

/* Callbacks de la transmisión serial */
void usart6Rx_Callback(void){
	rxData = getRxData();
}

/* Callbacks de los Timers */
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerLEDBlinky);
}

void BasicTimer4_Callback(void){
	if(samplingEnable){
		flagSampling = 1;
	}
}

/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
