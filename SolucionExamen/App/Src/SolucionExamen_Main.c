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

/*	-	-	-	Definición de las macros a utilizar	-	-	-	*/

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

/*	-	-	-	Definición de variables	-	-	-	*/

// Variables de la comunicación serial por comandos
uint8_t rxData 					= 0;
uint8_t counterReception		= 0;
bool stringComplete 			= false;
int firstParameter				= 0;
int secondParameter				= 0;
char bufferReception[64]		= {0};
char cmd[64]					= {0};
char userMsg[64]				= {0};
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
uint32_t interrupt	 	= 0;
uint8_t flag			= 0;

float x,y,z;
float dataAccel[3][200];

// Variables para el SysTick
uint8_t systemTicks = 0;
uint8_t systemTicksStart = 0;
uint8_t systemTicksEnd = 0;


/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void initSystem(void);
void tuneMCU(void);
void parseCommands(char *ptrBufferReception);
void saveDatum(void);
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
		if (stringComplete){
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
	handlerSampling.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_10us;
	handlerSampling.TIMx_Config.TIMx_period               = 500;
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
	sscanf(ptrBufferReception, "%s %u %u %s", cmd, &firstParameter, &secondParameter, userMsg);

	// 1) help. Imprime una lista con todos los comandos disponibles
	if(strcmp(cmd, "help") == 0){
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
		writeMsg(&handlerCommTerminal, "4) setMCO1Channel # - - - - - - - - - - - Set # as the active channel for the MCO1 (PA8). 0->HSI	1->LSE	2->PLL. Set as 2 by default.\n");
		writeMsg(&handlerCommTerminal, "5) setMCO1PreScaler # - - - - - - - - - - Set # as the division factor for the active channel of MCO1 (PA8). # = {1, 2, 3, 4, 5}. Set as 5 by default\n");
		writeMsg(&handlerCommTerminal, "6) sampleAccel - - - - - - - - - - - - -  Sample  and show 200 acceleration values\n");
		writeMsg(&handlerCommTerminal, "7) Descripción del septimo comando\n");
		writeMsg(&handlerCommTerminal, "8) Descripción del octavo comando\n");
		writeMsg(&handlerCommTerminal, "9) Descripción del noveno comando\n");
		writeMsg(&handlerCommTerminal, "10) Descripción del decimo comando\n");
	}

	// 2) trimHSI #. Le permite al usuario ajustar la frecuencia del HSI.
	else if(strcmp(cmd, "setTrimHSI") == 0){
		// Por si el usuario ingresa un número fuera del rango establecido
		if(firstParameter < 0){
			writeMsg(&handlerCommTerminal, "Wrong input. Remember that trim values\n");
			writeMsg(&handlerCommTerminal, "can only be between 0 and 20\n");
		}
		else if(firstParameter > 20){
			writeMsg(&handlerCommTerminal, "Wrong input. Remember that trim values\n");
			writeMsg(&handlerCommTerminal, "can only be between 0 and 20\n");
		}
		// Se hace el ajuste si todo está correcto
		else{
			RCC->CR &= ~(RCC_CR_HSITRIM);
			RCC->CR |= firstParameter << RCC_CR_HSITRIM_Pos;
			sprintf(bufferData, "Trim set at %u successfully. Default is 13.\n", firstParameter);
			writeMsg(&handlerCommTerminal, bufferData);
		}
	}

	// 3) getTrimHSI. Le permite al usuario conocer el valor actual del trim del HSI
	else if(strcmp(cmd,"getTrimHSI") == 0){
		sprintf(bufferData, "The current trim value is %u. Default is 13.\n", (int)RCC->CR >>3 & 0b11111);
		writeMsg(&handlerCommTerminal, bufferData);
	}

	// 4) setMCO1Channel #. Le permite al usuario establecer el canal activo para el MCO1 (PA8)
	else if(strcmp(cmd,"setMCO1Channel") == 0){
		if(firstParameter == 0){
			RCC->CFGR &= ~(RCC_CFGR_MCO1);
			RCC->CFGR |= 0b00 << RCC_CFGR_MCO1_Pos;
			writeMsg(&handlerCommTerminal, "Active channel for the MCO1 (PA8) successfully set as HSI\n");
		}
		else if(firstParameter == 1){
			// Se activa el LSE
			RCC->BDCR &= ~(RCC_BDCR_LSEON);
			RCC->BDCR |= 0b1 << RCC_BDCR_LSEON_Pos;
			// Se selecciona el LSE
			RCC->CFGR &= ~(RCC_CFGR_MCO1);
			RCC->CFGR |= 0b01 << RCC_CFGR_MCO1_Pos;
			writeMsg(&handlerCommTerminal, "Active channel for the MCO1 (PA8) successfully set as LSE\n");
		}
		else if(firstParameter == 2){
			RCC->CFGR &= ~(RCC_CFGR_MCO1);
			RCC->CFGR |= 0b11 << RCC_CFGR_MCO1_Pos;
			writeMsg(&handlerCommTerminal, "Active channel for the MCO1 (PA8) successfully set as PLL\n");
		}
		else{
			writeMsg(&handlerCommTerminal, "Channel selection not valid.\n");
		}
	}

	// 5) setMCO1PreScaler # Le permite al usuario establecer el pre escalado del canal activo en el MCO1 (PA8)
	else if(strcmp(cmd,"setMCO1PreScaler") == 0){
		if(firstParameter == 1){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b0 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "Division factor for the active channel for the MCO1 (PA8) successfully set as 1\n");
		}
		else if(firstParameter == 2){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b100 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "Division factor for the active channel for the MCO1 (PA8) successfully set as 2\n");
		}
		else if(firstParameter == 3){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b101 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "Division factor for the active channel for the MCO1 (PA8) successfully set as 3\n");
		}
		else if(firstParameter == 4){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b110 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "Division factor for the active channel for the MCO1 (PA8) successfully set as 4\n");
		}
		else if(firstParameter == 5){
			RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
			RCC->CFGR |= 0b111 << RCC_CFGR_MCO1PRE_Pos;
			writeMsg(&handlerCommTerminal, "Division factor for the active channel for the MCO1 (PA8) successfully set as 5\n");
		}
		else{
			writeMsg(&handlerCommTerminal, "Division factor selection not valid.\n");
		}

	}

	// 6) sampleAccel. Imprime 200 datos de aceleración
	else if(strcmp(cmd,"sampleAccel") == 0){
		// Se samplean 200 datos
		writeMsg(&handlerCommTerminal, "Sampling data...\n" );
		saveDatum();

		sprintf(bufferData, "Axis Z data \n");
		writeMsg(&handlerCommTerminal, bufferData);
		sprintf(bufferData, "AccelZ = %.2f m/s^2 \n", z);
		writeMsg(&handlerCommTerminal, bufferData);
	}
	else{
		// Se imprime el mensaje "Wrong CMD" si la escritura no corresponde a los CMD implementados
		writeMsg(&handlerCommTerminal, "Wrong CMD\n");
	}
}
void saveDatum(void){
	flag = 1;
	if(interrupt <= 200){
		AccelX_low =  i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_L);
		AccelX_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_H);
		AccelX = AccelX_high << 8 | AccelX_low;
		AccelY_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_L);
		AccelY_high = i2c_readSingleRegister(&handlerAccelerometer,ACCEL_YOUT_H);
		AccelY = AccelY_high << 8 | AccelY_low;
		AccelZ_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_L);
		AccelZ_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_H);
		AccelZ = AccelZ_high << 8 | AccelZ_low;
	}

	// Conversión a m/s²
	x = (AccelX/256.f)*9.78;
	y = (AccelY/256.f)*9.78;
	z = (AccelZ/256.f)*9.78;
    dataAccel[0][interrupt] = x;
    dataAccel[1][interrupt] = y;
    dataAccel[2][interrupt] = z;

    flag = 0;
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
	if(flag){
		interrupt++;
		if(interrupt == 200){
			interrupt = 0;
		}
	}
}

/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
