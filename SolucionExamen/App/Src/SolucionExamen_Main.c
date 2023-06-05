/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : SoluciónExamen_Main.c
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
#include "DriverLCD.h"

/*	-	-	-	Definición de las macros a utilizar	-	-	-	*/

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


/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void initSystem(void);
void parseCommands(char *ptrBufferReception);
/*	=	=	=	INICIO DEL MAIN	=	=	=	*/
int main (void){
	// Se afina el micro para que el HSI quede lo más cercano posible a 100MHz.
	// Por tanteo se llegó a que un trim down de 5 daba los mejores resultados.
	// Más adelante se implementa una función que permite establecer un trim definido por el usuario.
	RCC->CR &= ~(RCC_CR_HSITRIM);
	RCC->CR |= 13 << RCC_CR_HSITRIM_Pos;

	// Se activa el coprocesador matematico FPU
	SCB->CPACR |= (0xF << 20);

	// Se inicializan todos los sistemas
	initSystem();

	// Se establece la frecuencia del micro en 100MHz
	configPLL(FREQUENCY_100_MHz);

	// Se imprime un mensaje de inicio por la terminal serial
	writeMsg(&handlerCommTerminal, "EXAMEN FINAL - Taller V (2023-01) \n"
			"Alejandro Rodriguez Montes \n"
			"Escriba el comando 'help @' para ver los comandos disponibles \n");

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
		writeMsg(&handlerCommTerminal, "1) help								Print this menu\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "2) setTrimHSI #						Set # as the trim value for the HSI (0 < # < 20)\n");
		writeMsg(&handlerCommTerminal, "This command can be used  by the user to manualy calibrate the HSI (MCUs internal clock signal)\n");
		writeMsg(&handlerCommTerminal, "to compensate the frequency variaton caused by external factors.\n");
		writeMsg(&handlerCommTerminal, "This trim can take values between 0 and 20, and each unitary increase or decrease is equal to 48kHz (approximatley)\n");
		writeMsg(&handlerCommTerminal, "The MCU is calibrated by default with a trim of 13, aimed at an HSI frequency of 100MHz.\n");
		writeMsg(&handlerCommTerminal, "It's strongly recommended to keep the trim at 13. Nevertheless, the user can adjust accordingly\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "3) getTrimHSI						Returns the current value of the HSI trim\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "4) testLcd							Simple test for the LCD\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "5) setPeriod #						Change the Led_state period (us)\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "6) autoUpdate #						Automatic LCD update (# -> 1/0)\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "7) Descripción del septimo comando\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "8) Descripción del octavo comando\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "9) Descripción del noveno comando\n");
		writeMsg(&handlerCommTerminal, "\n");
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
			sprintf(bufferData, "Trim set at %u succesfully. Default is 13.\n", firstParameter);
			writeMsg(&handlerCommTerminal, bufferData);
		}
	}

	// 3) getTrimHSI. Le permite al usuario conocer el valor actual del trim del HSI
	else if(strcmp(cmd,"getTrimHSI") == 0){
		sprintf(bufferData, "The current trim value is %u. Default is 13.\n", (int)RCC->CR >>3 & 0b11111);
		writeMsg(&handlerCommTerminal, bufferData);
	}

	else{
		// Se imprime el mensaje "Wrong CMD" si la escritura no corresponde a los CMD implementados
		writeMsg(&handlerCommTerminal, "Wrong CMD\n");
	}
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

/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
