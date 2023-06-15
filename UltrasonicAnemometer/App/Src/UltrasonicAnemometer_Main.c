/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : UltrasonicAnemometer_Main.c
 * @brief    : PROYECTO FINAL - TALLER V (2023-01)
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
#include "USARTxDriver.h"
#include "SysTickDriver.h"
#include "I2CDriver.h"
#include "ExtiDriver.h"

#include "arm_math.h"

/*	-	-	-	Definición de las macros a utilizar	-	-	-	*/

/*	-	-	-	Definición de handlers	-	-	-	*/

// Handlers de la comunicacion serial
GPIO_Handler_t handlerPinTX 			= {0};
GPIO_Handler_t handlerPinRX 			= {0};

USART_Handler_t handlerCommTerminal 	= {0};

// Handlers del led de estado (Blinky)
GPIO_Handler_t handlerLEDBlinky = {0};

// Handlers de los pines Trigger del HC-SR04
GPIO_Handler_t handlerTrigX		= {0};
GPIO_Handler_t handlerTrigY		= {0};

// Handlers de los pines Echo del HC-SR04
GPIO_Handler_t handlerEchoXRise		= {0};
GPIO_Handler_t handlerEchoXFall		= {0};

GPIO_Handler_t handlerEchoYRise		= {0};
GPIO_Handler_t handlerEchoYFall		= {0};

// Handlers para las interrupciones
EXTI_Config_t handlerExtiEchoXRise 		= {0};
EXTI_Config_t handlerExtiEchoXFall 		= {0};

EXTI_Config_t handlerExtiEchoYRise 		= {0};
EXTI_Config_t handlerExtiEchoYFall 		= {0};


// Handlers de los timers
BasicTimer_Handler_t handlerBlinkyTimer 	= {0};
BasicTimer_Handler_t handlerStopwatchX   	= {0};
BasicTimer_Handler_t handlerStopwatchY   	= {0};

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

// Variables auxiliares
uint64_t stopwatchX				= 0;
uint64_t stopwatchY				= 0;
float  timeOfFlightX			= 0;
float  timeOfFlightY			= 0;
float distanceX					= 0;
float distanceY					= 0;

/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void initSystem(void);
void tuneMCU(void);
void parseCommands(char *ptrBufferReception);
/*	=	=	=	INICIO DEL MAIN	=	=	=	*/
int main (void){
	// Se afina el micro y se establecen los parámetros de operación correctos
	tuneMCU();

	// Se inicializan todos los sistemas
	initSystem();


	// Se imprime un mensaje de inicio por la terminal serial
	writeMsg(&handlerCommTerminal, "EXAMEN FINAL - Taller V (2023-01)\n");
	writeMsg(&handlerCommTerminal, "Alejandro Rodriguez Montes \n");
	writeMsg(&handlerCommTerminal, "Command format is 'command #1 #2 #3 @' \n");
	writeMsg(&handlerCommTerminal, "Send command 'help @' to get a list of the available commands\n");


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
	// Se afina el micro para que el HSI quede lo más cercano posible a 16MHz.
	// Por tanteo se llegó a que un trim down de 5 daba los mejores resultados.
	RCC->CR &= ~(RCC_CR_HSITRIM);
	RCC->CR |= 13 << RCC_CR_HSITRIM_Pos;

	// Se activa el coprocesador matematico FPU
	SCB->CPACR |= (0xF << 20);

	// Se configura el systick a 16MHz
	config_SysTick_ms(0);
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
	handlerBlinkyTimer.TIMx_Config.TIMx_speed                = BTIMER_SPEED_1ms;
	handlerBlinkyTimer.TIMx_Config.TIMx_period               = 250;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración y se inicializa el timer
	BasicTimer_Config(&handlerBlinkyTimer);
	StartTimer(&handlerBlinkyTimer);

	// Configuracion del TIM3 para que cuente tiempo a incrementos de 10us (Stopwatch X)
	handlerStopwatchX.ptrTIMx                               = TIM3;
	handlerStopwatchX.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerStopwatchX.TIMx_Config.TIMx_speed                = BTIMER_SPEED_1us;
	handlerStopwatchX.TIMx_Config.TIMx_period               = 10;
	handlerStopwatchX.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerStopwatchX);

	// Configuracion del TIM4 para que cuente tiempo a incrementos de 10us (Stopwatch Y)
	handlerStopwatchY.ptrTIMx                               = TIM4;
	handlerStopwatchY.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerStopwatchY.TIMx_Config.TIMx_speed                = BTIMER_SPEED_1us;
	handlerStopwatchY.TIMx_Config.TIMx_period               = 10;
	handlerStopwatchY.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerStopwatchY);

	/*	-	-	-	Pines Trigger de los HC-SR04	-	-	-	*/

	/*	-	-	-	EJE X	-	-	-	*/

	handlerTrigX.pGPIOx												= GPIOB;
	handlerTrigX.GPIO_PinConfig.GPIO_PinNumber						= PIN_9;
	handlerTrigX.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_OUT;
	handlerTrigX.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerTrigX.GPIO_PinConfig.GPIO_PinPuPdControl					= GPIO_PUPDR_NOTHING;
	handlerTrigX.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerTrigX.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	// Se carga la configuración y se inicializa en 0
	GPIO_Config(&handlerTrigX);
	GPIO_WritePin(&handlerTrigX, RESET);

	/*	-	-	-	EJE Y	-	-	-	*/

	handlerTrigY.pGPIOx												= GPIOC;
	handlerTrigY.GPIO_PinConfig.GPIO_PinNumber						= PIN_9;
	handlerTrigY.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_OUT;
	handlerTrigY.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerTrigY.GPIO_PinConfig.GPIO_PinPuPdControl					= GPIO_PUPDR_NOTHING;
	handlerTrigY.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerTrigY.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	// Se carga la configuración y se inicializa en 0
	GPIO_Config(&handlerTrigY);
	GPIO_WritePin(&handlerTrigY, RESET);

	/*	-	-	-	Pines Echo de los HC-SR04	-	-	-	*/

	/*	-	-	-	EJE X	-	-	-	*/

	handlerEchoXRise.pGPIOx												= GPIOB;
	handlerEchoXRise.GPIO_PinConfig.GPIO_PinNumber						= PIN_4;
	handlerEchoXRise.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_IN;
	handlerEchoXRise.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerEchoXRise.GPIO_PinConfig.GPIO_PinPuPdControl					= GPIO_PUPDR_NOTHING;
	handlerEchoXRise.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerEchoXRise.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	handlerEchoXFall.pGPIOx												= GPIOB;
	handlerEchoXFall.GPIO_PinConfig.GPIO_PinNumber						= PIN_5;
	handlerEchoXFall.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_IN;
	handlerEchoXFall.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerEchoXFall.GPIO_PinConfig.GPIO_PinPuPdControl					= GPIO_PUPDR_NOTHING;
	handlerEchoXFall.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerEchoXFall.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoXRise);
	GPIO_Config(&handlerEchoXFall);

	// Se configura la exti de los Echo
	handlerExtiEchoXRise.edgeType 		= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiEchoXRise.pGPIOHandler	= &handlerEchoXRise;

	handlerExtiEchoXFall.edgeType 		= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoXFall.pGPIOHandler	= &handlerEchoXFall;

	// Se carga la configuración
	extInt_Config(&handlerExtiEchoXRise);
	extInt_Config(&handlerExtiEchoXFall);

	/*	-	-	-	EJE Y	-	-	-	*/

	handlerEchoYRise.pGPIOx												= GPIOC;
	handlerEchoYRise.GPIO_PinConfig.GPIO_PinNumber						= PIN_6;
	handlerEchoYRise.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_IN;
	handlerEchoYRise.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerEchoYRise.GPIO_PinConfig.GPIO_PinPuPdControl					= GPIO_PUPDR_NOTHING;
	handlerEchoYRise.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerEchoYRise.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	handlerEchoYFall.pGPIOx												= GPIOB;
	handlerEchoYFall.GPIO_PinConfig.GPIO_PinNumber						= PIN_15;
	handlerEchoYFall.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_IN;
	handlerEchoYFall.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerEchoYFall.GPIO_PinConfig.GPIO_PinPuPdControl					= GPIO_PUPDR_NOTHING;
	handlerEchoYFall.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerEchoYFall.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoYRise);
	GPIO_Config(&handlerEchoYFall);

	// Se configura la exti de los Echo
	handlerExtiEchoYRise.edgeType 		= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiEchoYRise.pGPIOHandler	= &handlerEchoYRise;

	handlerExtiEchoYFall.edgeType 		= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoYFall.pGPIOHandler	= &handlerEchoYFall;

	// Se carga la configuración
	extInt_Config(&handlerExtiEchoYRise);
	extInt_Config(&handlerExtiEchoYFall);

	/*	-	-	-	Comunicación serial	-	-	-	*/
	handlerPinTX.pGPIOx                               = GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber        = PIN_2;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF7;

	// Se carga la configuración
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx                               = GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber        = PIN_3;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF7;

	// Se carga la configuración
	GPIO_Config(&handlerPinRX);

	handlerCommTerminal.ptrUSARTx                       = USART2;
	handlerCommTerminal.USART_Config.USART_baudrate     = USART_BAUDRATE_115200;
	handlerCommTerminal.USART_Config.USART_datasize     = USART_DATASIZE_8BIT;
	handlerCommTerminal.USART_Config.USART_parity       = USART_PARITY_NONE;
	handlerCommTerminal.USART_Config.USART_stopbits     = USART_STOPBIT_1;
	handlerCommTerminal.USART_Config.USART_mode         = USART_MODE_RXTX;
	handlerCommTerminal.USART_Config.USART_enableIntRX  = USART_RX_INTERRUP_ENABLE;
	handlerCommTerminal.USART_Config.USART_enableIntTX  = USART_TX_INTERRUP_DISABLE;
	handlerCommTerminal.USART_Config.USART_frequency    = 16;

	// Se carga la configuración
	USART_Config(&handlerCommTerminal);

}

	/*	-	-	-	Protocolo I2C	-	-	-	*/

	/*	=	=	=	INICIO DE LA DEFINICIÓN DE FUNCIONES	=	=	=	*/

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
		writeMsg(&handlerCommTerminal, "1) help\n");
		writeMsg(&handlerCommTerminal, "Print this menu\n");
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "2) measureTOF\n");
		writeMsg(&handlerCommTerminal, "Measures time of flight of sound to a given obstacle\n");
		writeMsg(&handlerCommTerminal, "\n");
	}

	// 2) measureTOF. Permite medir el tiempo de vuelo del sonido hasta un obstáculo
	else if(strcmp(cmd,"measureTOF") == 0){

		// Se manda un pulso ultrasónico...
		GPIO_WritePin(&handlerTrigX, SET);
		delay_ms(1);
		GPIO_WritePin(&handlerTrigX, RESET);

		delay_ms(9);
		timeOfFlightX = stopwatchX / 200.0;

		distanceX = (348.2*timeOfFlightX/1000)*100;


		sprintf(bufferData,"X-AXIS: time of flight %.5f ms ; distance %.2f cm\n", timeOfFlightX,distanceX);
		writeMsg(&handlerCommTerminal, bufferData);

		stopwatchX = 0;
		timeOfFlightX = 0;
		distanceX = 0;

		delay_ms(50);

		// Se manda un pulso ultrasónico...
		GPIO_WritePin(&handlerTrigY, SET);
		delay_ms(1);
		GPIO_WritePin(&handlerTrigY, RESET);

		delay_ms(9);
		timeOfFlightY = stopwatchY / 200.0;

		distanceY = (348.2*timeOfFlightY/1000)*100;


		sprintf(bufferData,"Y-AXIS: time of flight %.5f ms ; distance %.2f cm\n", timeOfFlightY,distanceY);
		writeMsg(&handlerCommTerminal, bufferData);

		stopwatchY = 0;
		timeOfFlightY = 0;
		distanceY = 0;
	}
	else{
		// Se imprime el mensaje "Wrong CMD" si la escritura no corresponde a los CMD implementados
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "Wrong CMD\n");
		writeMsg(&handlerCommTerminal, "\n");
	}
}

/*	=	=	=	INICIO DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
/* Callbacks de la transmisión serial */
void usart2Rx_Callback(void){
	rxData = getRxData();
}

/* Callbacks de los Timers */
void BasicTimer2_Callback(void){
	// Timer encargado del blinky
	GPIOxTooglePin(&handlerLEDBlinky);
}

void BasicTimer3_Callback(void){
	// Timer encargado de cronometrar
	stopwatchX++;
}

void BasicTimer4_Callback(void){
	// Timer encargado de cronometrar
	stopwatchY++;
}

void callback_extInt4(void){
	// Callback rise X
	StartTimer(&handlerStopwatchX);
}

void callback_extInt5(void){
	// Callback fall X
	StopTimer(&handlerStopwatchX);
}

void callback_extInt6(void){
	// Callback rise Y
	StartTimer(&handlerStopwatchY);
}

void callback_extInt15(void){
	// Callback fall Y
	StopTimer(&handlerStopwatchY);
}

/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
