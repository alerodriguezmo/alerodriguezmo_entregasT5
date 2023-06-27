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
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "SysTickDriver.h"
#include "I2CDriver.h"
#include "PwmDriver.h"
#include "PLLDriver.h"
#include "DriverLCD.h"

#include "arm_math.h"

/*	-	-	-	Definición de las macros a utilizar	-	-	-	*/

#define SHT30_ADDRESS			 0x45
#define LCD_ADDRESS				 0x24

/*	-	-	-	Definición de handlers	-	-	-	*/

// Handlers de la comunicacion serial
GPIO_Handler_t handlerPinTX 			= {0};
GPIO_Handler_t handlerPinRX 			= {0};

USART_Handler_t handlerCommTerminal 	= {0};

// Handlers del led de estado (Blinky)
GPIO_Handler_t handlerLEDBlinky = {0};

// Handlers de los timers
BasicTimer_Handler_t handlerBlinkyTimer 	= {0};
BasicTimer_Handler_t handlerStopwatch   	= {0};

// Handlers del protocolo I2C
GPIO_Handler_t handlerI2cSDA 		= {0};
GPIO_Handler_t handlerI2cSCL 		= {0};
GPIO_Handler_t handlerI2cLcdSCL 	= {0};
GPIO_Handler_t handlerI2cLcdSDA 	= {0};
I2C_Handler_t handlerSHT30 			= {0};
I2C_Handler_t handlerLCD 			= {0};

// Handlers de los pines Trigger de los HC-SR04
GPIO_Handler_t handlerTrigX1		= {0};
GPIO_Handler_t handlerTrigX2		= {0};

GPIO_Handler_t handlerTrigY1		= {0};
GPIO_Handler_t handlerTrigY2		= {0};

// Handlers de los pines Echo de los HC-SR04
GPIO_Handler_t handlerEchoFallX1		= {0};
GPIO_Handler_t handlerEchoFallX2		= {0};
GPIO_Handler_t handlerEchoFallY1		= {0};
GPIO_Handler_t handlerEchoFallY2		= {0};

EXTI_Config_t handlerExtiEchoFallX1		= {0};
EXTI_Config_t handlerExtiEchoFallX2		= {0};
EXTI_Config_t handlerExtiEchoFallY1		= {0};
EXTI_Config_t handlerExtiEchoFallY2		= {0};


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

// Variables para el protocolo I2C
char state 				= 0;
char hum1 				= 0;
char hum2 				= 0;
char humAndTemp 		= 0;
char temp1 				= 0;
char temp2 				= 0;
char crc 				= 0;

char bufferTemp[64];
char bufferHum[64];
float temperature_read = 0;
float humidity_read 	= 0;
float temperature 	= 0;
float humidity	 	= 0;

uint8_t counterLCD = 0;
uint8_t counterSampling = 0;

uint8_t i2cBuffer 		= {0};

// Variables auxiliares
uint64_t stopwatch				= 0;

float  timeOfFlightAB			= 0;
float  timeOfFlightBA			= 0;
float distanceX1				= 0;
float distanceX2				= 0;

float  timeOfFlightCD			= 0;
float  timeOfFlightDC			= 0;
float distanceY1				= 0;
float distanceY2				= 0;

float Vx						= {0};
float Vx_mean					= {0};

float Vy						= {0};
float Vy_mean					= {0};

float32_t V							= 0;
float32_t squares					= 0;

float atan_arg					= 0;
float Direction					= 0;


/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void initSystem(void);
void tuneMCU(void);
void parseCommands(char *ptrBufferReception);
void measureTemp_Hum(void);
/*	=	=	=	INICIO DEL MAIN	=	=	=	*/
int main (void){
	// Se afina el micro y se establecen los parámetros de operación correctos
	tuneMCU();

	// Se inicializan todos los sistemas
	initSystem();

	// Se inicializa la pantala LCD
	clearScreenLCD(&handlerLCD);

	init_LCD(&handlerLCD);
	delay_10(); // DELAYS CON EL SYSTICK IMPLEMENTADOS EN EL DRIVER DE LA LCD
	clearLCD(&handlerLCD);
	delay_10();

	// Esctirura de los caracteres permanentes

	// Para el eje X
	moveCursor_inLCD(&handlerLCD, 0, 1);
	sendSTR_toLCD(&handlerLCD, "Temp = ");
	moveCursor_inLCD(&handlerLCD, 0, 14);
	sendSTR_toLCD(&handlerLCD, "C");

	// Para el eje Y
	moveCursor_inLCD(&handlerLCD, 1, 1);
	sendSTR_toLCD(&handlerLCD, "Humidity = ");
	moveCursor_inLCD(&handlerLCD, 1, 17);
	sendSTR_toLCD(&handlerLCD, "%");

	delay_ms(20);

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

		// Ciclo que muestrea temperatura y humedad
		if(counterSampling > 2){
			measureTemp_Hum();
		}
		// Ciclo que permite actualizar las lecturas en pantalla
		if(counterLCD > 4){
			sprintf(bufferTemp,"%.2f",temperature);
			sprintf(bufferHum,"%.2f", humidity);

			moveCursor_inLCD(&handlerLCD, 0, 8);
			sendSTR_toLCD(&handlerLCD, bufferTemp);
			moveCursor_inLCD(&handlerLCD, 1, 12);
			sendSTR_toLCD(&handlerLCD, bufferHum);

			counterLCD = 0;
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
	handlerStopwatch.ptrTIMx                               = TIM4;
	handlerStopwatch.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerStopwatch.TIMx_Config.TIMx_speed                = BTIMER_SPEED_1us;
	handlerStopwatch.TIMx_Config.TIMx_period               = 2;
	handlerStopwatch.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerStopwatch);

	/*	-	-	-	Protocolo I2C	-	-	-	*/

	// Pin Clock (SCL)
	handlerI2cSCL.pGPIOx                                      = GPIOB;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinNumber               = PIN_8;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinMode                 = GPIO_MODE_ALTFN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinOPType               = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinSpeed                = GPIO_OSPEED_FAST;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinPuPdControl          = GPIO_PUPDR_PULLUP;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinAltFunMode           = AF4;
	GPIO_Config(&handlerI2cSCL);

	// Pin Data (SDA)
	handlerI2cSDA.pGPIOx                                      = GPIOB;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinNumber               = PIN_9;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinMode                 = GPIO_MODE_ALTFN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinOPType               = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinSpeed                = GPIO_OSPEED_FAST;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinPuPdControl          = GPIO_PUPDR_PULLUP;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinAltFunMode           = AF4;
	GPIO_Config(&handlerI2cSDA);

	// Pines de la pantalla LCD
	handlerI2cLcdSCL.pGPIOx                                      = GPIOA;
	handlerI2cLcdSCL.GPIO_PinConfig.GPIO_PinNumber               = PIN_8;
	handlerI2cLcdSCL.GPIO_PinConfig.GPIO_PinMode                 = GPIO_MODE_ALTFN;
	handlerI2cLcdSCL.GPIO_PinConfig.GPIO_PinOPType               = GPIO_OTYPE_OPENDRAIN;
	handlerI2cLcdSCL.GPIO_PinConfig.GPIO_PinSpeed                = GPIO_OSPEED_FAST;
	handlerI2cLcdSCL.GPIO_PinConfig.GPIO_PinPuPdControl          = GPIO_PUPDR_PULLUP;
	handlerI2cLcdSCL.GPIO_PinConfig.GPIO_PinAltFunMode           = AF4;
	GPIO_Config(&handlerI2cLcdSCL);

	handlerI2cLcdSDA.pGPIOx                                      = GPIOC;
	handlerI2cLcdSDA.GPIO_PinConfig.GPIO_PinNumber               = PIN_9;
	handlerI2cLcdSDA.GPIO_PinConfig.GPIO_PinMode                 = GPIO_MODE_ALTFN;
	handlerI2cLcdSDA.GPIO_PinConfig.GPIO_PinOPType               = GPIO_OTYPE_OPENDRAIN;
	handlerI2cLcdSDA.GPIO_PinConfig.GPIO_PinSpeed                = GPIO_OSPEED_FAST;
	handlerI2cLcdSDA.GPIO_PinConfig.GPIO_PinPuPdControl          = GPIO_PUPDR_PULLUP;
	handlerI2cLcdSDA.GPIO_PinConfig.GPIO_PinAltFunMode           = AF4;
	GPIO_Config(&handlerI2cLcdSDA);

	// Configuración del SHT30
	handlerSHT30.ptrI2Cx                          = I2C1;
	handlerSHT30.modeI2C                          = I2C_MODE_FM;
	handlerSHT30.slaveAddress                     = SHT30_ADDRESS;
	handlerSHT30.mainClock						  = MAIN_CLOCK_16_MHz_FOR_I2C;
	handlerSHT30.maxI2C_FM						  = I2C_MAX_RISE_TIME_FM_16MHz;
	handlerSHT30.modeI2C_FM						  = I2C_MODE_FM_SPEED_400KHz_16MHz;

	i2c_config(&handlerSHT30);

	// Configuración de la LCD
	handlerLCD.ptrI2Cx                            = I2C3;
	handlerLCD.modeI2C                            = I2C_MODE_FM;
	handlerLCD.slaveAddress                       = LCD_ADDRESS	;
	handlerLCD.mainClock						  = MAIN_CLOCK_16_MHz_FOR_I2C;
	handlerLCD.maxI2C_FM						  = I2C_MAX_RISE_TIME_FM_16MHz;
	handlerLCD.modeI2C_FM						  = I2C_MODE_FM_SPEED_400KHz_16MHz;

	i2c_config(&handlerLCD);


	/*	-	-	-	Pines Trigger de los HC-SR04	-	-	-	*/

	/*	-	-	-	EJE X	-	-	-	*/

	handlerTrigX1.pGPIOx											= GPIOC;
	handlerTrigX1.GPIO_PinConfig.GPIO_PinNumber						= PIN_5;
	handlerTrigX1.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_OUT;
	handlerTrigX1.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerTrigX1.GPIO_PinConfig.GPIO_PinPuPdControl				= GPIO_PUPDR_NOTHING;
	handlerTrigX1.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerTrigX1.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	handlerTrigX2.pGPIOx											= GPIOC;
	handlerTrigX2.GPIO_PinConfig.GPIO_PinNumber						= PIN_6;
	handlerTrigX2.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_OUT;
	handlerTrigX2.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerTrigX2.GPIO_PinConfig.GPIO_PinPuPdControl				= GPIO_PUPDR_NOTHING;
	handlerTrigX2.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerTrigX2.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	// Se cargan la configuraciones y se inicializan en 0
	GPIO_Config(&handlerTrigX1);
	GPIO_Config(&handlerTrigX2);
	GPIO_WritePin(&handlerTrigX1, RESET);
	GPIO_WritePin(&handlerTrigX2, RESET);

	/*	-	-	-	EJE Y	-	-	-	*/

	handlerTrigY1.pGPIOx											= GPIOA;
	handlerTrigY1.GPIO_PinConfig.GPIO_PinNumber						= PIN_12;
	handlerTrigY1.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_OUT;
	handlerTrigY1.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerTrigY1.GPIO_PinConfig.GPIO_PinPuPdControl				= GPIO_PUPDR_NOTHING;
	handlerTrigY1.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerTrigY1.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	handlerTrigY2.pGPIOx											= GPIOA;
	handlerTrigY2.GPIO_PinConfig.GPIO_PinNumber						= PIN_11;
	handlerTrigY2.GPIO_PinConfig.GPIO_PinMode						= GPIO_MODE_OUT;
	handlerTrigY2.GPIO_PinConfig.GPIO_PinOPType						= GPIO_OTYPE_PUSHPULL;
	handlerTrigY2.GPIO_PinConfig.GPIO_PinPuPdControl				= GPIO_PUPDR_NOTHING;
	handlerTrigY2.GPIO_PinConfig.GPIO_PinSpeed						= GPIO_OSPEED_FAST;
	handlerTrigY2.GPIO_PinConfig.GPIO_PinAltFunMode					= AF0;

	// Se cargan la configuraciones y se inicializan en 0
	GPIO_Config(&handlerTrigY1);
	GPIO_Config(&handlerTrigY2);
	GPIO_WritePin(&handlerTrigY1, RESET);
	GPIO_WritePin(&handlerTrigY2, RESET);

	/*	-	-	-	Pines Echo de los HC-SR04	-	-	-	*/

	/*	-	-	-	EJE X	-	-	-	*/

	handlerEchoFallX1.pGPIOx										= GPIOC;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinNumber					= PIN_1;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerEchoFallX2.pGPIOx										= GPIOC;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinNumber					= PIN_7;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerExtiEchoFallX1.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallX1.pGPIOHandler		= &handlerEchoFallX1;

	handlerExtiEchoFallX2.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallX2.pGPIOHandler		= &handlerEchoFallX2;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoFallX1);
	GPIO_Config(&handlerEchoFallX2);
	GPIO_WritePin(&handlerEchoFallX1, RESET);
	GPIO_WritePin(&handlerEchoFallX2, RESET);

	extInt_Config(&handlerExtiEchoFallX1);
	extInt_Config(&handlerExtiEchoFallX2);

	/*	-	-	-	EJE Y	-	-	-	*/

	handlerEchoFallY1.pGPIOx										= GPIOB;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinNumber					= PIN_14;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerEchoFallY2.pGPIOx										= GPIOB;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinNumber					= PIN_15;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerExtiEchoFallY1.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallY1.pGPIOHandler		= &handlerEchoFallY1;

	handlerExtiEchoFallY2.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallY2.pGPIOHandler		= &handlerEchoFallY2;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoFallY1);
	GPIO_Config(&handlerEchoFallY2);
	GPIO_WritePin(&handlerEchoFallY1, RESET);
	GPIO_WritePin(&handlerEchoFallY2, RESET);

	extInt_Config(&handlerExtiEchoFallY1);
	extInt_Config(&handlerExtiEchoFallY2);

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

		for(int i = 0; i < 3; i++){
			// Pulso ultrasónico X+
			GPIO_WritePin(&handlerTrigX1, SET);
			delay_ms(1);
			GPIO_WritePin(&handlerTrigX1, RESET);
			StartTimer(&handlerStopwatch); // Se empieza a contabilizar el tiempo

			delay_ms(5);

			// Aquí la exti del echo detiene el conteo de tiempo

			timeOfFlightAB = stopwatch / 500000.0;

			distanceX1 = 348.2*timeOfFlightAB;

			stopwatch = 0;

			delay_ms(60);

			// Pulso ultrasónico X-
			GPIO_WritePin(&handlerTrigX2, SET);
			delay_ms(1);
			GPIO_WritePin(&handlerTrigX2, RESET);
			StartTimer(&handlerStopwatch); // Se empieza a contabilizar el tiempo

			delay_ms(5);

			// Aquí la exti del echo detiene el conteo de tiempo

			timeOfFlightBA = stopwatch / 500000.0;

			distanceX2 = 348.2*timeOfFlightBA;

			stopwatch = 0;

			delay_ms(60);

			// Pulso ultrasónico Y+
			GPIO_WritePin(&handlerTrigY1, SET);
			delay_ms(1);
			GPIO_WritePin(&handlerTrigY1, RESET);
			StartTimer(&handlerStopwatch); // Se empieza a contabilizar el tiempo

			delay_ms(5);

			// Aquí la exti del echo detiene el conteo de tiempo

			timeOfFlightCD = stopwatch / 500000.0;

			distanceY1 = 348.2*timeOfFlightCD;

			stopwatch = 0;

			delay_ms(60);

			// Pulso ultrasónico Y-
			GPIO_WritePin(&handlerTrigY2, SET);
			delay_ms(1);
			GPIO_WritePin(&handlerTrigY2, RESET);
			StartTimer(&handlerStopwatch); // Se empieza a contabilizar el tiempo

			delay_ms(5);

			// Aquí la exti del echo detiene el conteo de tiempo

			timeOfFlightDC = stopwatch / 500000.0;

			distanceY2 = 348.2*timeOfFlightDC;

			stopwatch = 0;

			delay_ms(60);


			Vx = (0.47 / 2)*((1 / timeOfFlightAB)-(1 / timeOfFlightBA));

			Vy = (0.475 / 2)*((1 / timeOfFlightCD)-(1 / timeOfFlightDC));

			Vx_mean = Vx / 3;

			Vy_mean = Vy / 3;

			timeOfFlightAB = 0;
			timeOfFlightBA = 0;

			timeOfFlightCD = 0;
			timeOfFlightDC = 0;

		}

		squares = (Vx_mean*Vx_mean) + (Vy_mean*Vy_mean);
		arm_sqrt_f32(squares, &V);

		atan_arg = Vy_mean / Vx_mean;
		Direction = atan(atan_arg)*(180/ M_PI);

		if(Direction < 0){
			Direction += 360;
		}

		sprintf(bufferData,"V = %.3f m/s\n Angle = %.2f °\n", V, Direction);
		writeMsg(&handlerCommTerminal, bufferData);

	}
	else{
		// Se imprime el mensaje "Wrong CMD" si la escritura no corresponde a los CMD implementados
		writeMsg(&handlerCommTerminal, "\n");
		writeMsg(&handlerCommTerminal, "Wrong CMD\n");
		writeMsg(&handlerCommTerminal, "\n");
	}
}

void measureTemp_Hum(void){
	// Se manda el start
	i2c_startTransaction(&handlerSHT30);

	// Se manda el slave address con la instrucción de escribir
	i2c_sendSlaveAddressRW(&handlerSHT30, SHT30_ADDRESS, I2C_WRITE_DATA);

	// Se le pide al sensor que mida humedad y temperatura, con alta repetibilidad y clock stretching activado
	i2c_sendDataByte(&handlerSHT30, 0x2C);
	i2c_sendDataByte(&handlerSHT30, 0x06);

	// Se manda el stop
	i2c_stopTransaction(&handlerSHT30);

	// Se espera a que el sensor complete la medida
	while(!(GPIO_ReadPin(&handlerI2cSCL))){
		__NOP();
	}

	// Se manda el restart
	i2c_reStartTransaction(&handlerSHT30);

	// Se manda el slave address con la instrucción de leer
	i2c_sendSlaveAddressRW(&handlerSHT30, SHT30_ADDRESS, I2C_READ_DATA);

	// Se habilita el send ACK
	i2c_sendAck(&handlerSHT30);

	// Lectura de temperatura
	temp1 = i2c_readDataByte(&handlerSHT30);
	temp2 = i2c_readDataByte(&handlerSHT30);
	crc = i2c_readDataByte(&handlerSHT30);

	// Lectura de humedad
	hum1 = i2c_readDataByte(&handlerSHT30);
	hum2 = i2c_readDataByte(&handlerSHT30);

	// Activamos el send NACK

	i2c_sendNoAck(&handlerSHT30);

	// Leemos ell último registro
	crc = i2c_readDataByte(&handlerSHT30);

	// Mandamos el stop
	i2c_stopTransaction(&handlerSHT30);


	// Se extraen las medidas de temperatura y humedad
	temperature_read = (temp1 << 8) | temp2;
	humidity_read	 = (hum1 << 8) | hum2;

	temperature = (temperature_read / 65535)*(175)-(45);
	humidity = (humidity_read / 65535)*(100);

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
	counterLCD++;
	counterSampling++;
}

void BasicTimer4_Callback(void){
	// Timer encargado de cronometrar
	stopwatch++;
}

void callback_extInt1(void){
	// Exti del falling edge del echo de X1
	StopTimer(&handlerStopwatch);
}

void callback_extInt7(void){
	// Exti del falling edge del echo de X2
	StopTimer(&handlerStopwatch);
}

void callback_extInt14(void){
	// Exti del falling edge del echo de Y1
	StopTimer(&handlerStopwatch);
}

void callback_extInt15(void){
	// Exti del falling edge del echo de Y2
	StopTimer(&handlerStopwatch);
}

/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
