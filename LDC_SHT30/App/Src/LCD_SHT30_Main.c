/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : AccelTest_Main.c
 * @brief    : TAREA ESPECIAL - TALLER V (2023-01)
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

#define SHT30_ADDRESS			 0x45
#define LCD_ADDRESS				 0x24

/*	-	-	-	Definición de handlers	-	-	-	*/

// Handler del led de estado (Blinky)
GPIO_Handler_t handlerLEDBlinky = {0};

// Handlers de los timers
BasicTimer_Handler_t handlerBlinkyTimer		= {0};
BasicTimer_Handler_t handlerSamplingTOFX	= {0};
BasicTimer_Handler_t handlerSamplingTOFY	= {0};

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
EXTI_Config_t handlerExtiEchoFallX1		= {0};
EXTI_Config_t handlerExtiEchoFallX2		= {0};

GPIO_Handler_t handlerEchoFallY1		= {0};
GPIO_Handler_t handlerEchoFallY2		= {0};
EXTI_Config_t handlerExtiEchoFallY1		= {0};
EXTI_Config_t handlerExtiEchoFallY2		= {0};


/*	-	-	-	Definición de variables	-	-	-	*/
char state 				= 0;
char hum1 				= 0;
char hum2 				= 0;
char humAndTemp 		= 0;
char temp1 				= 0;
char temp2 				= 0;
char crc 				= 0;

char bufferTemp[64];
char bufferHum[64];
float temperature_read 	= 0;
float humidity_read 	= 0;
float temperature 		= 0;
float humidity	 		= 0;

uint8_t counterLCD		= 0;
uint8_t counterSampling = 0;

uint8_t i2cBuffer 		= {0};

char bufferTOF[64];
char bufferDist[64];
char bufferVx[64];
uint64_t stopwatch		= 0;
float  timeOfFlightAB	= 0;
float  timeOfFlightBA	= 0;
float  timeOfFlightCD	= 0;
float  timeOfFlightDC	= 0;
float distanceX1		= 0;
float distanceX2		= 0;
float distanceY1		= 0;
float distanceY2		= 0;

float Vx = 0;
float Vx_mean = 0;
float Vx_report = 0;

float Vy = 0;
float Vy_mean = 0;
float Vy_report = 0;

float V = 0;
float V_mean = 0;
float V_report = 0;

uint8_t flagExtiX1 = 0;
uint8_t flagExtiX2 = 0;
uint8_t flagExtiY1 = 0;
uint8_t flagExtiY2 = 0;


/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void tuneMCU(void);
void initSystem(void);
void parseCommands(char *ptrBufferReception);
void measureTemp_Hum(void);
void resetSHT30(void);
void measureTOF_X1(void);
/*	=	=	=	INICIO DEL MAIN	=	=	=	*/
int main (void){

	// Se afina el MCU
	tuneMCU();

	// Se pone el MCU a 100MHz
	configPLL(1);

	// Se inicializan todos los sistemas, menos la pantalla LCD
	initSystem();

	// Se reseta el SHT30
	resetSHT30();

	// Se inicializa la pantala LCD
	clearScreenLCD(&handlerLCD);

	init_LCD(&handlerLCD);
	delay_10(); // DELAYS CON EL SYSTICK IMPLEMENTADOS EN EL DRIVER DE LA LCD
	clearLCD(&handlerLCD);
	delay_10();

	// Esctirura de los caracteres permanentes

	// Etiquieta de temperatura
	moveCursor_inLCD(&handlerLCD, 0, 1);
	sendSTR_toLCD(&handlerLCD, "Temp [C] = ");

	// Etiquieta de humedad relativa
	moveCursor_inLCD(&handlerLCD, 1, 1);
	sendSTR_toLCD(&handlerLCD, "Hum [%] = ");

	// Etiquieta de tempo de vuelo
	moveCursor_inLCD(&handlerLCD, 2, 1);
	sendSTR_toLCD(&handlerLCD, "Vx [m/s] = ");

	delay_ms(20);

	while(1){

		// Ciclo que muestrea temperatura y humedad
		if(counterSampling > 2){
			measureTOF_X1();
			measureTemp_Hum();
			counterSampling = 0;
		}
		// Ciclo que permite actualizar las lecturas en pantalla
		if(counterLCD > 4){

			sprintf(bufferTemp,"%.2f",temperature);
			sprintf(bufferHum,"%.2f", humidity);

			sprintf(bufferVx, "%.2f", Vx_report);

			moveCursor_inLCD(&handlerLCD, 0, 12);
			sendSTR_toLCD(&handlerLCD, bufferTemp);

			moveCursor_inLCD(&handlerLCD, 1, 11);
			sendSTR_toLCD(&handlerLCD, bufferHum);

			moveCursor_inLCD(&handlerLCD, 2, 12);
			sendSTR_toLCD(&handlerLCD, bufferVx);

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

	// Se establece el escalado del APB1 para que reciba los 25MHz para no exceder su frecuencia máxima
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	// Se enciende la señal de reloj del APB1
	RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN);
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

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

	// Configuracion del TIM para que haga un blinky cada 250ms
	handlerBlinkyTimer.ptrTIMx                               = TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period               = 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración y se inicializa el timer
	BasicTimer_Config(&handlerBlinkyTimer);
	StartTimer(&handlerBlinkyTimer);

	// Configuración del TIM4 para la medición de los tiempos de vuelo del eje x
	handlerSamplingTOFX.ptrTIMx                               = TIM4;
	handlerSamplingTOFX.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerSamplingTOFX.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_100ns;
	handlerSamplingTOFX.TIMx_Config.TIMx_period               = 2;
	handlerSamplingTOFX.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerSamplingTOFX);

	// Configuración del TIM5 para la medición de los tiempos de vuelo del eje y
	handlerSamplingTOFY.ptrTIMx                               = TIM5;
	handlerSamplingTOFY.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerSamplingTOFY.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_100ns;
	handlerSamplingTOFY.TIMx_Config.TIMx_period               = 2;
	handlerSamplingTOFY.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerSamplingTOFY);

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
	handlerSHT30.mainClock						  = MAIN_CLOCK_100_MHz_FOR_I2C;
	handlerSHT30.maxI2C_FM						  = I2C_MAX_RISE_TIME_FM_100MHz;
	handlerSHT30.modeI2C_FM						  = I2C_MODE_FM_SPEED_400KHz_100MHz;

	i2c_config(&handlerSHT30);

	// Configuración de la LCD
	handlerLCD.ptrI2Cx                            = I2C3;
	handlerLCD.modeI2C                            = I2C_MODE_FM;
	handlerLCD.slaveAddress                       = LCD_ADDRESS	;
	handlerLCD.mainClock						  = MAIN_CLOCK_100_MHz_FOR_I2C;
	handlerLCD.maxI2C_FM						  = I2C_MAX_RISE_TIME_FM_100MHz;
	handlerLCD.modeI2C_FM						  = I2C_MODE_FM_SPEED_400KHz_100MHz;

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

	handlerEchoFallX2.pGPIOx										= GPIOB;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinNumber					= PIN_14;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;

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

	handlerEchoFallY1.pGPIOx										= GPIOC;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinNumber					= PIN_2;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerEchoFallY2.pGPIOx										= GPIOB;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinNumber					= PIN_13;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;

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

void resetSHT30(void){
	// Se manda el start
	i2c_startTransaction(&handlerSHT30);

	// Se manda el slave address con la instrucción de escribir
	i2c_sendSlaveAddressRW(&handlerSHT30, SHT30_ADDRESS, I2C_WRITE_DATA);

	// Se le pide al sensor que se resetee
	i2c_sendDataByte(&handlerSHT30, 0x30);
	i2c_sendDataByte(&handlerSHT30, 0xA2);

	// Se manda el stop
	i2c_stopTransaction(&handlerSHT30);

}

void measureTOF_X1(void){
	for(int i = 0; i<4; i++){
		// Pulso ultrasónico X+
		GPIO_WritePin(&handlerTrigX1, SET);
		delay_ms(1);
		// Habilitamos la recepción de la Exti
		flagExtiX1 = 1;
		GPIO_WritePin(&handlerTrigX1, RESET);
		StartTimer(&handlerSamplingTOFX); // INICIO DEL TIMER

		delay_ms(5);

		// Aquí la exti del echo de X1 para el timer correspondiente

		timeOfFlightAB = (200*((float)(stopwatch)) / 100000000) - 0.00040525;  // Factor de corrección experimental

		distanceX1 = (331+0.6*temperature)*timeOfFlightAB;

		stopwatch = 0;
		delay_ms(60);

		// Pulso ultrasónico X-
		GPIO_WritePin(&handlerTrigX2, SET);
		delay_ms(1);
		// Habilitamos la recepción de la Exti
		flagExtiX2 = 1;
		GPIO_WritePin(&handlerTrigX2, RESET);
		StartTimer(&handlerSamplingTOFX);

		delay_ms(5);

		// Aquí la exti del echo de X2 para el timer

		timeOfFlightBA = (200*(float)(stopwatch+430)/ 100000000) - 0.00040525; // Factor de corrección experimental

		distanceX2 = (331+0.6*temperature)*timeOfFlightBA;
		stopwatch = 0;
		delay_ms(60);

		// Pulso ultrasónico Y+
		GPIO_WritePin(&handlerTrigY1, SET);
		delay_ms(1);
		// Habilitamos la recepción de la Exti
		flagExtiY1 = 1;
		GPIO_WritePin(&handlerTrigY1, RESET);
		StartTimer(&handlerSamplingTOFY);

		delay_ms(5);

		// Aquí la exti del echo de Y1 para el timer

		timeOfFlightCD = (200*(float)(stopwatch+432)/ 100000000) - 0.00040525; // Factor de corrección experimental

		distanceY1 = (331+0.6*temperature)*timeOfFlightCD;
		stopwatch = 0;
		delay_ms(60);

		Vx = (0.475 / 2)*((1 / timeOfFlightBA)-(1 / timeOfFlightAB));
		Vx_mean += Vx/4;

		delay_ms(60);

		// Pulso ultrasónico Y-
		GPIO_WritePin(&handlerTrigY2, SET);
		delay_ms(1);
		// Habilitamos la recepción de la Exti
		flagExtiY2 = 1;
		GPIO_WritePin(&handlerTrigY2, RESET);
		StartTimer(&handlerSamplingTOFY);

		delay_ms(5);

		// Aquí la exti del echo de Y2 para el timer

		timeOfFlightDC = (200*(float)(stopwatch+432)/ 100000000) - 0.00040525; // Factor de corrección experimental

		distanceY2 = (331+0.6*temperature)*timeOfFlightDC;
		stopwatch = 0;
		delay_ms(60);


		Vx = (0.475 / 2)*((1 / timeOfFlightBA)-(1 / timeOfFlightAB));
		Vx_mean += Vx/4;

		delay_ms(60);
	}
	Vx_report = Vx_mean;
	Vx_mean = 0;
}

/*	=	=	=	FIN DE LA DEFINICIÓN DE FUNCIONES	=	=	=	*/

/*	=	=	=	INICIO DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/

/* Callbacks de las interrupciones */

/* Callbacks de los Timers */
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerLEDBlinky);
	counterLCD++;
	counterSampling++;
}

void BasicTimer4_Callback(void){
	stopwatch++;
}

void BasicTimer5_Callback(void){
	stopwatch++;
}

void callback_extInt1(void){
	if(flagExtiX1){
		// Exti del falling edge del echo de X1
		StopTimer(&handlerSamplingTOFX);
		flagExtiX1 = 0;
	}
}

void callback_extInt2(void){
	if(flagExtiY1){
		// Exti del falling edge del echo de Y1
		StopTimer(&handlerSamplingTOFY);
		flagExtiY1 = 0;
	}
}

void callback_extInt14(void){
	if(flagExtiX2){
		// Exti del falling edge del echo de X2
		StopTimer(&handlerSamplingTOFX);
		flagExtiX2 = 0;
	}
}

void callback_extInt13(void){
	if(flagExtiY2){
		// Exti del falling edge del echo de Y2
		StopTimer(&handlerSamplingTOFY);
		flagExtiY2 = 0;
	}
}
/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
