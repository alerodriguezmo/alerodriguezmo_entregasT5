/**
 * **************************************************************************************************
 * @file     : Lazarus_Main.c
 * @author   : Alejandro Rodríguez Montes (alerodriguezmo@unal.edu.co)
 * @brief    : La única derrota es rendirse
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
#include "PLLDriver.h"
#include "DriverLCD.h"

#include "arm_math.h"

/*	-	-	-	Definición de las macros a utilizar	-	-	-	*/

#define SHT30_ADDRESS			 0x45
#define LCD_ADDRESS				 0x24

/*	-	-	-	Definición de handlers	-	-	-	*/

// Handler del led de estado (Blinky)
GPIO_Handler_t handlerLEDBlinky = {0};

// Handlers de los timers
BasicTimer_Handler_t handlerBlinkyTimer		= {0};
BasicTimer_Handler_t handlerSamplingTOF_one	= {0};
BasicTimer_Handler_t handlerSamplingTOF_two	= {0};

// Handlers del protocolo I2C
GPIO_Handler_t handlerI2cSDA 		= {0};
GPIO_Handler_t handlerI2cSCL 		= {0};
GPIO_Handler_t handlerI2cLcdSCL 	= {0};
GPIO_Handler_t handlerI2cLcdSDA 	= {0};
I2C_Handler_t handlerSHT30 			= {0};
I2C_Handler_t handlerLCD 			= {0};

// Handlers de la comunicacion serial
GPIO_Handler_t handlerPinTX 			= {0};
GPIO_Handler_t handlerPinRX 			= {0};

USART_Handler_t handlerCommTerminal 	= {0};

// Handlers de los pines Trigger de los HC-SR04
GPIO_Handler_t handlerTrigX		= {0};

GPIO_Handler_t handlerTrigY		= {0};

// Handlers de los pines Echo de los HC-SR04
GPIO_Handler_t handlerEchoRiseX1		= {0};
GPIO_Handler_t handlerEchoFallX1		= {0};
EXTI_Config_t handlerExtiEchoRiseX1		= {0};
EXTI_Config_t handlerExtiEchoFallX1		= {0};

GPIO_Handler_t handlerEchoRiseX2		= {0};
GPIO_Handler_t handlerEchoFallX2		= {0};
EXTI_Config_t handlerExtiEchoRiseX2		= {0};
EXTI_Config_t handlerExtiEchoFallX2		= {0};


GPIO_Handler_t handlerEchoRiseY1		= {0};
GPIO_Handler_t handlerEchoFallY1		= {0};
EXTI_Config_t handlerExtiEchoRiseY1		= {0};
EXTI_Config_t handlerExtiEchoFallY1		= {0};

GPIO_Handler_t handlerEchoRiseY2		= {0};
GPIO_Handler_t handlerEchoFallY2		= {0};
EXTI_Config_t handlerExtiEchoRiseY2		= {0};
EXTI_Config_t handlerExtiEchoFallY2		= {0};

/*	-	-	-	Definición de variables	-	-	-	*/

// Variables para la implementación del sensor de humedad y temperatura
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

// Variables para la implementación de la pantalla LCD
uint8_t counterLCD		= 0;
uint8_t counterSampling = 0;

uint8_t i2cBuffer 		= {0};

// Variables para la comunicación serial
uint8_t rxData = 0;
char bufferData[64];

// Variables para la implementación de los sensores ultrasónicos HC-SR04
char bufferTOFAB[64];
char bufferTOFBA[64];
char bufferVx[64];

uint64_t stopwatch_one	= 0;
uint64_t stopwatch_two	= 0;
double  timeOfFlightAB	= 0;
double  timeOfFlightBA	= 0;
double  timeOfFlightCD	= 0;
double  timeOfFlightDC	= 0;
float distanceX1		= 0;
float distanceX2		= 0;
float distanceY1		= 0;
float distanceY2		= 0;

float Vx = 0;
float Vx_data[32];
float Vx_mean = 0;

float Vy = 0;
float Vy_data[32];
float Vy_mean = 0;

float32_t squares = 0;

float32_t V = 0;
float direction = 0;



uint8_t flagExtiX1_rise = 0;
uint8_t flagExtiX1_fall = 0;

uint8_t flagExtiX2_rise = 0;
uint8_t flagExtiX2_fall = 0;

uint8_t flagExtiY1_rise = 0;
uint8_t flagExtiY1_fall = 0;

uint8_t flagExtiY2_rise = 0;
uint8_t flagExtiY2_fall = 0;


/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void tuneMCU(void);
void initSystem(void);
void measureTemp_Hum(void);
void resetSHT30(void);
void measureTOF(void);
float calculate_mean(const float* data, uint32_t length);
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

	// Etiquieta uno
	moveCursor_inLCD(&handlerLCD, 0, 0);
	sendSTR_toLCD(&handlerLCD, "USE SERIAL TERMINAL");

	// Etiquieta dos
	moveCursor_inLCD(&handlerLCD, 1, 1);
	sendSTR_toLCD(&handlerLCD, "FOR AIRSPEED INFO");

	// Etiquieta tres
	moveCursor_inLCD(&handlerLCD, 2, 1);
	sendSTR_toLCD(&handlerLCD, "Temp [C] = ");

	// Etiquieta cuatro
	moveCursor_inLCD(&handlerLCD, 3, 1);
	sendSTR_toLCD(&handlerLCD, "RH [%] = ");


	delay_ms(20);

	while(1){

		// Ciclo que muestrea temperatura y humedad
		if(counterSampling > 4){
			measureTemp_Hum();
			counterSampling = 0;
		}
		// Ciclo que permite actualizar las lecturas en pantalla
		if(counterLCD > 4){

			sprintf(bufferTemp,"%.2f",temperature);
			sprintf(bufferHum,"%.2f", humidity);

			moveCursor_inLCD(&handlerLCD, 2, 12);
			sendSTR_toLCD(&handlerLCD, bufferTemp);

			moveCursor_inLCD(&handlerLCD, 3, 11);
			sendSTR_toLCD(&handlerLCD, bufferHum);

			counterLCD = 0;
		}

		// Hacemos un "eco" con el valor que nos llega por el puerto serial

		if(rxData != '\0'){
			// Sección de ayuda donde se muestran los diferentes comandos disponibles (help)
			if(rxData == 'h'){
				writeMsg(&handlerCommTerminal, "HELP MENU: \n"
						"h -> List of available commands \n"
						"x -> Sample for 5 seconds and calculate airspeed in x-axis \n"
						"y -> Sample for 5 seconds and calculate airspeed in y-axis \n"
						"s -> Sample both axes for 5 seconds and calculate airspeed magnitude \n"
						"d -> Sample for 5 seconds and calculate airspeed's general direction \n" );
			    rxData = '\0';
			}
			// Muestreo en el eje X
			else if (rxData == 'x'){
				sprintf(bufferData, "Sampling airspeed in x-axis... \n");
				writeMsg(&handlerCommTerminal, bufferData);
				measureTOF();

				sprintf(bufferData, "Airspeed in x-axis: %.2f m/s \n", calculate_mean(Vx_data, 32));
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Muestreo en el eje Y
			else if(rxData == 'y'){
				sprintf(bufferData, "Sampling airspeed in y-axis... \n");
				writeMsg(&handlerCommTerminal, bufferData);
				measureTOF();

				sprintf(bufferData, "Airspeed in y-axis: %.2f m/s \n", calculate_mean(Vy_data, 32));
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Muestreo de la magnitud
			else if(rxData == 's'){
				sprintf(bufferData, "Sampling airspeed magnitude... \n");
				writeMsg(&handlerCommTerminal, bufferData);
				measureTOF();

				Vx_mean = calculate_mean(Vx_data, 32);
				Vy_mean = calculate_mean(Vy_data, 32);

				squares = (Vx_mean*Vx_mean)+(Vy_mean*Vy_mean);

				arm_sqrt_f32(squares, &V);

				sprintf(bufferData, "Airspeed: %.2f m/s \n", V);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Muestreo de la dirección
			else if(rxData == 'd'){
				sprintf(bufferData, "Sampling airspeed's general direction... \n");
				writeMsg(&handlerCommTerminal, bufferData);
				measureTOF();

				Vx_mean = calculate_mean(Vx_data, 32);
				Vy_mean = calculate_mean(Vy_data, 32);

				direction = atan(Vy_mean / Vx_mean)*(180/M_PI);

				if (direction > -10 && direction < 10){
					sprintf(bufferData, "Wind is coming from the North \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				else if (direction > 10 && direction < 80){
					sprintf(bufferData, "Wind is coming from the Northeast \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				else if (direction > 80 && direction < 100){
					sprintf(bufferData, "Wind is coming from the East \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				else if (direction > 100 && direction < 170){
					sprintf(bufferData, "Wind is coming from the Southeast \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				else if ((direction >= -180 && direction <= -170) || (direction >= 170 && direction <= 180)){
					sprintf(bufferData, "Wind is coming from the South \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				else if (direction > -170 && direction < -100){
					sprintf(bufferData, "Wind is coming from the Southwest \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				else if (direction > -100 && direction < -80){
					sprintf(bufferData, "Wind is coming from the West \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				else if (direction > -80 && direction < -10){
					sprintf(bufferData, "Wind is coming from the Northwest \n");
					writeMsg(&handlerCommTerminal, bufferData);
				}
				rxData = '\0';
			}

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
	handlerSamplingTOF_one.ptrTIMx                               = TIM4;
	handlerSamplingTOF_one.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerSamplingTOF_one.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_100ns;
	handlerSamplingTOF_one.TIMx_Config.TIMx_period               = 2;
	handlerSamplingTOF_one.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerSamplingTOF_one);

	// Configuración del TIM5 para la medición de los tiempos de vuelo del eje y
	handlerSamplingTOF_two.ptrTIMx                               = TIM5;
	handlerSamplingTOF_two.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerSamplingTOF_two.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_100ns;
	handlerSamplingTOF_two.TIMx_Config.TIMx_period               = 2;
	handlerSamplingTOF_two.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerSamplingTOF_two);

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

	/*	-	-	-	Comunicación serial	-	-	-	*/
	handlerPinTX.pGPIOx                               = GPIOC;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber        = PIN_6;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF8;

	// Se carga la configuración
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx                               = GPIOC;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber        = PIN_7;
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

	/*	-	-	-	Pines Trigger de los HC-SR04	-	-	-	*/

	/*	-	-	-	EJE X	-	-	-	*/

	handlerTrigX.pGPIOx											= GPIOC;
	handlerTrigX.GPIO_PinConfig.GPIO_PinNumber					= PIN_11;
	handlerTrigX.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_OUT;
	handlerTrigX.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerTrigX.GPIO_PinConfig.GPIO_PinPuPdControl				= GPIO_PUPDR_PULLDOWN;
	handlerTrigX.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerTrigX.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	// Se carga la configuración y se inicializa el pin en 0
	GPIO_Config(&handlerTrigX);
	GPIO_WritePin(&handlerTrigX, RESET);

	/*	-	-	-	EJE Y	-	-	-	*/

	handlerTrigY.pGPIOx											= GPIOC;
	handlerTrigY.GPIO_PinConfig.GPIO_PinNumber					= PIN_3;
	handlerTrigY.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_OUT;
	handlerTrigY.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerTrigY.GPIO_PinConfig.GPIO_PinPuPdControl				= GPIO_PUPDR_PULLDOWN;
	handlerTrigY.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerTrigY.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	// Se carga la configuración y se inicializa el pin en 0
	GPIO_Config(&handlerTrigY);
	GPIO_WritePin(&handlerTrigY, RESET);


	/*	-	-	-	Pines Echo de los HC-SR04	-	-	-	*/

	/*	-	-	-	EJE X	-	-	-	*/

	// Sensor X1
	handlerEchoRiseX1.pGPIOx										= GPIOC;
	handlerEchoRiseX1.GPIO_PinConfig.GPIO_PinNumber					= PIN_10;
	handlerEchoRiseX1.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoRiseX1.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoRiseX1.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoRiseX1.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoRiseX1.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerEchoFallX1.pGPIOx										= GPIOC;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinNumber					= PIN_12;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoFallX1.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;

	handlerExtiEchoRiseX1.edgeType			= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiEchoRiseX1.pGPIOHandler		= &handlerEchoRiseX1;

	handlerExtiEchoFallX1.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallX1.pGPIOHandler		= &handlerEchoFallX1;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoRiseX1);
	GPIO_Config(&handlerEchoFallX1);
	GPIO_WritePin(&handlerEchoRiseX1, RESET);
	GPIO_WritePin(&handlerEchoFallX1, RESET);

	extInt_Config(&handlerExtiEchoRiseX1);
	extInt_Config(&handlerExtiEchoFallX1);

	// Sensor X2
	handlerEchoRiseX2.pGPIOx										= GPIOC;
	handlerEchoRiseX2.GPIO_PinConfig.GPIO_PinNumber					= PIN_13;
	handlerEchoRiseX2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoRiseX2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoRiseX2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoRiseX2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoRiseX2.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerEchoFallX2.pGPIOx										= GPIOD;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinNumber					= PIN_2;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoFallX2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;

	handlerExtiEchoRiseX2.edgeType			= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiEchoRiseX2.pGPIOHandler		= &handlerEchoRiseX2;

	handlerExtiEchoFallX2.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallX2.pGPIOHandler		= &handlerEchoFallX2;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoRiseX2);
	GPIO_Config(&handlerEchoFallX2);
	GPIO_WritePin(&handlerEchoRiseX2, RESET);
	GPIO_WritePin(&handlerEchoFallX2, RESET);

	extInt_Config(&handlerExtiEchoRiseX2);
	extInt_Config(&handlerExtiEchoFallX2);

	/*	-	-	-	EJE Y	-	-	-	*/

	// Sensor Y1
	handlerEchoRiseY1.pGPIOx										= GPIOC;
	handlerEchoRiseY1.GPIO_PinConfig.GPIO_PinNumber					= PIN_1;
	handlerEchoRiseY1.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoRiseY1.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoRiseY1.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoRiseY1.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoRiseY1.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerEchoFallY1.pGPIOx										= GPIOB;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinNumber					= PIN_0;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoFallY1.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;

	handlerExtiEchoRiseY1.edgeType			= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiEchoRiseY1.pGPIOHandler		= &handlerEchoRiseY1;

	handlerExtiEchoFallY1.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallY1.pGPIOHandler		= &handlerEchoFallY1;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoRiseY1);
	GPIO_Config(&handlerEchoFallY1);
	GPIO_WritePin(&handlerEchoRiseY1, RESET);
	GPIO_WritePin(&handlerEchoFallY1, RESET);

	extInt_Config(&handlerExtiEchoRiseY1);
	extInt_Config(&handlerExtiEchoFallY1);

	// Sensor Y2
	handlerEchoRiseY2.pGPIOx										= GPIOB;
	handlerEchoRiseY2.GPIO_PinConfig.GPIO_PinNumber					= PIN_5;
	handlerEchoRiseY2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoRiseY2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoRiseY2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoRiseY2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;
	handlerEchoRiseY2.GPIO_PinConfig.GPIO_PinAltFunMode				= AF0;

	handlerEchoFallY2.pGPIOx										= GPIOB;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinNumber					= PIN_4;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinOPType					= GPIO_OTYPE_PUSHPULL;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_PULLDOWN;
	handlerEchoFallY2.GPIO_PinConfig.GPIO_PinSpeed					= GPIO_OSPEED_FAST;

	handlerExtiEchoRiseY2.edgeType			= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiEchoRiseY2.pGPIOHandler		= &handlerEchoRiseY2;

	handlerExtiEchoFallY2.edgeType			= EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExtiEchoFallY2.pGPIOHandler		= &handlerEchoFallY2;

	// Se cargan las configuraciones
	GPIO_Config(&handlerEchoRiseY2);
	GPIO_Config(&handlerEchoFallY2);
	GPIO_WritePin(&handlerEchoRiseY2, RESET);
	GPIO_WritePin(&handlerEchoFallY2, RESET);

	extInt_Config(&handlerExtiEchoRiseY2);
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

void measureTOF(void){
	for(int i = 0; i < 32; i++){

		// Se habiltan las exti correspondientes al eje X. Los callbacks las desactivan.
		flagExtiX1_rise = 1;
		flagExtiX1_fall = 1;

		flagExtiX2_rise = 1;
		flagExtiX2_fall = 1;

		// Se activan los pulos ultrasónicos del eje X.
		GPIO_WritePin(&handlerTrigX, SET);
		delay_ms(1);
		GPIO_WritePin(&handlerTrigX, RESET);

		delay_ms(5);

		// Aquí la exti de los echo empiezan y paran los timers correspondientes

		timeOfFlightAB = (200*((float)stopwatch_one+6520)) / (1000000000); // + 0.0001 Correcíon experimental

		timeOfFlightBA = (200*((float)stopwatch_two+6520)) / (1000000000); // Correcíon experimental

		Vx = (0.465/2)*((1/timeOfFlightAB) - (1/timeOfFlightBA));

		Vx_data[i] = Vx;

		stopwatch_one = 0;
		stopwatch_two = 0;

		delay_ms(60); // Delay entre mediciones recomendado por el fabricante

		// Se habiltan las exti correspondientes al eje Y. Los callbacks las desactivan.
		flagExtiY1_rise = 1;
		flagExtiY1_fall = 1;

		flagExtiY2_rise = 1;
		flagExtiY2_fall = 1;

		// Se activan los pulos ultrasónicos del eje Y.
		GPIO_WritePin(&handlerTrigY, SET);
		delay_ms(1);
		GPIO_WritePin(&handlerTrigY, RESET);

		delay_ms(5);

		// Aquí la exti de los echo empiezan y paran los timers correspondientes

		timeOfFlightCD = (200*((float)stopwatch_one)) / (100000000000) + 0.0001; // Correcíon experimental

		timeOfFlightDC = (200*((float)stopwatch_two)) / (100000000000) + 0.0001; // Correcíon experimental

		Vy = (0.465/2)*((1/timeOfFlightDC) - (1/timeOfFlightCD));

		Vy_data[i] = Vx;

		stopwatch_one = 0;
		stopwatch_two = 0;

		delay_ms(60); // Delay entre mediciones recomendado por el fabricante
	}
}
float calculate_mean(const float* data, uint32_t length) {
    if (length == 0) {
        return 0.0f;  // Handle zero-length array
    }

    float sum = 0.0f;
    for (uint32_t i = 0; i < length; i++) {
        sum += data[i];
    }

    return sum / length;
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
	// Callback corresponiente al TOF one
	stopwatch_one++;
}

void BasicTimer5_Callback(void){
	// Callback corresponiente al TOF two
	stopwatch_two++;
}

// CALLBACKS EJE X
void callback_extInt10(void){
	if(flagExtiX1_rise){
		// Exti del rising edge del echo de X1 (PC10). Inicia el timer de TOF one
		StartTimer(&handlerSamplingTOF_one);
		flagExtiX1_rise = 0;
	}
}

void callback_extInt13(void){
	if(flagExtiX2_rise){
		// Exti del rising edge del echo de X2 (PC13). Inicia el timer de TOF two
		StartTimer(&handlerSamplingTOF_two);
		flagExtiX2_rise = 0;
	}
}

void callback_extInt12(void){
	if(flagExtiX1_fall){
		// Exti del falling edge del echo de X1 (PC12). Para el timer de TOF one
		StopTimer(&handlerSamplingTOF_one);
		flagExtiX1_fall = 0;
	}
}

void callback_extInt2(void){
	if(flagExtiX2_fall){
		// Exti del falling edge del echo de X2 (PC14). Para el timer de TOF two
		StopTimer(&handlerSamplingTOF_two);
		flagExtiX2_fall = 0;
	}
}

// CALLBACKS EJE Y
void callback_extInt1(void){
	if(flagExtiY1_rise){
		// Exti del rising edge del echo de Y1 (PC1). Inicia el timer de TOF one
		StartTimer(&handlerSamplingTOF_one);
		flagExtiY1_rise = 0;
	}
}

void callback_extInt5(void){
	if(flagExtiY2_rise){
		// Exti del rising edge del echo de Y2 (PB5). Inicia el timer de TOF one
		StartTimer(&handlerSamplingTOF_two);
		flagExtiY2_rise = 0;
	}
}

void callback_extInt0(void){
	if(flagExtiY1_fall){
		// Exti del falling edge del echo de Y1 (PB0). Para el timer de TOF one
		StopTimer(&handlerSamplingTOF_one);
		flagExtiY1_fall = 0;
	}
}

void callback_extInt4(void){
	if(flagExtiY2_fall){
		// Exti del falling edge del echo de Y2 (PB4). Para el timer de TOF two
		StopTimer(&handlerSamplingTOF_two);
		flagExtiY2_fall = 0;
	}
}

// Callback del usart
void usart6Rx_Callback(void){
	rxData = getRxData();
}
/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
