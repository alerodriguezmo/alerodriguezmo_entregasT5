/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : FFTAccel_Main.c
 * @brief    : Archivo de prueba para la implementación de la FFT para los datos del acelerómetro
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

#define PLL_100_CLOCK_CONFIGURED  2
#define PLL_80  0

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

#define LCD_ADDRESS				 0x24

/*	-	-	-	Definición de handlers	-	-	-	*/

// Handlers de la comunicacion serial
GPIO_Handler_t handlerPinTX 			= {0};
GPIO_Handler_t handlerPinRX 			= {0};
USART_Handler_t handlerCommTerminal 	= {0};

// Handler del led de estado (Blinky)
GPIO_Handler_t handlerLEDBlinky = {0};

// Handlers de los timers
BasicTimer_Handler_t handlerBlinkyTimer = {0};
BasicTimer_Handler_t handlerSampling	= {0};

// Handlers del protocolo I2C
GPIO_Handler_t handlerI2cSDA = {0};
GPIO_Handler_t handlerI2cSCL = {0};
I2C_Handler_t handlerAccelerometer = {0};

/*	-	-	-	Definición de variables	-	-	-	*/
uint8_t AccelX_low 		= 0;
uint8_t AccelX_high 	= 0;
int16_t AccelX 			= 0;
uint8_t AccelY_low 		= 0;
uint8_t AccelY_high 	= 0;
int16_t AccelY 			= 0;
uint8_t AccelZ_low		= 0;
uint8_t AccelZ_high 	= 0;
int16_t AccelZ 			= 0;

uint8_t rxData = 0;

char bufferData[64]= "Accel ADXL-345";

uint8_t systemTicks = 0;
uint8_t systemTicksStart = 0;
uint8_t systemTicksEnd = 0;


uint8_t i2cBuffer 		= {0};
uint32_t interrupt	 	= 0;
uint8_t flag			= 0;

float x,y,z;
float dataAccel[3][2000];
char bufferx[64] = {0};
char buffery[64] = {0};
char bufferz[64] = {0};

/*	-	-	-	Definición de las cabeceras de las funciones	-	-	-	*/
void initSystem(void);
void saveDatum(void);

/*	=	=	=	INICIO DEL MAIN	=	=	=	*/
int main (void){
	// Se activa el coprocesador matematico FPU
	SCB->CPACR |= (0xF << 20);

	// Se afina la señal de reloj
	RCC->CR &= ~(RCC_CR_HSITRIM);
	RCC->CR |= 13 << RCC_CR_HSITRIM_Pos;

	// Se establece la frecuencia del micro en 100MHz
	configPLL(FREQUENCY_100_MHz);

	// Se escala la frecuencia de reloj dividiendo por 4 para que el APB1
	// y el protocolo I2C funcionen correctamente
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 << RCC_CFGR_PPRE1_Pos;

	// Se inicializan todos los sistemas

	initSystem();

	// Se establece la frecuencia de muestreo a 1600Hz
	i2c_writeSingleRegister(&handlerAccelerometer, BW_RATE , 0xF);
	i2c_writeSingleRegister(&handlerAccelerometer, POWER_CTL , 0x2D);

	// Se configura el SysTick a 10MHz.
	config_SysTick_ms(PLL_100_CLOCK_CONFIGURED);

	// Se imprime un mensaje de inicio por la terminal serial
	writeMsg(&handlerCommTerminal, "FFT para Accel - Taller V (2023-01) \n"
			"Alejandro Rodriguez Montes \n"
			"Presione la tecla 'h' para ver los comandos disponibles \n");

	while(1){

		saveDatum();


		// Hacemos un "eco" con el valor que nos llega por el puerto serial

		if(rxData != '\0'){
			// Dirección del acelerómetro (slave address)
			if(rxData == 'w'){
				sprintf(bufferData, "DEVID \n");
				writeMsg(&handlerCommTerminal, bufferData);
				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
				sprintf(bufferData, "Direccion = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Estado del power control
			else if (rxData == 'p'){
				sprintf(bufferData, "PWR_MGMT_1 state \n");
				writeMsg(&handlerCommTerminal, bufferData);
				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, POWER_CTL);
				sprintf(bufferData, "Estado = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Reseteado del acelerómetro (reset)
			else if (rxData == 'r'){
				sprintf(bufferData, "PWR_MGMT_1 reset \n");
				writeMsg(&handlerCommTerminal, bufferData);
				i2c_writeSingleRegister(&handlerAccelerometer, POWER_CTL , 0x2D);
				rxData = '\0';
			}
			// Sección de ayuda donde se muestran los diferentes comandos disponibles (help)
			else if(rxData == 'h'){
				writeMsg(&handlerCommTerminal, "Los comandos disponibles son los siguientes: \n"
						"h -> Lista de comandos disponibles \n"
						"w -> Direccion del acelerometro (slave address) \n"
						"p -> Estado del acelerometro \n"
						"r -> Resetear el acelerometro (reset) \n"
						"x -> Aceleracion en el eje x \n"
						"y -> Aceleracion en el eje y \n"
						"z -> Aceleracion en el eje z \n"
						"m -> Muestreo de aceleracion por 1 segundo (x;y;z) \n"
						"d -> Valores del Dutty para PwmA,PwmB y PwmC \n"
						"s -> Detener el muestreo constante a 1KHz \n"
						"c -> Reanudar el muestreo constante, si estaba inactivo \n" );
			    rxData = '\0';
			}
			// Muestreo de aceleración en el eje X
			else if (rxData == 'x'){
				sprintf(bufferData, "Axis X data \n");
				writeMsg(&handlerCommTerminal, bufferData);
				sprintf(bufferData, "AccelX = %.2f m/s^2 \n", x);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Muestreo de aceleración en el eje Y
			else if(rxData == 'y'){
				sprintf(bufferData, "Axis Y data \n");
				writeMsg(&handlerCommTerminal, bufferData);
				sprintf(bufferData, "AccelY = %.2f m/s^2 \n", y);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Muestreo de aceleración en el eje Z
			else if(rxData == 'z'){
				sprintf(bufferData, "Axis Z data \n");
				writeMsg(&handlerCommTerminal, bufferData);
				sprintf(bufferData, "AccelZ = %.2f m/s^2 \n", z);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// Muestreo de aceleración por 2 segundos. Por la frecuencia de muestreo (200Hz), se entregan 400 datos
			else if(rxData == 'm' ){
				writeMsg(&handlerCommTerminal, "Muestreo por 2 segundos \n" );
				delay_ms(2000);
				for (int i=0;i<2000;i++){
					sprintf(bufferData, "Accel = x %.5f; y %.5f; z %.5f ; #Dato %d \n", dataAccel[0][i], dataAccel[1][i], dataAccel[2][i], i+1);
                    writeMsg(&handlerCommTerminal, bufferData);
				}
				rxData = '\0';
			}
			// Detención del muestreo constante a 1Khz (stop)
			else if(rxData == 's'){
				writeMsg(&handlerCommTerminal, "Se detiene el muestreo constante a 1KHz \n" );
			    flag = 0;
			    interrupt = 0;
			    rxData = '\0';
			}
			// Reanudación del muestreo constante
			else if(rxData == 'c'){
				writeMsg(&handlerCommTerminal, "Se reanuda el muestreo constante a 1KHz \n" );
			    flag = 1;
			    rxData = '\0';
			}
			else{
				writeChar(&handlerCommTerminal, rxData);
				rxData = '\0';
			}
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

	// Configuracion del TIM4 para establecer la frecuencia de muestreo en 200Hz
	handlerSampling.ptrTIMx                               = TIM4;
	handlerSampling.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerSampling.TIMx_Config.TIMx_speed                = BTIMER_SPEED_100Mhz_100us;
	handlerSampling.TIMx_Config.TIMx_period               = 50;
	handlerSampling.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerSampling);

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

/* Funcion para el muestreo inicial por 2 segundos */
void saveDatum(void){
	flag = 1;
	if(interrupt < 200){
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
    dataAccel[0][interrupt] =x;
    dataAccel[1][interrupt] =y;
    dataAccel[2][interrupt] =z;
}

/*	=	=	=	FIN DE LA DEFINICIÓN DE FUNCIONES	=	=	=	*/

/*	=	=	=	INICIO DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/

/* Callbacks de las interrupciones */
void usart6Rx_Callback(void){
	rxData = getRxData();
}

/* Callbacks de los Timers */
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerLEDBlinky);
}

void BasicTimer4_Callback(void){
	if (flag){
		interrupt++;
		if(interrupt == 200){
			interrupt = 0;
		}
	}
}
/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
