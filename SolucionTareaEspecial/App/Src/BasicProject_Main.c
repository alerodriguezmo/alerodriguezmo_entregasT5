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

#define PLL_80_CLOCK_CONFIGURED  3
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

// Handlers del PWM
GPIO_Handler_t handlerPinPwmA 		= {0};
GPIO_Handler_t handlerPinPwmB 		= {0};
GPIO_Handler_t handlerPinPwmC 		= {0};
PWM_Handler_t handlerSignalPwmA 	= {0};
PWM_Handler_t handlerSignalPwmB 	= {0};
PWM_Handler_t handlerSignalPwmC 	= {0};

// Handlers de los diferentes I2C
GPIO_Handler_t handlerI2cSDA = {0};
GPIO_Handler_t handlerI2cSCL = {0};
GPIO_Handler_t handlerI2cLcdSDA = {0};
GPIO_Handler_t handlerI2cLcdSCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
I2C_Handler_t handlerLCD = {0};

/*	-	-	-	Definición de variables	-	-	-	*/
uint16_t duttyValueX = 20000;
uint16_t duttyValueY = 20000;
uint16_t duttyValueZ = 20000;

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

uint8_t counterLCD = 0;

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
void PwmSignals(void);

/*	=	=	=	INICIO DEL MAIN	=	=	=	*/
int main (void){
	// Se activa el coprocesador matematico FPU
	SCB->CPACR |= (0xF << 20);

	// Se inicializan todos los sistemas, menos la pantalla LCD

	initSystem();
	// Se establece la frecuencia del micro en 80MHz
	configPLL(PLL_80);

	// Se establece la frecuencia de muestreo a 1600Hz
	i2c_writeSingleRegister(&handlerAccelerometer, BW_RATE , 0xF);
	i2c_writeSingleRegister(&handlerAccelerometer, POWER_CTL , 0x2D);

	// Se configura el SysTick a 80MHz.
	config_SysTick_ms(PLL_80_CLOCK_CONFIGURED);

	// Se imprime un mensaje de inicio por la terminal serial
	writeMsg(&handlerCommTerminal, "TAREA ESPECIAL - Taller V (2023-01) \n"
			"Alejandro Rodriguez Montes \n"
			"Presione la tecla 'h' para ver los comandos disponibles \n");

	// Se inicializa la pantala LCD
	clearScreenLCD(&handlerLCD);

	init_LCD(&handlerLCD);
	delay_10(); //EN EL DRIVER DE LA LCD
	clearLCD(&handlerLCD);
	delay_10();

	// Esctirura de los caracteres permanentes

	// Para el eje X
	moveCursor_inLCD(&handlerLCD, 0, 1);
	sendSTR_toLCD(&handlerLCD, "Ax =");
	moveCursor_inLCD(&handlerLCD, 0, 15);
	sendSTR_toLCD(&handlerLCD, "m/s^2");

	// Para el eje Y
	moveCursor_inLCD(&handlerLCD, 1, 1);
	sendSTR_toLCD(&handlerLCD, "Ay = ");
	moveCursor_inLCD(&handlerLCD, 1, 15);
	sendSTR_toLCD(&handlerLCD, "m/s^2");

	// Para el eje Z
	moveCursor_inLCD(&handlerLCD, 2, 1);
	sendSTR_toLCD(&handlerLCD, "Az = ");
	moveCursor_inLCD(&handlerLCD, 2, 15);
	sendSTR_toLCD(&handlerLCD, "m/s^2");

	// Información sobre el sistema
	moveCursor_inLCD(&handlerLCD, 3, 0);
	sendSTR_toLCD(&handlerLCD, "Sampling at: 1.6KHz");


	while(1){

		// Ciclo que permite actualizar las lecturas en pantalla
		if(counterLCD > 4){
			sprintf(bufferx,"%.2f",x);
			sprintf(buffery,"%.2f",y);
			sprintf(bufferz,"%.2f",z);

			moveCursor_inLCD(&handlerLCD, 0, 8);
			sendSTR_toLCD(&handlerLCD, bufferx);
			moveCursor_inLCD(&handlerLCD, 1, 8);
			sendSTR_toLCD(&handlerLCD, buffery);
			moveCursor_inLCD(&handlerLCD, 2, 8);
			sendSTR_toLCD(&handlerLCD, bufferz);

			counterLCD = 0;
		}

		saveDatum();

		PwmSignals();

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
						"m -> Muestreo de aceleracion por 2 segundos (x;y;z) \n"
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
			// Muestreo de aceleración por 2 segundos. Por la frecuencia de muestreo (1KHz), se entregan 2000 datos
			else if(rxData == 'm' ){
				writeMsg(&handlerCommTerminal, "Muestreo por 2 segundos \n" );
				delay_ms(2000);
				for (int i=0;i<2000;i++){
					sprintf(bufferData, "Accel = x %.5f; y %.5f; z %.5f ; #Dato %d \n", dataAccel[0][i], dataAccel[1][i], dataAccel[2][i], i+1);
                    writeMsg(&handlerCommTerminal, bufferData);
				}
				rxData = '\0';
			}
			// Entrega de los valores del dutty
			else if(rxData == 'd'){
				sprintf(bufferData, "Dutty = PwmA %d ; PwmB %d ; PwmC %d \n", duttyValueX, duttyValueY, duttyValueZ);
				writeMsg(&handlerCommTerminal, bufferData);
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
	handlerBlinkyTimer.TIMx_Config.TIMx_speed                = BTIMER_SPEED_80Mhz_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period               = 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable      = 1;

	// Se carga la configuración
	BasicTimer_Config(&handlerBlinkyTimer);

	// Configuracion del TIM4 para establecer la frecuencia de muestreo en 1KHz
	handlerSampling.ptrTIMx                               = TIM4;
	handlerSampling.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerSampling.TIMx_Config.TIMx_speed                = BTIMER_SPEED_80Mhz_100us;
	handlerSampling.TIMx_Config.TIMx_period               = 10;
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

	// Configuración del acelerómetro
	handlerAccelerometer.ptrI2Cx                            = I2C1;
	handlerAccelerometer.modeI2C                            = I2C_MODE_FM;
	handlerAccelerometer.slaveAddress                       = ACCEL_ADDRESS;
	handlerAccelerometer.mainClock							= MAIN_CLOCK_80_MHz_FOR_I2C;
	handlerAccelerometer.maxI2C_FM							= I2C_MAX_RISE_TIME_FM_80MHz;
	handlerAccelerometer.modeI2C_FM							= I2C_MODE_FM_SPEED_400KHz_80MHz;

	i2c_config(&handlerAccelerometer);

	// Configuración del display LCD
	handlerLCD.ptrI2Cx                            = I2C3;
	handlerLCD.modeI2C                            = I2C_MODE_FM;
	handlerLCD.slaveAddress                       = LCD_ADDRESS	;
	handlerLCD.mainClock						  = MAIN_CLOCK_80_MHz_FOR_I2C;
	handlerLCD.maxI2C_FM						  = I2C_MAX_RISE_TIME_FM_80MHz;
	handlerLCD.modeI2C_FM						  = I2C_MODE_FM_SPEED_400KHz_80MHz;

	i2c_config(&handlerLCD);

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
	handlerCommTerminal.USART_Config.USART_frequency    = 80;

	// Se carga la configuración
	USART_Config(&handlerCommTerminal);


	/*	-	-	-	Pulse Width Modulation (PWM)	-	-	-	*/
	handlerPinPwmA.pGPIOx                                = GPIOA;
	handlerPinPwmA.GPIO_PinConfig.GPIO_PinNumber         = PIN_6;
	handlerPinPwmA.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	handlerPinPwmA.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmA.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	handlerPinPwmA.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerPinPwmA.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;

	// Se carga la configuración
	GPIO_Config(&handlerPinPwmA);

	handlerPinPwmB.pGPIOx                                = GPIOB;
	handlerPinPwmB.GPIO_PinConfig.GPIO_PinNumber         = PIN_5;
	handlerPinPwmB.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	handlerPinPwmB.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmB.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	handlerPinPwmB.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerPinPwmB.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;

	// Se carga la configuración
	GPIO_Config(&handlerPinPwmB);

	handlerPinPwmC.pGPIOx                                = GPIOB;
	handlerPinPwmC.GPIO_PinConfig.GPIO_PinNumber         = PIN_0;
	handlerPinPwmC.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	handlerPinPwmC.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmC.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	handlerPinPwmC.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerPinPwmC.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;

	// Se carga la configuración
	GPIO_Config(&handlerPinPwmC);

	// Configuracion del TIM3 para que genere la signal PWM
	handlerSignalPwmA.ptrTIMx                = TIM3;
	handlerSignalPwmA.config.channel         = PWM_CHANNEL_1;
	handlerSignalPwmA.config.duttyCicle      = duttyValueX;
	handlerSignalPwmA.config.periodo         = 20000;
	handlerSignalPwmA.config.prescaler       = 80;

	// Se carga la configuración
	pwm_Config(&handlerSignalPwmA);

	/* Configuracion del TIM3 para que genere la signal PWM*/
	handlerSignalPwmB.ptrTIMx                = TIM3;
	handlerSignalPwmB.config.channel         = PWM_CHANNEL_2;
	handlerSignalPwmB.config.duttyCicle      = duttyValueY;
	handlerSignalPwmB.config.periodo         = 20000;
	handlerSignalPwmB.config.prescaler       = 80;

	// Se carga la configuración
	pwm_Config(&handlerSignalPwmB);

	// Configuracion del TIM3 para que genere la signal PWM
	handlerSignalPwmC.ptrTIMx                = TIM3;
	handlerSignalPwmC.config.channel         = PWM_CHANNEL_3;
	handlerSignalPwmC.config.duttyCicle      = duttyValueZ;
	handlerSignalPwmC.config.periodo         = 20000;
	handlerSignalPwmC.config.prescaler       = 80;

	// Se carga la configuración
	pwm_Config(&handlerSignalPwmC);

	enableOutput(&handlerSignalPwmA);
	enableOutput(&handlerSignalPwmB);
	enableOutput(&handlerSignalPwmC);

	startPwmSignal(&handlerSignalPwmA);
	startPwmSignal(&handlerSignalPwmB);
	startPwmSignal(&handlerSignalPwmC);

}

/* Funcion para el muestreo inicial por 2 segundos */
void saveDatum(void){
	flag = 1;
	if(interrupt < 2000){
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


void PwmSignals(void){
	duttyValueX = (int)10000+x*1000;
	duttyValueY = (int)10000+y*1000;
	duttyValueZ = (int)10000+z*1000;
	updateDuttyCycle(&handlerSignalPwmA, duttyValueX);
	updateDuttyCycle(&handlerSignalPwmB, duttyValueY);
	updateDuttyCycle(&handlerSignalPwmC, duttyValueZ);
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
	counterLCD++;
}

void BasicTimer4_Callback(void){
	if (flag){
		interrupt++;
		if(interrupt == 2000){
			interrupt = 0;
		}
	}
}
/*	=	=	=	FIN DE LAS RUTINAS DE ATENCIÓN (Callbacks)	=	=	=	*/
