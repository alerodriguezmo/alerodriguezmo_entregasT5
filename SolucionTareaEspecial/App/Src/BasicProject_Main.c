/**
 * **************************************************************************************************
 * @file     : Maria Jose Uribe Henao
 * @author   : AccelTest_Main.c
 * @brief    : TAREA ESPECIAL - TALLER V
 * **************************************************************************************************
 */


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


#define PLL_80_CLOCK_CONFIGURED  3
#define PLL_80  0

/* Definicion de las macros */
#define ACCEL_ADDRESS          	 0x1D
#define ACCEL_XOUT_L             50     //DATAX0
#define ACCEL_XOUT_H             51     //DATAX1
#define ACCEL_YOUT_L             52     //DATAYO
#define ACCEL_YOUT_H             53     //DATAY1
#define ACCEL_ZOUT_L             54     //DATAZ0
#define ACCEL_ZOUT_H             55     //DATAZ1

#define BW_RATE                  44     //Data Rate

#define POWER_CTL                45
#define WHO_AM_I                 0      //DEVID

#define LCD_ADDRESS				0x24

/* Definicion de variables */

/* Elementos para hacer la comunicacion serial */
GPIO_Handler_t handlerPinTX ={0};
GPIO_Handler_t handlerPinRX ={0};
USART_Handler_t handlerCommTerminal={0};

// Handler del Blinky simple
GPIO_Handler_t handlerBlinkyPin = {0};

/* Handler de los timers */
BasicTimer_Handler_t handlerBlinkyTimer = {0};
BasicTimer_Handler_t handlerMuestreo = {0};

GPIO_Handler_t handlerPinPwmChannelX = {0};
GPIO_Handler_t handlerPinPwmChannelY = {0};
GPIO_Handler_t handlerPinPwmChannelZ = {0};
PWM_Handler_t handlerSignalPwmX = {0};
PWM_Handler_t handlerSignalPwmY = {0};
PWM_Handler_t handlerSignalPwmZ = {0};

uint16_t duttyValueX = 20000;
uint16_t duttyValueY = 20000;
uint16_t duttyValueZ = 20000;

uint8_t AccelX_low =0;
uint8_t AccelX_high=0;
int16_t AccelX=0;
uint8_t AccelY_low=0 ;
uint8_t AccelY_high=0;
int16_t AccelY=0;
uint8_t AccelZ_low=0;
uint8_t AccelZ_high=0;
int16_t AccelZ=0;

//uint8_t usart2DataReceived=0;
uint8_t rxData = 0;
uint8_t contadorLCD=0;

char bufferData[64]= "Accel ADXL-345";

uint8_t systemTicks = 0;
uint8_t systemTicksStart = 0;
uint8_t systemTicksEnd = 0;

/*Configuracion para el I2C */
GPIO_Handler_t handlerI2cSDA = {0};
GPIO_Handler_t handlerI2cSCL = {0};
GPIO_Handler_t handlerI2cLcdSDA = {0};
GPIO_Handler_t handlerI2cLcdSCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
I2C_Handler_t handlerLCD = {0};
uint8_t i2cBuffer ={0};
uint32_t interrupcion=0;
uint8_t bandera=0;

float x,y,z;
float datosAccel[3][2000];
char bufferx[64] = {0};
char buffery[64] = {0};
char bufferz[64] = {0};
char buffersum[64] = {0};

// Definicion de las cabeceras de las funciones
void initSystem(void);
void guardarDato(void);
void PwmSignals(void);

int main (void){
	// Activamos el coprocesador matematico FPU
	SCB->CPACR |= (0xF << 20);

	//Inicio del sistemaS
	initSystem();
	//Frecuencia del micro a 80MHz
	configPLL(PLL_80);
	//Muestreo a 1600Hz
	i2c_writeSingleRegister(&handlerAccelerometer, BW_RATE , 0xF);
	i2c_writeSingleRegister(&handlerAccelerometer, POWER_CTL , 0x2D);
	// Se configura a 80MHz
	config_SysTick_ms(PLL_80_CLOCK_CONFIGURED);
	//Imprimir un mensaje de inicio
	writeMsg(&handlerCommTerminal, "Tarea Especial - Taller V \n"
			"Accel ADXL-345 \n"
			"Frecuencia del microcontrolador: 80 MHz \n"
			"Presionar tecla 'a' para ver los comandos \n");

	clearScreenLCD(&handlerLCD);

	init_LCD(&handlerLCD);
	delay_10(); //EN EL DRIVER DE LA LCD
	clearLCD(&handlerLCD);
	delay_10();
	moveCursor_inLCD(&handlerLCD, 0, 1);
	sendSTR_toLCD(&handlerLCD, "Ax =");
	moveCursor_inLCD(&handlerLCD, 1, 1);
	sendSTR_toLCD(&handlerLCD, "Ay = ");
	moveCursor_inLCD(&handlerLCD, 2, 1);
	sendSTR_toLCD(&handlerLCD, "Az = ");
	moveCursor_inLCD(&handlerLCD, 3, 0);
	sendSTR_toLCD(&handlerLCD, "Sensibilidad = ");


	while(1){
		if(contadorLCD > 4){
			sprintf(bufferx,"%.2f m/s^2",x);
			sprintf(buffery,"%.2f m/s^2",y);
			sprintf(bufferz,"%.2f m/s^2",z);
			sprintf(buffersum,"%.2f", x);

			moveCursor_inLCD(&handlerLCD, 0, 8);
			sendSTR_toLCD(&handlerLCD, bufferx);
			moveCursor_inLCD(&handlerLCD, 1, 8);
			sendSTR_toLCD(&handlerLCD, buffery);
			moveCursor_inLCD(&handlerLCD, 2, 8);
			sendSTR_toLCD(&handlerLCD, bufferz);
			moveCursor_inLCD(&handlerLCD, 3, 10);
			sendSTR_toLCD(&handlerLCD, buffersum);

			contadorLCD = 0;
		}

		guardarDato();

		PwmSignals();

		//Hacemos un "eco" con el valor que nos llega por el serial
		if(rxData != '\0'){
			// DIRECCION DEL ACELEROMETRO
			if(rxData == 'w'){
				sprintf(bufferData, "DEVID \n");
				writeMsg(&handlerCommTerminal, bufferData);
				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
				sprintf(bufferData, "Direccion = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// ESTADO POWER CONTROL
			else if (rxData == 'p'){
				sprintf(bufferData, "PWR_MGMT_1 state \n");
				writeMsg(&handlerCommTerminal, bufferData);
				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, POWER_CTL);
				sprintf(bufferData, "Estado = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// RESET DEL ACELEROMETRO
			else if (rxData == 'r'){
				sprintf(bufferData, "PWR_MGMT_1 reset \n");
				writeMsg(&handlerCommTerminal, bufferData);
				i2c_writeSingleRegister(&handlerAccelerometer, POWER_CTL , 0x2D);
				rxData = '\0';
			}
			// MENSAJES DE INFORMACION
			else if(rxData == 'a'){
				writeMsg(&handlerCommTerminal, "Los mensajes de informacion son los siguientes: \n"
						"a -> Explicacion de cada mensaje \n"
						"w -> Direccion del acelerometro \n"
						"p -> Estado del acelerometro \n"
						"r -> Reset \n"
						"x -> Aceleracion en el eje x \n"
						"y -> Aceleracion en el eje y \n"
						"z -> Aceleracion en el eje z \n"
						"m -> Datos x;y;z del muestreo por 2s \n"
						"d -> Valores del Dutty para PWMx,PWMy,PWMz \n"
						"s -> Se detiene el muestreo constante de 1KHz \n"
						"c -> Se continua el muestreo constante si se detuvo \n" );
			    rxData = '\0';
			}
			// COORDENADAS EN EL EJE X
			else if (rxData == 'x'){
				sprintf(bufferData, "Axis X data \n");
				writeMsg(&handlerCommTerminal, bufferData);
				sprintf(bufferData, "AccelX = %.2f m/s^2 \n", x);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// COORDENADAS EN EL EJE Y
			else if(rxData == 'y'){
				sprintf(bufferData, "Axis Y data \n");
				writeMsg(&handlerCommTerminal, bufferData);
				sprintf(bufferData, "AccelY = %.2f m/s^2 \n", y);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// COORDENADAS EN EL EJE Z
			else if(rxData == 'z'){
				sprintf(bufferData, "Axis Z data \n");
				writeMsg(&handlerCommTerminal, bufferData);
				sprintf(bufferData, "AccelZ = %.2f m/s^2 \n", z);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			// IMPRESION DE LOS 2000 DATOS POR MUESTREO DE 2 SEGUNDOS
			else if(rxData == 'm' ){
				writeMsg(&handlerCommTerminal, "Muestreo por 2 segundos \n" );
				delay_ms(2000);
				for (int i=0;i<2000;i++){
					sprintf(bufferData, "Accel = x %.5f; y %.5f; z %.5f ; #Dato %d \n", datosAccel[0][i],datosAccel[1][i],datosAccel[2][i], i+1);
                    writeMsg(&handlerCommTerminal, bufferData);
				}
				rxData = '\0';
			}
			// valores de Dutty
			else if(rxData == 'd'){
				sprintf(bufferData, "DUTTY = PWMx %d ; PWMy %d ; PWMz %d \n", duttyValueX, duttyValueY, duttyValueZ);
				writeMsg(&handlerCommTerminal, bufferData);
			    rxData = '\0';
			}
			// STOP AL MUESTREO CONSTANTE
			else if(rxData == 's'){
				writeMsg(&handlerCommTerminal, "Se detiene el muestreo constante a 1KHz\n" );
			    bandera = 0;
			    interrupcion = 0;
			    rxData = '\0';
			}
			// RESUME AL MUESTREO CONSTANTE
			else if(rxData == 'c'){
				writeMsg(&handlerCommTerminal, "Se continua el muestreo constante a 1KHz\n" );
			    bandera = 1;
			    rxData = '\0';
			}
			else{
				writeChar(&handlerCommTerminal, rxData);
				rxData = '\0';
			}
		}
	} // Fin del while
	return(0);
} // Fin del main


void initSystem(void){

	// ----------------------------BLINKY SIMPLE ----------------------------------------
	/* Configuracion del LED2 - PA5 - Blinky Simple */
	handlerBlinkyPin.pGPIOx                             = GPIOA;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber      = PIN_5;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEED_FAST;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;

	//Cargar la configuracion del LED2 en los registros
	GPIO_Config(&handlerBlinkyPin);
	GPIO_WritePin(&handlerBlinkyPin, SET);

	// ---------------------------- CONFIGURACION DE LOS TIMERS  ----------------------------------------
	/* Configuracion del TIM2 para que haga un blinky cada 250ms */
	handlerBlinkyTimer.ptrTIMx                               = TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed                = BTIMER_SPEED_80Mhz_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period               = 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable      = 1;

	/* Cargando la configuracion del TIM2 en los registros */
	BasicTimer_Config(&handlerBlinkyTimer);

	/* Configuracion del TIM4 para un muestreo con una Freq de 1KHz  */
	handlerMuestreo.ptrTIMx                               = TIM4;
	handlerMuestreo.TIMx_Config.TIMx_mode                 = BTIMER_MODE_UP;
	handlerMuestreo.TIMx_Config.TIMx_speed                = BTIMER_SPEED_80Mhz_100us;
	handlerMuestreo.TIMx_Config.TIMx_period               = 10;
	handlerMuestreo.TIMx_Config.TIMx_interruptEnable      = 1;

	/* Cargando la configuracion del TIM4 en los registros */
	BasicTimer_Config(&handlerMuestreo);

	// ---------------------------- CONFIGURACION PINES DEL I2C  ----------------------------------------
	/* Pines del acelerometro */
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

	/* Pines de la pantalla LCD */
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

	// ---------------------------- CONFIGURACION DE LA COMUNICACION SERIAL  ----------------------------------------
	handlerPinTX.pGPIOx                               = GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber        = PIN_11;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF8;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx                               = GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber        = PIN_12;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF8;
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

	USART_Config(&handlerCommTerminal);

	// ---------------------------- CONFIGURACION DEL ACELEROMETRO  ----------------------------------------
	handlerAccelerometer.ptrI2Cx                            = I2C1;
	handlerAccelerometer.modeI2C                            = I2C_MODE_FM;
	handlerAccelerometer.slaveAddress                       = ACCEL_ADDRESS;
	handlerAccelerometer.mainClock							= MAIN_CLOCK_80_MHz_FOR_I2C;
	handlerAccelerometer.maxI2C_FM							= I2C_MAX_RISE_TIME_FM_80MHz;
	handlerAccelerometer.modeI2C_FM							= I2C_MODE_FM_SPEED_400KHz_80MHz;

	i2c_config(&handlerAccelerometer);

	// ---------------------------- CONFIGURACION DE LA LCD  ----------------------------------------
	handlerLCD.ptrI2Cx                            = I2C3;
	handlerLCD.modeI2C                            = I2C_MODE_FM;
	handlerLCD.slaveAddress                       = LCD_ADDRESS	;
	handlerLCD.mainClock						  = MAIN_CLOCK_80_MHz_FOR_I2C;
	handlerLCD.maxI2C_FM						  = I2C_MAX_RISE_TIME_FM_80MHz;
	handlerLCD.modeI2C_FM						  = I2C_MODE_FM_SPEED_400KHz_80MHz;

	i2c_config(&handlerLCD);

	// ---------------------------- CONFIGURACION DEL PWM  ----------------------------------------
	handlerPinPwmChannelX.pGPIOx                                = GPIOA;
	handlerPinPwmChannelX.GPIO_PinConfig.GPIO_PinNumber         = PIN_6;
	handlerPinPwmChannelX.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	handlerPinPwmChannelX.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmChannelX.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	handlerPinPwmChannelX.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerPinPwmChannelX.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;
	/* Cargamos la configuracion en los registros del MCU */
	GPIO_Config(&handlerPinPwmChannelX);

	handlerPinPwmChannelY.pGPIOx                                = GPIOB;
	handlerPinPwmChannelY.GPIO_PinConfig.GPIO_PinNumber         = PIN_5;
	handlerPinPwmChannelY.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	handlerPinPwmChannelY.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmChannelY.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	handlerPinPwmChannelY.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerPinPwmChannelY.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;
	/* Cargamos la configuracion en los registros del MCU */
	GPIO_Config(&handlerPinPwmChannelY);

	handlerPinPwmChannelZ.pGPIOx                                = GPIOB;
	handlerPinPwmChannelZ.GPIO_PinConfig.GPIO_PinNumber         = PIN_0;
	handlerPinPwmChannelZ.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	handlerPinPwmChannelZ.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmChannelZ.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	handlerPinPwmChannelZ.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerPinPwmChannelZ.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;
	/* Cargamos la configuracion en los registros del MCU */
	GPIO_Config(&handlerPinPwmChannelZ);

	/* Configuracion del TIM3 para que genere la signal PWM*/
	handlerSignalPwmX.ptrTIMx                = TIM3;
	handlerSignalPwmX.config.channel         = PWM_CHANNEL_1;
	handlerSignalPwmX.config.duttyCicle      = duttyValueX;
	handlerSignalPwmX.config.periodo         = 20000;
	handlerSignalPwmX.config.prescaler       = 80;
	/* Cargamos la configuracion en los registros del MCU */
	pwm_Config(&handlerSignalPwmX);

	/* Configuracion del TIM3 para que genere la signal PWM*/
	handlerSignalPwmY.ptrTIMx                = TIM3;
	handlerSignalPwmY.config.channel         = PWM_CHANNEL_2;
	handlerSignalPwmY.config.duttyCicle      = duttyValueY;
	handlerSignalPwmY.config.periodo         = 20000;
	handlerSignalPwmY.config.prescaler       = 80;
	/* Cargamos la configuracion en los registros del MCU */
	pwm_Config(&handlerSignalPwmY);

	/* Configuracion del TIM3 para que genere la signal PWM*/
	handlerSignalPwmZ.ptrTIMx                = TIM3;
	handlerSignalPwmZ.config.channel         = PWM_CHANNEL_3;
	handlerSignalPwmZ.config.duttyCicle      = duttyValueZ;
	handlerSignalPwmZ.config.periodo         = 20000;
	handlerSignalPwmZ.config.prescaler       = 80;
	/* Cargamos la configuracion en los registros del MCU */
	pwm_Config(&handlerSignalPwmZ);

	enableOutput(&handlerSignalPwmX);
	enableOutput(&handlerSignalPwmY);
	enableOutput(&handlerSignalPwmZ);

	startPwmSignal(&handlerSignalPwmX);
	startPwmSignal(&handlerSignalPwmY);
	startPwmSignal(&handlerSignalPwmZ);

}

/* Funcion para el muestreo inicial por 2 segundos */
void guardarDato(void){
	bandera=1;
	if(interrupcion<2000){
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
	x=(AccelX/256.f)*9.78;
	y=(AccelY/256.f)*9.78;
	z=(AccelZ/256.f)*9.78;
    datosAccel[0][interrupcion] =x;
    datosAccel[1][interrupcion] =y;
    datosAccel[2][interrupcion] =z;
}


void PwmSignals(void){
	duttyValueX = (int)10000+x*1000;
	duttyValueY = (int)10000+y*1000;
	duttyValueZ = (int)10000+z*1000;
	updateDuttyCycle(&handlerSignalPwmX, duttyValueX);
	updateDuttyCycle(&handlerSignalPwmY, duttyValueY);
	updateDuttyCycle(&handlerSignalPwmZ, duttyValueZ);
}


/* Callbacks de las interrupciones */
void usart6Rx_Callback(void){
	rxData = getRxData();
}

/* Callbacks de los Timers */
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin);
	contadorLCD++;
}

void BasicTimer4_Callback(void){
	if (bandera){
		interrupcion++;
		if(interrupcion==2000){
			interrupcion=0;
		}
	}
}
