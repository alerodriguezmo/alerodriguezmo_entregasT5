/*
 *************************************************************************
 * @file		: ejemploDisplayLCD_Main.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Código ejemplo del uso del display LCD
 *
 *************************************************************************
 */

#include <DriverLCD.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stm32f4xx.h>
//#include <math.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "PwmDriver.h"
#include "SysTickDriver.h"
#include "I2CDriver.h"

/* = = = = = INICIO DE DEFINICIÓN DE LOS ELEMENTOS DEL SISTEMA = = = = =  */

/* = = = Handlers GPIO = = = */
GPIO_Handler_t handlerLED2 			= {0};		 // LED blinky

GPIO_Handler_t handlerI2CSDA = {0};				// Línea SDA del I2C
GPIO_Handler_t handlerI2CSCL = {0};				// Línea SCL del I2C


/* = = = Handlers Timers = = = */
BasicTimer_Handler_t handlerBlinkyTimer 	= {0};		// Timer del LED blinky

/* = = = Handlers I2C = = = */
I2C_Handler_t handlerLCD		= {0};

/* = = = Variables = = = */
char bufferx[64] = {0};
char buffery[64] = {0};
char bufferz[64] = {0};
char buffersum[64] = {0};
uint8_t countA = 0;
uint8_t x = 0;
uint8_t y = 0;
uint8_t z = 0;

//uint8_t usart2DataReceived = 0;
//uint8_t printMsg = 0;

/* = = = Macros = = = */
#define LCD_ADDRESS		0x24 // !OJO QUE EL ADDRESS DE CADA PANTALLA ES DIFERENTE¡ Verificar los jumpers Ai

/* = = = Cabeceras de las funciones = = = */
void init_System(void); 					// Función para inicializar el sistema

/* = = = = = FIN DE DEFINICIÓN DE LOS ELEMENTOS DEL SISTEMA = = = = =  */

/* = = = = = INICIO DE LA FUNCIÓN PRINCIPAL DEL PROGRAMA = = = = =  */
int main(void){

	// Activación del coprocesador matemático - FPU
//	SCB->CPACR |= (0XF << 20);

	// Se configura el systick a 16MHz
//	config_SysTick_ms(0);

	// Inicialización de todos los elementos del sistema
	init_System();

	// LCD
	clearScreenLCD(&handlerLCD);

	init_LCD(&handlerLCD);
	delay_10();
	clearLCD(&handlerLCD);
	delay_10();
	moveCursor_inLCD(&handlerLCD, 0, 2);
	sendSTR_toLCD(&handlerLCD, "Ax =");
	moveCursor_inLCD(&handlerLCD, 1, 2);
	sendSTR_toLCD(&handlerLCD, "Ay = ");
	moveCursor_inLCD(&handlerLCD, 2, 2);
	sendSTR_toLCD(&handlerLCD, "Az = ");
	moveCursor_inLCD(&handlerLCD, 3, 2);
	sendSTR_toLCD(&handlerLCD, "Suma = ");

	while(1){
		if(countA > 4){
			x++;
			y++;
			z++;
			sprintf(bufferx,"%d",x);
			sprintf(buffery,"%d",y);
			sprintf(bufferz,"%d",z);
			sprintf(buffersum,"%d",x+y+z);

			moveCursor_inLCD(&handlerLCD, 0, 8);
			sendSTR_toLCD(&handlerLCD, bufferx);
			moveCursor_inLCD(&handlerLCD, 1, 8);
			sendSTR_toLCD(&handlerLCD, buffery);
			moveCursor_inLCD(&handlerLCD, 2, 8);
			sendSTR_toLCD(&handlerLCD, bufferz);
			moveCursor_inLCD(&handlerLCD, 3, 10);
			sendSTR_toLCD(&handlerLCD, buffersum);

			countA = 0;
		}
		}


}
/* = = = = = FIN DEL CORE DEL PROGRAMA = = = = =  */

/* = = = = = INICIO DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */

// Función de inicialización de hardware
void init_System(void){


	/* = = = INICIO DE LA CONFIGURACIÓN DEL LED DE ESTADO (BLINKY) = = = */
	// Configuración del LED2 - PA5
	handlerLED2.pGPIOx								= GPIOA;
	handlerLED2.GPIO_PinConfig.GPIO_PinNumber		= PIN_5;
	handlerLED2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerLED2.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerLED2.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerLED2.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	// Cargando la configuración
	GPIO_Config(&handlerLED2);

	GPIO_WritePin(&handlerLED2, SET); // Se establece que el LED esté encendido por defecto

	// Configuración del TIM2 para que haga un blinky cada 250ms
	handlerBlinkyTimer.ptrTIMx								= TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 250; // Lanza una interrupción cada 250 ms
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable		= 1;

	// Cargando la configuración del TIM2
	BasicTimer_Config(&handlerBlinkyTimer);

	/* = = = FIN DE LA CONFIGURACIÓN DEL LED DE ESTADO (BLINKY) = = = */

	/* = = = INICIO DE LA CONFIGURACIÓN DEL DISPLAY LCD = = = */
	// Configurando los pines sobre los que funciona el I2C1
	handlerI2CSCL.pGPIOx								= GPIOB;
	handlerI2CSCL.GPIO_PinConfig.GPIO_PinNumber			= PIN_8;
	handlerI2CSCL.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2CSCL.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_OPENDRAIN;
	handlerI2CSCL.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerI2CSCL.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_PULLUP;
	handlerI2CSCL.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;

	handlerI2CSDA.pGPIOx								= GPIOB;
	handlerI2CSDA.GPIO_PinConfig.GPIO_PinNumber			= PIN_9;
	handlerI2CSDA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2CSDA.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_OPENDRAIN;
	handlerI2CSDA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerI2CSDA.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_PULLUP;
	handlerI2CSDA.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;

	// Cargamos las configuraciones
	GPIO_Config(&handlerI2CSCL);
	GPIO_Config(&handlerI2CSDA);

	// Configuramos el protocolo I2C y cargamos dicha configuración
	handlerLCD.ptrI2Cx			= I2C1;
	handlerLCD.modeI2C			= I2C_MODE_SM;
	handlerLCD.slaveAddress		= LCD_ADDRESS;

	i2c_config(&handlerLCD);

	/* = = = FIN DE LA CONFIGURACIÓN DEL DISPLAY LCD  = = = */

}
	/* = = = = = INICIO DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */

	/* = = = = = FIN DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */

	/* = = = = = INICIO DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */

// Callback del timer 2 correspondiente al LED Blinky
void BasicTimer2_Callback(void){
	// Callback del blinky
	GPIOxTooglePin(&handlerLED2);
	countA++;
}

/* = = = = = FIN DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */
