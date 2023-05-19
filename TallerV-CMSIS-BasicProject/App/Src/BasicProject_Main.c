/*
 *************************************************************************
 * @file		: BasicProject_Main.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Solución básica de un proyecto con librerías externas
 *
 *************************************************************************
 */

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
GPIO_Handler_t handlerUserButton	= {0};		 // Botón del usuario
GPIO_Handler_t handlerPinTX			= {0};
GPIO_Handler_t handlerPinRX			= {0};

/* = = = Handlers Timers = = = */
BasicTimer_Handler_t handlerBlinkyTimer 	= {0};		// Timer del LED blinky

/* = = = Handlers EXTI = = = */
EXTI_Config_t handlerExtiUserButton		= {0};			// EXTI correspondiente al botón del usuario

/* = = = Handlers USART = = = */
USART_Handler_t	usart2Comm			= {0};

/* = = = Variables = = = */
uint8_t sendMsg = 0;
uint8_t usart2DataReceived = 0;
uint8_t printMsg = 0;

/* = = = Cabeceras de las funciones = = = */
void init_System(void); 					// Función para inicializar el sistema

/* = = = = = FIN DE DEFINICIÓN DE LOS ELEMENTOS DEL SISTEMA = = = = =  */

/* = = = = = INICIO DE LA FUNCIÓN PRINCIPAL DEL PROGRAMA = = = = =  */
int main(void){

	// Inicialización de todos los elementos del sistema
	init_System();

	while(1){
		if(printMsg > 4){
			writeMsg(&usart2Comm, "Hola mundo!\n");
			printMsg = 0;
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

	/* = = = INICIO DE CONFIGURACIÓN DEL BOTON DE USUARIO = = = */

	// Pin para el botón de usuario (PC13)

	handlerUserButton.pGPIOx								= GPIOC;
	handlerUserButton.GPIO_PinConfig.GPIO_PinNumber			= PIN_13;
	handlerUserButton.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	handlerUserButton.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	// Cargando la configuración
	GPIO_Config(&handlerUserButton);

	// Se configuran la interrupción del usuario para que se lancen con un flanco de subida
	handlerExtiUserButton.edgeType 		= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiUserButton.pGPIOHandler	= &handlerUserButton;

	// Se carga la configuración de la exti
	extInt_Config(&handlerExtiUserButton);

	/* = = = FIN DE CONFIGURACIÓN DEL BOTON DEL USUARIO = = = */

	/* = = = INICIO DE CONFIGURACIÓN DEL USART = = = */
	// Configuración del pin de transmisión
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber		= PIN_2;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;

	// Configuración del pin de recepción
	handlerPinRX.pGPIOx                               = GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber        = PIN_3;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode    = AF7;

	// Se cargan ambas configuraciones
	GPIO_Config(&handlerPinTX);
	GPIO_Config(&handlerPinRX);

	// Configuración del USART
	usart2Comm.ptrUSARTx                       = USART2;
	usart2Comm.USART_Config.USART_baudrate     = USART_BAUDRATE_19200;
	usart2Comm.USART_Config.USART_datasize     = USART_DATASIZE_8BIT;
	usart2Comm.USART_Config.USART_parity       = USART_PARITY_NONE;
	usart2Comm.USART_Config.USART_stopbits     = USART_STOPBIT_1;
	usart2Comm.USART_Config.USART_mode         = USART_MODE_RXTX;
	usart2Comm.USART_Config.USART_enableIntRX  = USART_RX_INTERRUPT_ENABLE;
	usart2Comm.USART_Config.USART_enableIntTX  = USART_TX_INTERRUPT_DISABLE;

	// Cargamos la configuración
	USART_Config(&usart2Comm);


	/* = = = = = INICIO DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */

	/* = = = = = FIN DE LA DEFINICIÓN DE LAS FUNCIONES = = = = = */
}
	/* = = = = = INICIO DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */

// Callback del timer 2 correspondiente al LED Blinky
void BasicTimer2_Callback(void){
	// Callback del blinky
	GPIOxTooglePin(&handlerLED2);
	printMsg++;
}

// Callback correspondiente al botón de usuario
void callback_extInt13(void){

	// Hacer algo...
	__NOP();
}

/* = = = = = FIN DE LAS RUTINAS DE ATENCIÓN (CALLBACKS) = = = = = */
