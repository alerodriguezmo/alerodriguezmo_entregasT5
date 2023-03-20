/*
 * **********************************************************************************
 * @file		: SolucionTarea2Main.c
 * @author		: Alejandro Rodríguez Montes
 *
 * @brief		: Archivo principal (main)
 * **********************************************************************************
 *
 * Con este archivo se da solución a la tarea 2 del curso Taller V (2023-01)
 *
 ************************************************************************************
 */
#include <stdint.h>

#include "stm32f411xx_hal.h"
#include "GPIOxDriver.h"

/*  + + + + + = = = = = PUNTO 1 = = = = = + + + + +
 *
 * En el desarrollo mostrado en el video del Driver para el puerto GPIOx, la función GPIO_ReadPin()
 * tiene un error, el cuál no nos permite obtener el valor real del PinX que estamos leyendo.
 * La función accede al registro IDR (Input Data Register) del puerto GPIO. El valor de este registro
 * representa el estado actual de los pines del puerto GPIO en el momento de la lectura.
 * Luego, se desplaza a la derecha el valor del registro IDR tantas veces como la ubicación del pin específico
 * que se quiere leer. El resultado se almacena en la variable pinValue, donde se espera que el número binario,
 * indique si el estado del PinX es de "0" ó "1".
 *
 * Sin embargo, la función no tiene en cuenta los bits que se encuentran a la izquierda del bit correspondiente al
 * PinX en el Input Data Register, que bien pueden tener un valor distinto de 0 y que no son modificados por
 * el desplazamiento a la derecha. Esto afecta el valor binario de pinValue, haciendo que pueda devolver números diferentes de
 * "0" ó "1".
 *
 * En otras palabras, la función retorna un binario cuyo primer bit es el estado del pin, pero que no representa el
 * verdadero estado del pin.
 *
 * Esto se soluciona con la implementación de una máscara que permita obtener el valor del primer bit de
 * la variable pinValue. Esta solución se encuentra implementada en el archivo GPIOxDriver.c
 */



/*  + + + + + = = = = = PUNTO 2 = = = = = + + + + +
 *
 * Se define la función GPIOxTooglePin en el archivo "GPIOxDriver.c", pues es una función que se va a utilizar de manera regular.
 * Para ver el desarrollo de la misma, referise al archivo mencionado.
 *
 */


/*  + + + + + = = = = = PUNTO 3 = = = = = + + + + +
 *  Programa para un contador de segundos binario up-down */

/* = = = = = INICIO DEL PROGRAMA = = = = = */
int main(void){
	// Se definen los pines {PC9,PC6,PB8,PA6,PC7,PC8,PA7} como salidas
	// de propósito general y el {P13} como la comunicación con el botón.
	// Se crean y se definen sus respectivos handlers para que incien todos en 0.

	GPIO_Handler_t handlerPC9		= {0};
	GPIO_Handler_t handlerPC6		= {0};
	GPIO_Handler_t handlerPB8		= {0};
	GPIO_Handler_t handlerPA6		= {0};
	GPIO_Handler_t handlerPC7		= {0};
	GPIO_Handler_t handlerPC8		= {0};
	GPIO_Handler_t handlerPA7		= {0};
	GPIO_Handler_t handlerPC13		= {0};

	/* = = = = = INICIO DE LA CONFIGURACIÓN DE LOS PINES = = = = = */

	// Parámetros para la configuración del PC9
	handlerPC9.pGPIOx									= GPIOC;
	handlerPC9.GPIO_PinConfig.GPIO_PinNumber			= PIN_9;
	handlerPC9.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerPC9.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPC9.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPC9.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_MEDIUM;
	handlerPC9.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PC6
	handlerPC6.pGPIOx									= GPIOC;
	handlerPC6.GPIO_PinConfig.GPIO_PinNumber			= PIN_6;
	handlerPC6.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerPC6.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPC6.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPC6.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_MEDIUM;
	handlerPC6.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PB8
	handlerPB8.pGPIOx									= GPIOB;
	handlerPB8.GPIO_PinConfig.GPIO_PinNumber			= PIN_8;
	handlerPB8.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerPB8.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPB8.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPB8.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_MEDIUM;
	handlerPB8.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PA6
	handlerPA6.pGPIOx									= GPIOA;
	handlerPA6.GPIO_PinConfig.GPIO_PinNumber			= PIN_6;
	handlerPA6.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerPA6.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPA6.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPA6.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_MEDIUM;
	handlerPA6.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PC7
	handlerPC7.pGPIOx									= GPIOC;
	handlerPC7.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerPC7.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerPC7.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPC7.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPC7.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_MEDIUM;
	handlerPC7.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PC8
	handlerPC8.pGPIOx									= GPIOC;
	handlerPC8.GPIO_PinConfig.GPIO_PinNumber			= PIN_8;
	handlerPC8.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerPC8.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPC8.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPC8.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_MEDIUM;
	handlerPC8.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PA7
	handlerPA7.pGPIOx									= GPIOA;
	handlerPA7.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerPA7.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerPA7.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPA7.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPA7.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_MEDIUM;
	handlerPA7.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Parámetros para la configuración del PC13 (botón)
	handlerPC13.pGPIOx									= GPIOC;
	handlerPC13.GPIO_PinConfig.GPIO_PinNumber			= PIN_13;
	handlerPC13.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_IN;
	handlerPC13.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPC13.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_MEDIUM;
	handlerPC13.GPIO_PinConfig.GPIO_PinAltFunMode		= AF0;

	// Se cargan las configuraciones de los pines {PC9,PC6,PB8,PA6,PC7,PC8,PA7} y {PC13}
	GPIO_Config(&handlerPC9);
	GPIO_Config(&handlerPC6);
	GPIO_Config(&handlerPB8);
	GPIO_Config(&handlerPA6);
	GPIO_Config(&handlerPC7);
	GPIO_Config(&handlerPC8);
	GPIO_Config(&handlerPA7);
	GPIO_Config(&handlerPC13);

	/* = = = = = FIN DE LA CONFIGURACIÓN DE LOS PINES = = = = = */


	/* = = = = = INICIO DE LA SECCIÓN DE CONTADORES Y AUXILIARES = = = = = */

	// Se crea un contador principal
	uint32_t mainCounter		= 1;

	// Se asignan variables C1 a C7 para almacenar los bits individuales del contador principal.
	// Estas variables establecen ademas el encendido/apagado de los pines y, por ende, los LEDs.
	uint32_t C1					= 0;
	uint32_t C2					= 0;
	uint32_t C3					= 0;
	uint32_t C4					= 0;
	uint32_t C5					= 0;
	uint32_t C6					= 0;
	uint32_t C7					= 0;

	// Variable que almacena el estado del PC13, que es el botón del usuario.
	uint8_t state;


	/* = = = = = FIN DE LA SECCIÓN DE CONTADORES Y AUXILIARES = = = = = */

	/* = = = = = INICIO DEL CORE DEL PROGRAMA = = = = = */
	while(1){
		// Ciclo que nos permite generar un delay de aproximadamente 1 segundo
		for(uint32_t i = 1 ; i < 1250000 ; i++){
			NOP();
		}
		// Switch que permite llevar el contador principal a 60 si llega a 1
		// y a 1 si llega a 60
		switch(mainCounter){

		case(0):
			mainCounter = 60;
			break;

		case(61):
			mainCounter = 1;
			break;

		default:
			break;
		}

		// Cada variable Cx se establece extrayendo el bit correspondiente del contador principal utilizando
		// la operación de desplazamiento hacia la derecha (>>) y la operación AND (&) con el valor binario 1.
		C1 			= mainCounter & (1);
		C2 			= (mainCounter >> 1) & (1);
		C3 			= (mainCounter >> 2) & (1);
		C4 			= (mainCounter >> 3) & (1);
		C5 			= (mainCounter >> 4) & (1);
		C6 			= (mainCounter >> 5) & (1);
		C7 			= (mainCounter >> 6) & (1);

		// Se apagan y se prenden los pines/LEDs a conveniencia
		GPIO_WritePin(&handlerPA7, C1);
		GPIO_WritePin(&handlerPC8, C2);
		GPIO_WritePin(&handlerPC7, C3);
		GPIO_WritePin(&handlerPA6, C4);
		GPIO_WritePin(&handlerPB8, C5);
		GPIO_WritePin(&handlerPC6, C6);
		GPIO_WritePin(&handlerPC9, C7);

		// Lectura del estado del botón de usuario
		state 		= GPIO_ReadPin(&handlerPC13);

		// Esta línea incluye la acción del usuario en el programa. Si el botón está presionado,
		// la variable state es 0. En caso contrario es 1
		if(state){
			// Si el botón no está presionado, se aumenta el valor.
			mainCounter++;
		} else{
			// Si el botón está presionado, se disminuye el valor.
			mainCounter--;
		}

	}
	/* = = = = = FIN DEL CORE DEL PROGRAMA = = = = = */
}
/* = = = = = FIN DEL PROGRAMA = = = = = */
