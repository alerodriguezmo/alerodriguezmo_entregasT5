/*
 * **********************************************************************************
 * @file		: SolucionTarea2Main.c
 * @author		: Alejandro Rodríguez Montes
 *
 * @brief		: Archivo principal (main)
 * **********************************************************************************
 *
 * Con este programa se da solución a la tarea 2 del curso Taller V (2023-01)
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
 * que se quiere leer. El resultado se almacena en la variable pinValue, donde se espera que el primer bit, que
 * será el estado del PinX, indique si se devuelve un valor de "0" ó "1".
 * Sin embargo, la función no tiene en cuenta los bits que se encuentran a la izquierda del bit correspondiente al
 * PinX en el Input Data Register, que bien pueden tener un valor distinto de 0 y que pueden no ser modificados por
 * el desplazamiento a la derecha. Esto afecta el valor de pinValue, haciendo que pueda devolver números diferentes de
 * "0" ó "1".
 *
 * Esto se puede solucionar con la implementación de una máscara que nos permita obtener el valor del primer bit de
 * la variable pinValue. Esta solución se encuentra implementada en el archivo GPIOxDriver.c
 */



/*  + + + + + = = = = = PUNTO 2 = = = = = + + + + +
 *
 * Se define la función GPIOxTooglePin en el archivo "GPIOxDriver.c", pues es una función que se va a utilizar de manera regular.
 * Para ver el desarrollo de la misma, referise al archivo mencionado.
 *
 */



// Definición de un elemento
GPIO_Handler_t handlerLed = {0}; // PA5

int main(void)
{
	// Configurando el LED_2 -> PA5
	handlerLed.pGPIOx									= GPIOA;
	handlerLed.GPIO_PinConfig.GPIO_PinNumber			= PIN_5;
	handlerLed.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	handlerLed.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerLed.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEED_FAST;
	handlerLed.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	// Cargamos la configuración del PinA5
	GPIO_Config(&handlerLed);

	GPIO_WritePin(&handlerLed, SET);

    /* Loop forever */
	while(1){
		GPIOxTooglePin(&handlerLed);

		for(int i = 0; i < 2000000 ; i++){
			NOP();

		}

	}

	return 0;
}
