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




int main(void)
{
    /* Loop forever */
	while(1){
		NOP();
	}

}
