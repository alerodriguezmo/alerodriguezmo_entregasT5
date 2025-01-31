/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : GPIOx.h
 * @brief    : Archivo de cabecera del driver del periférico GPIOx
 * **************************************************************************************************
 */

#ifndef INC_GPIOXDRIVER_H_
#define INC_GPIOXDRIVER_H_

//Incluyendo este archivo estamos incluyendo también el correspondiente al GPIOx
#include <stm32f4xx.h>
#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3

#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1

#define GPIO_OTYPE_PUSHPULL    0
#define GPIO_OTYPE_OPENDRAIN   1

#define GPIO_OSPEED_LOW       0
#define GPIO_OSPEED_MEDIUM    1
#define GPIO_OSPEED_FAST      2
#define GPIO_OSPEED_HIGH      3

#define GPIO_PUPDR_NOTHING    0
#define GPIO_PUPDR_PULLUP     1
#define GPIO_PUPDR_PULLDOWN   2
#define GPIO_PUPDR_RESERVED   3

#define PIN_0        0
#define PIN_1        1
#define PIN_2        2
#define PIN_3        3
#define PIN_4        4
#define PIN_5        5
#define PIN_6        6
#define PIN_7        7
#define PIN_8        8
#define PIN_9        9
#define PIN_10       10
#define PIN_11       11
#define PIN_12       12
#define PIN_13       13
#define PIN_14       14
#define PIN_15       15

#define AF0     0b0000
#define AF1     0b0001
#define AF2     0b0010
#define AF3     0b0011
#define AF4     0b0100
#define AF5     0b0101
#define AF6     0b0110
#define AF7     0b0111
#define AF8     0b1000
#define AF9     0b1001
#define AF10    0b1010
#define AF11    0b1011
#define AF12    0b1100
#define AF13    0b1101
#define AF14    0b1110
#define AF15    0b1111

typedef struct
{
	uint8_t GPIO_PinNumber;        //PIN con el que deseamos trabajar
	uint8_t GPIO_PinMode;          //Modo de la configuración: entrada, salida, análogo, f.a
	uint8_t GPIO_PinSpeed;         //Velocidad de respuesta del PIN (solo para digital)
	uint8_t GPIO_PinPuPdControl;   //Seleccionamos si deseamos una salida Pull-up, Pull-down o "libre"
	uint8_t GPIO_PinOPType;        //Trabaja de la mano con el anterior, selecciona salida PUPD o OpenDrain
	uint8_t GPIO_PinAltFunMode;    //Selecciona cual es la función alt.que se está configurando

}GPIO_PinConfig_t;

/* Esta es una estructura que contiene dos elementos:
 * -La dirección del puerto que se está utilizando (la referencia al puerto)
 * -La configuración específica del PIN que se está utilizando
 */
typedef struct
{
	GPIO_TypeDef    *pGPIOx; /*!< Direccion del puerto al que el PIN corresponde >*/
	GPIO_PinConfig_t  GPIO_PinConfig; /*!< Configuracion del PIN >*/

}GPIO_Handler_t;

/*Definicion de las cabeceras de las funciones del GPIOxDriver */
void GPIO_Config (GPIO_Handler_t *pGPIOHandler);
void GPIO_WritePin(GPIO_Handler_t *pPinHandler, uint8_t newState);
uint32_t GPIO_ReadPin(GPIO_Handler_t *pPinHandler);
void GPIOxTooglePin(GPIO_Handler_t*pPinHandler);


#endif /* INC_GPIOXDRIVER_H_ */
