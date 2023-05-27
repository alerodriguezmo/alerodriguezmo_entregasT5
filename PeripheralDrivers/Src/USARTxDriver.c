/*
 *************************************************************************
 * @file		: USARTxDriver.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Archivo de fuente del driver del periférico USARTx
 *
 *************************************************************************
 */

#include <stm32f4xx.h>
#include "USARTxDriver.h"

/**
 * Configurando el puerto Serial...
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periferico que se está utilizando.
 */

uint8_t currentData;
uint8_t auxRxData = 0;
//Crear variable y puntero para el caracter
char auxTxData;
uint8_t esstring = 0;//Mira si es string o no, por defecto es 1
uint8_t pos = 0;
char *ptrMessage;

void USART_Config(USART_Handler_t *ptrUsartHandler){


	/* 1. Activamos la señal de reloj que viene desde el BUS al que pertenece el periferico */
	/* Lo debemos hacer para cada uno de las pisbles opciones que tengamos (USART1, USART2, USART6) */
    /* 1.1 Configuramos el USART1 */
	if(ptrUsartHandler->ptrUSARTx == USART1){
		RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);
	}
	
    /* 1.2 Configuramos el USART2 */
    // Escriba acá su código
	else if(ptrUsartHandler->ptrUSARTx == USART2){
		RCC->APB1ENR |= (RCC_APB1ENR_USART2EN);
	}
    
    /* 1.3 Configuramos el USART6 */
	else if(ptrUsartHandler->ptrUSARTx == USART6){
		RCC->APB2ENR |= (RCC_APB2ENR_USART6EN);
	}

	/* 2. Configuramos el tamaño del dato, la paridad y los bit de parada */
	/* En el CR1 estan parity (PCE y PS) y tamaño del dato (M) */
	/* Mientras que en CR2 estan los stopbit (STOP)*/
	/* Configuracion del Baudrate (registro BRR) */
	/* Configuramos el modo: only TX, only RX, o RXTX, esto es TE y RE juntos? */
	/* Por ultimo activamos el modulo USART cuando todo esta correctamente configurado */

	// 2.1 Comienzo por limpiar los registros, para cargar la configuración desde cero
	ptrUsartHandler->ptrUSARTx->CR1 = 0;
	ptrUsartHandler->ptrUSARTx->CR2 = 0;

	// 2.2 Configuracion del Parity:
	// Verificamos si el parity esta activado o no
    // Tenga cuidado, el parity hace parte del tamaño de los datos...
	if(ptrUsartHandler->USART_Config.USART_parity != USART_PARITY_NONE){

			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_PCE; //Se hace el parity control enable
		// Verificamos si se ha seleccionado ODD or EVEN
		if(ptrUsartHandler->USART_Config.USART_parity == USART_PARITY_EVEN){
			// Es even, entonces cargamos la configuracion adecuada
			ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_PS;
			
		}else{
			// Si es "else" significa que la paridad seleccionada es ODD, y cargamos esta configuracion
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_PS;
		}
	}else{
		// Si llegamos aca, es porque no deseamos tener el parity-check
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_PCE;
	}

	// 2.3 Configuramos el tamaño del dato PREGUNTAR POR LOS STOP BIT
    if(ptrUsartHandler->USART_Config.USART_datasize == USART_DATASIZE_8BIT){
    	ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_M;
    }else{
    	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_M;
    }
	// 2.4 Configuramos los stop bits (SFR USART_CR2)
	switch(ptrUsartHandler->USART_Config.USART_stopbits){
	case USART_STOPBIT_1: {
		// Debemoscargar el valor 0b00 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 |= 0b00<<12;
		break;
	}
	case USART_STOPBIT_0_5: {
		// Debemoscargar el valor 0b01 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 |= 0b01<<12;
		break;
	}
	case USART_STOPBIT_2: {
		// Debemoscargar el valor 0b10 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 |= 0b10<<12;
		break;
	}
	case USART_STOPBIT_1_5: {
		// Debemoscargar el valor 0b11 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 |= 0b11<<12;
		break;
	}
	default: {
		// En el casopor defecto seleccionamos 1 bit de parada
		ptrUsartHandler->ptrUSARTx->CR2 |= 0b00<<12;
		break;
	}
	}

	// 2.5 Configuracion del Baudrate (SFR USART_BRR)
	// Ver tabla de valores (Tabla 73), Frec = 16MHz, overr = 0;
	switch(ptrUsartHandler->USART_Config.USART_freq){
	case 80:{
		if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
			//Valor a cargar 520.83, 0x208->Mantissa
			//.83333*16=13-> 0x208D
			ptrUsartHandler->ptrUSARTx->BRR = 0x208D;
		}

		else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
			//Valor a cargar 260.41667, 0x104->Mantissa
			//.41667*16=7-> 0x1047
			ptrUsartHandler->ptrUSARTx->BRR = 0x1047;
		}

		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
			//Valor a cargar 43.40278, 0x2B->Mantissa
			//.40278*16=6-> 0x02B6
			ptrUsartHandler->ptrUSARTx->BRR = 0x02B6;
		}
		break;
	}
	default:{
		if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
			// Valor a cargar es 104.1875 -> Mantissa = 104,fraction = 0.1875
			// Mantissa = 104 = 0x68, fraction = 16 * 0.1875 = 3
			// Valor a cargar 0x0683
			ptrUsartHandler->ptrUSARTx->BRR = 0x0683;
		}

		else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
			// Valor a cargar es 52.0625 -> Mantissa = 52,fraction = 0.0625
			// Mantissa = 52 = 0x34, fraction = 16 * 0.0625 = 1
			// Valor a cargar 0x0341
			ptrUsartHandler->ptrUSARTx->BRR = 0x0341;
		}

		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
			// El valor a cargar es 8.6875 -> Mantissa = 8,fraction = 0.6875
			// Mantissa = 8 = 0x8, fraction = 16 * 0.6875 = b
			// Valor a cargar 0x08B
			ptrUsartHandler->ptrUSARTx->BRR = 0x008B;
		}
		break;
	}
	}

	// 2.6 Configuramos el modo: TX only, RX only, RXTX, disable
	switch(ptrUsartHandler->USART_Config.USART_mode){
	case USART_MODE_TX:
	{
		// Activamos la parte del sistema encargada de enviar
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Limpio Tx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TE; 	// Escribo Tx
		break;
	}
	case USART_MODE_RX:
	{
		// Activamos la parte del sistema encargada de recibir
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Limpio Rx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RE; 	// Escribo Rx
		break;
	}
	case USART_MODE_RXTX:
	{
		// Activamos ambas partes, tanto transmision como recepcion
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Limpio Rx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RE; 	// Escribo Rx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Limpio Tx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TE; 	// Escribo Tx
		break;
	}
	case USART_MODE_DISABLE:
	{
		// Desactivamos ambos canales
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Rx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Tx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_UE;
		break;
	}

	default:
	{
		// Actuando por defecto, desactivamos ambos canales
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Rx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Tx
		break;
	}
	}
	//Se desactivan interrup. globales
	__disable_irq();
	//Se activan o desactivan interrupciones de transmision o recepcion
	if(ptrUsartHandler->USART_Config.USART_enableIntRX == USART_RX_INTERRUPT_ENABLE){
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RXNEIE;
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RXNEIE;
	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RXNEIE;
	}

	// Interrupción por transmisión
	if(ptrUsartHandler->USART_Config.USART_enableIntTX == USART_TX_INTERRUPT_ENABLE){
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;
	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;
	}
	//Se matricula en el NVIC
	if(ptrUsartHandler->ptrUSARTx == USART1){
		__NVIC_EnableIRQ(USART1_IRQn);
	}
	else if(ptrUsartHandler->ptrUSARTx == USART2){
		__NVIC_EnableIRQ(USART2_IRQn);
	}
	else if(ptrUsartHandler->ptrUSARTx == USART6){
		__NVIC_EnableIRQ(USART6_IRQn);
	}

	//Se activan interrup globales
	__enable_irq();
	// 2.7 Activamos el modulo serial.
	if(ptrUsartHandler->USART_Config.USART_mode != USART_MODE_DISABLE){
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_UE;
	}
}

/* funcion para escribir un solo char */
//int writeChar(USART_Handler_t *ptrUsartHandler, char dataToSend ){
//	while( !(ptrUsartHandler->ptrUSARTx->SR & USART_SR_TXE)){
//		__NOP();
//	}
//	ptrUsartHandler->ptrUSARTx->DR = dataToSend;
//	return dataToSend;
//}
//
///** Funcion para escribir todo un string */
//void writeMsg(USART_Handler_t *ptrUsartHandler, char *messageToSend){
//	int i = 0;
//	while(messageToSend[i] != '\0'){
//		writeChar(ptrUsartHandler, messageToSend[i]);
//		i++;
//	}
//}
uint8_t getRxData(void){
	return auxRxData;
}
int writeCharTXE(USART_Handler_t *ptrUsartHandler, char dataToSend ){
	auxTxData = dataToSend;
	esstring = 0;
	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;
	return dataToSend;

}
void writeMessageTXE(USART_Handler_t *ptrUsartHandler, char *stringToSend){
	ptrMessage = stringToSend;
	esstring=1;
	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;

}

void USART1_IRQHandler(void){
	if(USART1->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART1->DR;
		usart1Rx_Callback();
	}else if(USART1->SR&USART_SR_TXE){//Si es una transmisión
		if(!esstring){//Si es un char
			USART1->DR = auxTxData;//Se escribe el dato
			USART1->CR1 &= ~USART_CR1_TXEIE;//Se baja el IE
		}else{//Si es un string
			if(ptrMessage[pos]!='\0'){
				USART1->DR = ptrMessage[pos];
				pos++;
			}else{
				USART1->CR1 &= ~USART_CR1_TXEIE;
				pos = 0;
			}

		}
	}
}
void USART2_IRQHandler(void){
	if(USART2->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART2->DR;
		usart2Rx_Callback();
	}else if(USART2->SR&USART_SR_TXE){//Si es una transmisión
		if(!esstring){//Si es un char
			USART2->DR = auxTxData;//Se escribe el dato
			USART2->CR1 &= ~USART_CR1_TXEIE;//Se baja el IE
		}else{//Si es un string
			if(esstring){
				if(ptrMessage[pos]!='\0'){
					USART2->DR = ptrMessage[pos];
					pos++;
				}else{
					USART2->CR1 &= ~USART_CR1_TXEIE;
					pos = 0;
				}
			}
		}
}}
void USART6_IRQHandler(void){
	if(USART6->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART6->DR;
		usart6Rx_Callback();
	}
	else if(USART6->SR&USART_SR_TXE){//Si es una transmisión
		if(!esstring){//Si es un char
			USART6->DR = auxTxData;//Se escribe el dato
			USART6->CR1 &= ~USART_CR1_TXEIE;//Se baja el IE
		}else{//Si es un string
			if(esstring){
				if(ptrMessage[pos]!='\0'){
					USART6->DR = ptrMessage[pos];
					pos++;
				}else{
					USART6->CR1 &= ~USART_CR1_TXEIE;
					pos = 0;
				}
			}
		}
}}

__attribute__((weak)) void usart1Rx_Callback(void){
	__NOP();
}
__attribute__((weak)) void usart2Rx_Callback(void){
	__NOP();
}
__attribute__((weak)) void usart6Rx_Callback(void){
	__NOP();
}
