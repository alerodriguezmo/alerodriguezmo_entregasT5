/*
 *************************************************************************
 * @file		: I2CDriver.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Archivo de fuente del driver del periférico I2C
 *
 *************************************************************************
 */

#include <stdint.h>
#include "I2CDriver.h"

//Se configuran 2 pines, uno para SDA y otro SCL , se configura como altfunction
//Tambien se configuran como salidas open drain y resistencias modo pull-up

void i2c_config(I2C_Handler_t *ptrHandlerI2C){
	//Se activa CS
	if(ptrHandlerI2C->ptrI2Cx == I2C1){
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	}
	else if(ptrHandlerI2C->ptrI2Cx == I2C2){
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}
	else if(ptrHandlerI2C->ptrI2Cx == I2C3){
		RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	}
	//Se reinicia el periferico para que inicie en un estado conocido
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_SWRST;
	__NOP();
	ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_SWRST;

	//3. Se indica la velocidad de reloj principal
	ptrHandlerI2C->ptrI2Cx->CR2 &= ~(0b111111 << I2C_CR2_FREQ_Pos);//Se borra conf anterior
	//Se selecciona dependiendo la frecuencia a la que esté el APB1
	if(ptrHandlerI2C->freq==40){//Si está a 40MHz
		ptrHandlerI2C->ptrI2Cx->CR2 |= (40<<I2C_CR2_FREQ_Pos);
	}else{//Por default es 16MHz
		ptrHandlerI2C->ptrI2Cx->CR2 |= (MAIN_CLOCK_16MHz_FOR_I2C << I2C_CR2_FREQ_Pos);
	}

	//4. Se configura el modo I2C en el que sistema funciona
	ptrHandlerI2C->ptrI2Cx->CCR = 0;
	ptrHandlerI2C->ptrI2Cx->TRISE = 0;

	if(ptrHandlerI2C->freq == 40){
		if(ptrHandlerI2C->modeI2C == I2C_MODE_SM){
			//se configura el modo estándar
			ptrHandlerI2C->ptrI2Cx->CCR &= ~I2C_CCR_FS;
			ptrHandlerI2C->ptrI2Cx->CCR |= (200 <<I2C_CCR_CCR_Pos);
			ptrHandlerI2C->ptrI2Cx->TRISE |= 41;
		}else{
			//se configura el modo rápido
			ptrHandlerI2C->ptrI2Cx->CCR |= I2C_CCR_FS;
			ptrHandlerI2C->ptrI2Cx->CCR |= (33 <<I2C_CCR_CCR_Pos);
			ptrHandlerI2C->ptrI2Cx->TRISE |= 13;
		}
	}
	else{

		if(ptrHandlerI2C->modeI2C == I2C_MODE_SM){
			//se configura el modo estándar
			ptrHandlerI2C->ptrI2Cx->CCR &= ~I2C_CCR_FS;
			ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_SM_SPEED_100KHz <<I2C_CCR_CCR_Pos);
			ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_SM;
		}else{
			//se configura el modo rápido
			ptrHandlerI2C->ptrI2Cx->CCR |= I2C_CCR_FS;
			ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_FM_SPEED_400KHz <<I2C_CCR_CCR_Pos);
			ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM;
		}
	}

	// 5. Se activa el módulo I2C
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_PE;
}

void i2c_stopTransaction(I2C_Handler_t *ptrHandlerI2C){
	//8. Se genera la condición de stop
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_STOP;
}
// 1. Se verifica que la linea no este ocupada, bit busy en I2C_CR2
// 2. Se genera la señal start
// 2a. Se espera que se levante la bandera start, mientras se espera el valor de SB es 0
void i2c_startTransaction(I2C_Handler_t *ptrHandlerI2C){
	//Se verifica que la linea no este busy
	while(ptrHandlerI2C->ptrI2Cx->SR2 & I2C_SR2_BUSY){
		__NOP();
	}
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_START;
	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_SB)){
		__NOP();
	}
}
void i2c_reStartTransaction(I2C_Handler_t *ptrHandlerI2C){
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_START;

	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_SB)){
		__NOP();
	}
}
//7a. Se activa la indicación para no-ACK
void i2c_sendNoAck(I2C_Handler_t *ptrHandlerI2C){
	ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_ACK;
}
//7a. Se activa la indicación para no-ACK
void i2c_sendAck(I2C_Handler_t *ptrHandlerI2C){
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_ACK;
}

void i2c_sendSlaveAddressRW(I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite){
	// Se define var. aux
	uint8_t auxByte = 0;
	(void) auxByte;

	//Se envia la direccion del Slave y el bit que indica que se quiere escribir (0)
	ptrHandlerI2C->ptrI2Cx->DR = (slaveAddress<<1) | readOrWrite;

	//Se espera hasta que la bandera del addr se levante
	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_ADDR)){
	__NOP();
	}
	//Se limpia la bandera de recepcion de ACK de la addr, se lee primero SR1 y luego SR2
	auxByte = ptrHandlerI2C->ptrI2Cx->SR1;
	auxByte = ptrHandlerI2C->ptrI2Cx->SR2;
}

void i2c_sendMemoryAddress(I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr){
	//Enviamos la direccion de memoria que deseamos leer
	ptrHandlerI2C->ptrI2Cx->DR = memAddr;

	//Se espera hasta que el byte sea transmitido
	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_TXE)){
		__NOP();
	}
}

void i2c_sendDataByte(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite){
	//Se carga el valor que se desea escribir
	ptrHandlerI2C->ptrI2Cx->DR = dataToWrite;
	//Se espera hasta que el byte sea transmitido
	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_BTF)){
		__NOP();
	}
}

uint8_t i2c_readDataByte(I2C_Handler_t *ptrHandlerI2C){
	//Se espera hasta que el bit entrante sea recibido
	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_RXNE)){
		__NOP();
	}

	ptrHandlerI2C->dataI2C = ptrHandlerI2C->ptrI2Cx->DR;
	return ptrHandlerI2C->dataI2C;
}

uint8_t i2c_readSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead){
	//variable aux
	uint8_t auxRead = 0;
	//Se genera el start
	i2c_startTransaction(ptrHandlerI2C);
	//Se envia la direccion del slave y se indica ESCRIBIR
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_WRITE_DATA);
	//Se envia la direccion de memoria que se desea leer
	i2c_sendMemoryAddress(ptrHandlerI2C, regToRead);
	//Se crea condición de restart
	i2c_reStartTransaction(ptrHandlerI2C);
	//Enviamos la direccion del slave y la indicacion de leer
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress,I2C_READ_DATA);
	//leemos el dato que envia el slave
	auxRead = i2c_readDataByte(ptrHandlerI2C);
	//Se genera NoAck para que solo se envie 1 byte
	i2c_sendNoAck(ptrHandlerI2C);
	//Se genera condición de stop para que se detenga despues de solo 1 byte
	i2c_stopTransaction(ptrHandlerI2C);
	return auxRead;
}

void i2c_writeSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead, uint8_t newValue){
	//Se genera el start
	i2c_startTransaction(ptrHandlerI2C);
	//Se envia la direccion del slave y se indica ESCRIBIR
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_WRITE_DATA);
	//Se envia la direccion de memoria que se desea leer
	i2c_sendMemoryAddress(ptrHandlerI2C, regToRead);
	//Se envia el valor que se desea escribir en el registro seleccionado
	i2c_sendDataByte(ptrHandlerI2C, newValue);
	//Se genera la condición stop para que el slave se detenga después de 1 byte
	i2c_stopTransaction(ptrHandlerI2C);
}






