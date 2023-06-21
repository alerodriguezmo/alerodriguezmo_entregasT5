/**
 * **************************************************************************************************
 * @file     : Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @author   : I2CDriver.h
 * @brief    : Archivo de cabecera del driver del periférico I2C
 * **************************************************************************************************
 */


#ifndef I2CDRIVER_H_
#define I2CDRIVER_H_

#include <stm32f4xx.h>

#define I2C_WRITE_DATA		0
#define I2C_READ_DATA		1

#define MAIN_CLOCK_4_MHz_FOR_I2C	4
#define MAIN_CLOCK_16_MHz_FOR_I2C	16
#define MAIN_CLOCK_20_MHz_FOR_I2C	20
#define MAIN_CLOCK_80_MHz_FOR_I2C	80
#define MAIN_CLOCK_100_MHz_FOR_I2C	100

#define I2C_MODE_SM			0
#define I2C_MODE_FM			1

#define I2C_MODE_SM_SPEED_100KHz_16MHz	80
#define I2C_MODE_FM_SPEED_400KHz_16MHz	14
#define I2C_MODE_SM_SPEED_100KHz_80MHz	400
#define I2C_MODE_FM_SPEED_400KHz_80MHz	67
#define I2C_MODE_SM_SPEED_100KHz_100MHz	500
#define I2C_MODE_FM_SPEED_400KHz_100MHz	83

#define I2C_MAX_RISE_TIME_SM_16MHZ		17
#define I2C_MAX_RISE_TIME_FM_16MHz		6
#define I2C_MAX_RISE_TIME_SM_80MHZ		81
#define I2C_MAX_RISE_TIME_FM_80MHz		25
#define I2C_MAX_RISE_TIME_SM_100MHZ		101
#define I2C_MAX_RISE_TIME_FM_100MHz		31


typedef struct
{
	I2C_TypeDef		*ptrI2Cx;
	uint8_t			slaveAddress;
	uint8_t			modeI2C;
	uint8_t			dataI2C;
	uint8_t			mainClock;
	uint8_t 		modeI2C_SM;
	uint8_t			modeI2C_FM;
	uint8_t 		maxI2C_SM;
	uint8_t			maxI2C_FM;
}I2C_Handler_t;

/*prototipos de las funciones públicas*/
void i2c_config(I2C_Handler_t *ptrHandlerI2C);
void i2c_startTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_reStartTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendSlaveAddressRW(I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite);
void i2c_sendMemoryAddress(I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr);
void i2c_sendDataByte(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite);
uint8_t i2c_readDataByte(I2C_Handler_t *ptrHandlerI2C);
void i2c_stopTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendACK(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendNoACK(I2C_Handler_t *ptrHandlerI2C);

uint8_t i2c_readSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead);
uint8_t i2c_readSingleRegister_16bits(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead);
uint8_t i2c_readMultipleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead);
void i2c_writeSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead, uint8_t newValue);
void i2c_writeSingleRegister_16bits(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead, uint16_t newValue);

#endif /* I2CDRIVER_H_ */
