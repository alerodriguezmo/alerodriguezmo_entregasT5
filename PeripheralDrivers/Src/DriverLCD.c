/*
 *************************************************************************
 * @file		: DriverLCD.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Archivo de fuente del driver del display LCD
 * 				  HD44780 con la interfaz I2C PCF8574
 *
 *************************************************************************
 */

#include <DriverLCD.h>
#include <stdint.h>

/*
 * Se debe configurar los pines para el I2Cx (SDA y SCL),
 * teniendo en cuenta cuáles pines tienen la función alternativa
 * correspondiente al I2Cx a usar.
 *
 * Los pines se deben configurar como open drain y con resistencia de pull up.
 */

void sendCMD_toLCD(I2C_Handler_t *ptrHandlerI2C, char cmd){
	char _U;
	char _L;
	uint8_t _T[4];
	_U=(cmd & 0xf0);
	_L=((cmd<<4) & 0xf0);
	_T[0] = _U|0x0C;
	writeData_inLCD(ptrHandlerI2C, _T[0]);
	_T[1] = _U|0x08;
	writeData_inLCD(ptrHandlerI2C, _T[1]);
	_T[2] = _L|0x0C;
	writeData_inLCD(ptrHandlerI2C, _T[2]);
	_T[3] = _L|0x08;
	writeData_inLCD(ptrHandlerI2C, _T[3]);
}

void writeData_inLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite){

	/* 1. Generamos la condición de Start*/
	i2c_startTransaction(ptrHandlerI2C);

	/* 2. Enviamos la diracción del esclavo y la indicación de ESCRIBIR */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria que deseamos escribir*/
	i2c_sendDataByte(ptrHandlerI2C, dataToWrite);

	i2c_stopTransaction(ptrHandlerI2C);
}


void sendData_toLCD(I2C_Handler_t *ptrHandlerI2C, char data){
	char _U;
	char _L;
	uint8_t _T[4];
	_U=(data & 0xf0);
	_L=((data<<4) & 0xf0);
	_T[0] = _U|0x0D;
	writeData_inLCD(ptrHandlerI2C, _T[0]);
	_T[1] = _U|0x09;
	writeData_inLCD(ptrHandlerI2C, _T[1]);
	_T[2] = _L|0x0D;
	writeData_inLCD(ptrHandlerI2C, _T[2]);
	_T[3] = _L|0x09;
	writeData_inLCD(ptrHandlerI2C, _T[3]);
}

void clearLCD(I2C_Handler_t *ptrHandlerI2C){
	sendCMD_toLCD(ptrHandlerI2C, 0x01);
	delay_50();
}

void init_LCD(I2C_Handler_t *ptrHandlerI2C){

	// Delay de inizializacion
	delay_50();
	// Primer 0x30 para BF
	sendCMD_toLCD(ptrHandlerI2C, 0x30);
	delay_5();
	// Segundo 0x30 para BF
	sendCMD_toLCD(ptrHandlerI2C, 0x30);
	delay_1();
	// Tercer 0x30 para BF
	sendCMD_toLCD(ptrHandlerI2C, 0x30);

	// Delay después de la secuencia inicial de inicializacion
	delay_50();

	/*
	 * Configuraciones para la escritura
	 */
	// Data lenght 4, lines 2, character font 5X8
	sendCMD_toLCD(ptrHandlerI2C, 0x20);
	delay_50();
	sendCMD_toLCD(ptrHandlerI2C, 0x28);
	delay_50();
	// Display off
	sendCMD_toLCD(ptrHandlerI2C, 0x08);
	delay_50();
	// Display Clear
	sendCMD_toLCD(ptrHandlerI2C, 0x01);
	delay_50();
	// Entry mode
	sendCMD_toLCD(ptrHandlerI2C, 0x06);

	delay_50();
	// Delay para encendido
	sendCMD_toLCD(ptrHandlerI2C, 0x0C);
}

void sendSTR_toLCD(I2C_Handler_t *ptrHandlerI2C, char *str){
	while (*str) sendData_toLCD(ptrHandlerI2C, *str++);
}

void moveCursor_inLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t x, uint8_t y){
	uint8_t cursor;
	switch (x) {
	case 0 :
		switch (y) {
		case 0 : cursor = 0x00; break;
		case 1 : cursor = 0x01; break;
		case 2 : cursor = 0x02; break;
		case 3 : cursor = 0x03; break;
		case 4 : cursor = 0x04; break;
		case 5 : cursor = 0x05; break;
		case 6 : cursor = 0x06; break;
		case 7 : cursor = 0x07; break;
		case 8 : cursor = 0x08; break;
		case 9 : cursor = 0x09; break;
		case 10 : cursor = 0x0A; break;
		case 11 : cursor = 0x0B; break;
		case 12 : cursor = 0x0C; break;
		case 13 : cursor = 0x0D; break;
		case 14 : cursor = 0x0E; break;
		case 15 : cursor = 0x0F; break;
		case 16 : cursor = 0x10; break;
		case 17 : cursor = 0x11; break;
		case 18 : cursor = 0x12; break;
		case 19 : cursor = 0x13; break;
		} break;

	case 1 :
		switch (y) {
		case 0 : cursor = 0x40; break;
		case 1 : cursor = 0x41; break;
		case 2 : cursor = 0x42; break;
		case 3 : cursor = 0x43; break;
		case 4 : cursor = 0x44; break;
		case 5 : cursor = 0x45; break;
		case 6 : cursor = 0x46; break;
		case 7 : cursor = 0x47; break;
		case 8 : cursor = 0x48; break;
		case 9 : cursor = 0x49; break;
		case 10 : cursor = 0x4A; break;
		case 11 : cursor = 0x4B; break;
		case 12 : cursor = 0x4C; break;
		case 13 : cursor = 0x4D; break;
		case 14 : cursor = 0x4E; break;
		case 15 : cursor = 0x4F; break;
		case 16 : cursor = 0x50; break;
		case 17 : cursor = 0x51; break;
		case 18 : cursor = 0x52; break;
		case 19 : cursor = 0x53; break;
		} break;

	case 2 :
		switch (y) {
		case 0 : cursor = 0x14; break;
		case 1 : cursor = 0x15; break;
		case 2 : cursor = 0x16; break;
		case 3 : cursor = 0x17; break;
		case 4 : cursor = 0x18; break;
		case 5 : cursor = 0x19; break;
		case 6 : cursor = 0x1A; break;
		case 7 : cursor = 0x1B; break;
		case 8 : cursor = 0x1C; break;
		case 9 : cursor = 0x1D; break;
		case 10 : cursor = 0x1E; break;
		case 11 : cursor = 0x1F; break;
		case 12 : cursor = 0x20; break;
		case 13 : cursor = 0x21; break;
		case 14 : cursor = 0x22; break;
		case 15 : cursor = 0x23; break;
		case 16 : cursor = 0x24; break;
		case 17 : cursor = 0x25; break;
		case 18 : cursor = 0x26; break;
		case 19 : cursor = 0x27; break;
		} break;

	case 3 :
		switch (y) {
		case 0 : cursor = 0x54; break;
		case 1 : cursor = 0x55; break;
		case 2 : cursor = 0x56; break;
		case 3 : cursor = 0x57; break;
		case 4 : cursor = 0x58; break;
		case 5 : cursor = 0x59; break;
		case 6 : cursor = 0x5A; break;
		case 7 : cursor = 0x5B; break;
		case 8 : cursor = 0x5C; break;
		case 9 : cursor = 0x5D; break;
		case 10 : cursor = 0x5E; break;
		case 11 : cursor = 0x5F; break;
		case 12 : cursor = 0x60; break;
		case 13 : cursor = 0x61; break;
		case 14 : cursor = 0x62; break;
		case 15 : cursor = 0x63; break;
		case 16 : cursor = 0x64; break;
		case 17 : cursor = 0x65; break;
		case 18 : cursor = 0x66; break;
		case 19 : cursor = 0x67; break;
		} break;
	}
	sendCMD_toLCD(ptrHandlerI2C, 0x80|cursor);
}

void delay_50 (void){
	for (int i=0;i<62500;i++){
		__NOP();
	}
}

void delay_5 (void){
	for (int i=0; i<6250; i++){
		__NOP();
	}
}

void delay_1 (void){
	for (int i=0;i<1250;i++){
		__NOP();
	}
}

void delay_10 (void){
	for (int i=0;i<12500;i++){
		__NOP();
	}
}

void clearScreenLCD(I2C_Handler_t *ptrHandlerI2C){
	char DataClean[64] = "                    ";
	for(int i=0;i<4;i++){
		moveCursor_inLCD(ptrHandlerI2C, i, 0);
		sendSTR_toLCD(ptrHandlerI2C,DataClean);
	}
}
