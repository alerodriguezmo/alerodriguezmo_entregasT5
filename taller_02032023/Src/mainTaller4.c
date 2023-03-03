/*
 * mainTaller4.c
 *
 *  Created on: Mar 2, 2023
 *      Author: ingfisica
 */

#include <stdint.h>

char var1 = 0;
int var2 = 0;
short var3 = 0;
long var4 = 0;

uint8_t var5 = 0;
int8_t var6 = 0;
uint16_t var7 = 0;
int16_t var8 = 0;
uint32_t var9 = 0;
int32_t var10 = 0;
uint64_t var11 = 0;
int64_t var12 = 0;
uint16_t resultado = 0;

// Funcion Main
int main(void){
	uint16_t testShift = 0b000001010110101;
	uint16_t testMask =  0b000000001001010;

    /* Loop forever */

	while(1){

		uint16_t resultado = testShift | testMask;

//		testMask = testMask << 3
//		testMask = -testMask;
//		testShift = testShift & testMask;

	}
}
