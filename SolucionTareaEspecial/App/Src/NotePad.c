/*
 * NotePad.c
 *
 *  Created on: Jun 4, 2023
 *      Author: ingfisica
 */

if (rxData != '\0'){
	bufferReception[counterReception] = rxData;
	counterReception++;

	// Si el caracter que llega representa un cambio de línea, se levanta una
	// bandera para el loop main
	if (rxData == '@'){

	}
}
