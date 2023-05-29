/*
 *************************************************************************
 * @file		: TestingAbs_Main.c
 * @author		: Alejandro Rodríguez Montes - alerodriguezmo@unal.edu.co
 * @brief		: Código ejemplo para probrar la librería DSP
 *
 *************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "ExtiDriver.h"
#include "PLLDriver.h"
#include "SysTickDriver.h"
#include "PwmDriver.h"
#include "I2CDriver.h"

/*Definición de macros para acelerometro*/
#define ACCEL_ADDRESS          	 0x1D
#define ACCEL_XOUT_L             50
#define ACCEL_XOUT_H             51
#define ACCEL_YOUT_L             52
#define ACCEL_YOUT_H             53
#define ACCEL_ZOUT_L             54
#define ACCEL_ZOUT_H             55

#define BW_RATE                  44

#define PWR_MGMT_1                45
#define WHO_AM_I                 0


//Definicion de los handlers necesarios
//Handlers para el led de estado
GPIO_Handler_t handlerBlinkyPin = {0};
BasicTimer_Handler_t handlerBlinkyTimer= {0};
//Handler pll- MCO que habilita que se pueda estudiar la señal de reloj
PLL_Config_t handlerPLL = {0};
GPIO_Handler_t handlerMCO = {0};

//Config comunicación USART_1
GPIO_Handler_t handlerPinTX = {0};
GPIO_Handler_t handlerPinRX = {0};
USART_Handler_t usart1Comm = {0};
char rxData = '\0';
char bufferData[64] = "Accel MPU-6050 testing...";


////Variables globales para no tener que declararlas siempre en cuanto a coordenadas
int16_t coordinates[3];//Array donde se van a actualizar los datos cada mS de las coordenadas
int16_t coordinatex[2000];//Array donde se van a guardar los datos para x
int16_t coordinatey[2000];//Array donde se van a guardar los datos para y
int16_t coordinatez[2000];//Array donde se van a guardar los datos para z
uint8_t cont2eimprimir;

uint8_t coordinatesbool = 0; //V. Auxiliar que es una flag para calcular las coordenadas

BasicTimer_Handler_t handlerMuestreo = {0};
int cont2i = 0;
uint8_t cont2e;
uint8_t cont2ee;
/*    Configración para el I2C    */
GPIO_Handler_t handlerI2CSDA = {0};
GPIO_Handler_t handlerI2CSCL = {0};

I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;

uint8_t sendMsg = 0;
/* Configuración para el PWM*/
PWM_Handler_t handlerPwmX = {0};
GPIO_Handler_t handlerPinPwmX = {0};
PWM_Handler_t handlerPwmY = {0};
GPIO_Handler_t handlerPinPwmY = {0};
PWM_Handler_t handlerPwmZ = {0};
GPIO_Handler_t handlerPinPwmZ = {0};
uint8_t setPwmf= 0;


void initSystem(void);
void getCoordinates();
void rxHandlerFunction();
void printXYZ();

int main(void){
	SCB->CPACR |= (0xF<<20);
    initSystem();
    while(1){
    	if(rxData != '\0'){
    		rxHandlerFunction();//es el que redirige los posibles casos en la recepcion
        }
        if(coordinatesbool){//cada mS va a calcular las aceleraciones
            getCoordinates();
        }
        if(cont2eimprimir){
            for(int contador;contador<2000;contador++){
                sprintf(bufferData,"%f;%f;%f dato #%d \n",coordinatex[contador]*2*9.78/32767,coordinatey[contador]*2*9.78/32767,coordinatez[contador]*2*9.78/32767,contador);
                writeMsg(&usart1Comm,bufferData);
            }
            cont2eimprimir = 0;
        }

    }
}
void rxHandlerFunction(){


    if(rxData == 'w'){
        sprintf(bufferData, "WHO AM I? (r)\n");
        writeMsg(&usart1Comm, bufferData);
        i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
        sprintf(bufferData, "dataRead = 0x%x \n",(unsigned int) i2cBuffer);
        writeMsg(&usart1Comm,bufferData);
        rxData = '\0';
    }
    else if(rxData == 'p'){
        sprintf(bufferData, "PWR_MGMT_1 state (r)\n");
        writeMsg(&usart1Comm, bufferData);
        i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, PWR_MGMT_1);
        sprintf(bufferData,"dataRead = 0x%x \n", (unsigned int) i2cBuffer);
        writeMsg(&usart1Comm, bufferData);
        rxData = '\0';
        }
        else if(rxData == 'r'){
            sprintf(bufferData, "PWR_MGMT_1 reset (w)\n");
            writeMsg(&usart1Comm, bufferData);

            i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
            rxData = '\0';
    }

    //Se hace la recepción por interrupción. punto 9
        else if(rxData == 'x'){
            sprintf(bufferData, "Axis X data (r)\n");
            writeMsg(&usart1Comm, bufferData);

            sprintf(bufferData, "AccelX = %d \n", (int) coordinates[1]);
            writeMessageTXE(&usart1Comm, bufferData);
            rxData = '\0';
            }
        else if(rxData == 'y'){
            sprintf(bufferData, "Axis Y data (r)\n");
            writeMessageTXE(&usart1Comm, bufferData);

            sprintf(bufferData, "AccelY = %d \n", (int) coordinates[1]);
            writeMessageTXE(&usart1Comm, bufferData);
            rxData = '\0';
            }
        else if(rxData == 'z'){
            sprintf(bufferData, "Axis Z data (r)\n");
            writeMessageTXE(&usart1Comm, bufferData);

            sprintf(bufferData, "AccelZ = %d \n", (int) coordinates[2]);
            writeMessageTXE(&usart1Comm, bufferData);
            rxData = '\0';
            }
        else if(rxData == 'a'){
            cont2e = 1;
            rxData = '\0';
            }
        else{
            rxData = '\0';
        }
}
void getCoordinates(void){
    uint8_t AccelX_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_L);
    uint8_t AccelX_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_H);
    coordinates[0] = AccelX_high << 8 | AccelX_low;
    uint8_t AccelY_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_L);
    uint8_t AccelY_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_H);
    int16_t AccelY = AccelY_high << 8 | AccelY_low;
    coordinates[1]=AccelY;
    uint8_t AccelZ_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_L);
    uint8_t AccelZ_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_H);
    int16_t AccelZ = AccelZ_high << 8 | AccelZ_low;
    coordinates[2]=AccelZ;
    if(cont2e){
        if(cont2i==0){
        }else{
            coordinatex[cont2i-1] = coordinates[0];
            coordinatey[cont2i-1] = coordinates[1];
            coordinatez[cont2i-1] = coordinates[2];
            }
    }
    coordinatesbool = 0;
}
void initSystem(void){
    // Configuración del LED2 - PA5
    handlerBlinkyPin.pGPIOx                                = GPIOA;
    handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber        = PIN_5;
    handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_OUT;
    handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType        = GPIO_OTYPE_PUSHPULL;
    handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_OSPEED_MEDIUM;
    handlerBlinkyPin.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
    GPIO_Config(&handlerBlinkyPin);

    handlerMCO.pGPIOx                                = GPIOA;
    handlerMCO.GPIO_PinConfig.GPIO_PinNumber        = PIN_8;
    handlerMCO.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_ALTFN;
    handlerMCO.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_OSPEED_MEDIUM;
    handlerMCO.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
    GPIO_Config(&handlerMCO);


    //Configuracion PLL
    handlerPLL.PLL_src = 1;
    handlerPLL.PLL_pllm = 8;
    handlerPLL.PLL_plln = 80;
    handlerPLL.PLL_pllp = 2;
    PLL_Config(&handlerPLL);
    // Cargando la configuración



    //Configuracion del timer del blinky
    handlerBlinkyTimer.ptrTIMx                                = TIM2;
    handlerBlinkyTimer.TIMx_Config.TIMx_mode                = BTIMER_MODE_UP;
    handlerBlinkyTimer.TIMx_Config.TIMx_speed                = 8000;//porque el reloj principal está a 80MHz
    handlerBlinkyTimer.TIMx_Config.TIMx_period                = 2500; // Lanza una interrupción cada 250 ms
    handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable        = 1;

    BasicTimer_Config(&handlerBlinkyTimer);
//    //Configuracion del timer para el muestreo a 1kHz
    handlerMuestreo.ptrTIMx                                = TIM3;
    handlerMuestreo.TIMx_Config.TIMx_mode                = BTIMER_MODE_UP;
    handlerMuestreo.TIMx_Config.TIMx_speed                = 8000;//porque el reloj principal está a 80MHz
    handlerMuestreo.TIMx_Config.TIMx_period                = 10; // Lanza una interrupción cada 1 ms
    handlerMuestreo.TIMx_Config.TIMx_interruptEnable        = 1;

    BasicTimer_Config(&handlerMuestreo);

    //Config. comunicación serial
    handlerPinTX.pGPIOx = GPIOA;
    handlerPinTX.GPIO_PinConfig.GPIO_PinNumber = PIN_9;
    handlerPinTX.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
    GPIO_Config(&handlerPinTX);//Cable blanco

    handlerPinRX.pGPIOx = GPIOA;
    handlerPinRX.GPIO_PinConfig.GPIO_PinNumber = PIN_10;
    handlerPinRX.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
    GPIO_Config(&handlerPinRX);//Cable verde

    usart1Comm.ptrUSARTx = USART1;
    usart1Comm.USART_Config.USART_baudrate = USART_BAUDRATE_115200;
    usart1Comm.USART_Config.USART_datasize = USART_DATASIZE_8BIT;
    usart1Comm.USART_Config.USART_parity = USART_PARITY_NONE;
    usart1Comm.USART_Config.USART_mode = USART_MODE_RXTX;
    usart1Comm.USART_Config.USART_enableIntRX = USART_RX_INTERRUPT_ENABLE;
    usart1Comm.USART_Config.USART_enableIntTX = USART_TX_INTERRUPT_ENABLE;
    usart1Comm.USART_Config.USART_freq = 80;
    USART_Config(&usart1Comm);

    // Configurando los pines sobre los que funciona el I2C1
    handlerI2CSCL.pGPIOx                                = GPIOB;
    handlerI2CSCL.GPIO_PinConfig.GPIO_PinNumber            = PIN_8;
    handlerI2CSCL.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_ALTFN;
    handlerI2CSCL.GPIO_PinConfig.GPIO_PinOPType            = GPIO_OTYPE_OPENDRAIN;
    handlerI2CSCL.GPIO_PinConfig.GPIO_PinSpeed            = GPIO_OSPEED_FAST;
    handlerI2CSCL.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
    handlerI2CSCL.GPIO_PinConfig.GPIO_PinAltFunMode     = AF4;

    handlerI2CSDA.pGPIOx                                = GPIOB;
    handlerI2CSDA.GPIO_PinConfig.GPIO_PinNumber            = PIN_9;
    handlerI2CSDA.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_ALTFN;
    handlerI2CSDA.GPIO_PinConfig.GPIO_PinOPType            = GPIO_OTYPE_OPENDRAIN;
    handlerI2CSDA.GPIO_PinConfig.GPIO_PinSpeed            = GPIO_OSPEED_FAST;
    handlerI2CSDA.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
    handlerI2CSDA.GPIO_PinConfig.GPIO_PinAltFunMode     = AF4;

    // Cargamos las configuraciones
    GPIO_Config(&handlerI2CSCL);
    GPIO_Config(&handlerI2CSDA);

    // Configuramos el protocolo I2C y cargamos dicha configuración
    handlerAccelerometer.ptrI2Cx            = I2C1;
    handlerAccelerometer.modeI2C            = I2C_MODE_FM;
    handlerAccelerometer.slaveAddress        = ACCEL_ADDRESS;
    handlerAccelerometer.freq                 = 40;

    i2c_config(&handlerAccelerometer);

    //Se configura el PWM en el timer 5
    handlerPwmX.ptrTIMx = TIM5;
    handlerPwmX.config.channel = 1;
    handlerPwmX.config.prescaler = 16;
    handlerPwmX.config.periodo = 65534;
    handlerPwmX.config.duttyCicle = 32767;
    handlerPinPwmX.pGPIOx                                = GPIOA;
    handlerPinPwmX.GPIO_PinConfig.GPIO_PinNumber            = PIN_0;
    handlerPinPwmX.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_ALTFN;
    handlerPinPwmX.GPIO_PinConfig.GPIO_PinOPType            = GPIO_OTYPE_OPENDRAIN;
    handlerPinPwmX.GPIO_PinConfig.GPIO_PinSpeed            = GPIO_OSPEED_FAST;
    handlerPinPwmX.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
    handlerPinPwmX.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;
    GPIO_Config(&handlerPinPwmX);
    pwm_Config(&handlerPwmX);
    handlerPinPwmY.pGPIOx                                = GPIOA;
    handlerPinPwmY.GPIO_PinConfig.GPIO_PinNumber            = PIN_1;
    handlerPinPwmY.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_ALTFN;
    handlerPinPwmY.GPIO_PinConfig.GPIO_PinOPType            = GPIO_OTYPE_OPENDRAIN;
    handlerPinPwmY.GPIO_PinConfig.GPIO_PinSpeed            = GPIO_OSPEED_FAST;
    handlerPinPwmY.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
    handlerPinPwmY.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;
    handlerPwmY.ptrTIMx = TIM5;
    handlerPwmY.config.channel = 2;
    handlerPwmY.config.prescaler = 16;
    handlerPwmY.config.periodo = 65534;
    handlerPwmY.config.duttyCicle = 32767;
    GPIO_Config(&handlerPinPwmY);
    pwm_Config(&handlerPwmY);
    handlerPwmZ.ptrTIMx = TIM5;
    handlerPwmZ.config.channel = 3;
    handlerPwmZ.config.prescaler = 16;
    handlerPwmZ.config.periodo = 65534;
    handlerPwmZ.config.duttyCicle = 32767;
    handlerPinPwmZ.pGPIOx                                = GPIOA;
    handlerPinPwmZ.GPIO_PinConfig.GPIO_PinNumber            = PIN_2;
    handlerPinPwmZ.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_ALTFN;
    handlerPinPwmZ.GPIO_PinConfig.GPIO_PinOPType            = GPIO_OTYPE_OPENDRAIN;
    handlerPinPwmZ.GPIO_PinConfig.GPIO_PinSpeed            = GPIO_OSPEED_FAST;
    handlerPinPwmZ.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
    handlerPinPwmZ.GPIO_PinConfig.GPIO_PinAltFunMode     = AF2;
    GPIO_Config(&handlerPinPwmZ);
    pwm_Config(&handlerPwmZ);

}
void BasicTimer2_Callback(){ // Callback de timer 2 se hace blinky
    GPIOxTooglePin(&handlerBlinkyPin);
    sendMsg++;
}
void BasicTimer3_Callback(){
    // Callback de timer 3 se activa "flag" para que en el main se calculen las coordenadas y no se tenga tanto tiempo
    // el micro en interrupt, se manda cada 1 ms
    coordinatesbool = 1;//Se activa la flag cada mS de que se quiere leer el acelerometro
    if(cont2i == 2001){//Si el contador de iteraciones de 2 segundos se pasa de las 2000 iteraciones
        cont2e = 0;//baje la bandera de contar 2seg
        cont2i = 0;//Reinicie las iteraciones de contar 2 seg
        cont2eimprimir = 1;//Active el flag que le permite imprimir toda la lista.
    }
    if(cont2e){//Si el flag de contar 2 seg está activado
        cont2i++;//Aumente en 1 el numero de iteraciones de los 2 segundos
    }
}
void BasicTimer5_Callback(){
   setPwmf = 1;
}
void callback_extInt13(void){
    __NOP();
}

void usart1Rx_Callback(void){
    rxData = getRxData();
}
