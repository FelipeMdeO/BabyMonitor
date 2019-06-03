/*
 * testes.h
 *
 *  Created on: 31 de mai de 2019
 *      Author: dell-felipe
 */

#ifndef TESTES_H_
#define TESTES_H_

#include "process.h"
#include "init.h"


#define TIME_DELAY_SEC 0

/* TODO working in progess*/
void initTestes(void);	/*	todo Implementar isso daqui melhor	*/
void oneShootByButtonToSerial(void);
void oneShootByButtonToBLE(void);
void continuosReadToBLE(void);
void continuosReadToSerial(void);
void continuosReadToSerialIRLed(void);
void adjustLedParameters(void);
void executeTestes(void);
void rCalibration(void);


#endif /* TESTES_H_ */
