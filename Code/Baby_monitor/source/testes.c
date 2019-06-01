/*
 * testes.c
 *
 *  Created on: 31 de mai de 2019
 *      Author: dell-felipe
 */


#include "testes.h"

char beat_text[50] = { 0 };
static bool isInitilizedTestes = false;

/* Rotinas de com funcoes individuais para facilitar testes do produto*/

void executeTestes(void) {

	if (!isInitilizedTestes)
		initTestes();

	/*	Selecione aqui os testes a serem realizados	*/
	continuosReadToBLE();
//	continuosReadToSerial();
//	oneShootByButtonToBLE();
//	oneShootByButtonToSerial();
//	executeTestes();
}


/*******************************************************************************
 * Static Functions
 ******************************************************************************/
static void oneShootByButton(void) {

}

static void continuosRead(void) {

	uint8_t spo2 = 0;
	uint16_t bpm_avg = 0;
	while(!processData(&spo2, &bpm_avg));

	sprintf(beat_text, "%d / %d", spo2, bpm_avg);
	USART_Printf(beat_text);

}


/*******************************************************************************
 * Public Functions
 ******************************************************************************/
// TODO como chamar isso daqui automaticamente ao realizar testes?
void initTestes(void) {
	isInitilizedTestes = true;
	ALL_LED_OFF();
}

/* leitura do dado a partir de um botao com escrita pela serial	*/
void oneShootByButtonToSerial(void) {

}

/* leitura do dado a partir de um botao com escrita por bluetooth	*/
void oneShootByButtonToBLE(void) {

}

/* leitura continua dos dados com um delay de X segundos e escrita via serial	*/
void continuosReadToSerial(void) {

	clearMillis();
	LED_RED_ON();
	while(millis() < TIME_DELAY_SEC * 1000); /*	delay by 3 second	*/
	LED_BLUE_OFF();
	continuosRead();

}

/* leitura continua dos dados com um delay de X segundos e escrita via bluetooth		*/
void continuosReadToBLE(void) {
	BLE_ON();
	continuosReadToSerial();
}



/*	Teste para ajuste dos parametros dos leds	*/
//todo - implementar

/*	Rotinas com testes de consumo	*/
//todo - implementar
