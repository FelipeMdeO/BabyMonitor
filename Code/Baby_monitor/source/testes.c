/*
 * testes.c
 *
 *  Created on: 31 de mai de 2019
 *      Author: dell-felipe
 */


#include "testes.h"

static bool isInitilizedTestes = false;
static bool canRead = false;

char value[50] = { 0 };
char newLine[] = "\n";

/* Rotinas de com funcoes individuais para facilitar testes do produto*/

void executeTestes(void) {

	if (!isInitilizedTestes)
		initTestes();

	/*	Selecione aqui os testes a serem realizados	*/

	//	continuosReadToBLE();
	//	continuosReadToSerial();
	//	oneShootByButtonToBLE();
	//	oneShootByButtonToSerial();
	//		continuosReadToSerialIRLed();
	//	rCalibration();
	max30102Start();

	if(!canRead) {
		LED_GREEN_ON();
		canBlinkGreenLed = false;
	}
	else LED_GREEN_OFF();
}


/*******************************************************************************
 * Static Functions
 ******************************************************************************/
static void oneShootByButton(void) {

	LED_GREEN_OFF();
	uint8_t spo2 = 0;
	uint16_t bpm_avg = 0;
	while(!processData(&spo2, &bpm_avg));

	sprintf(value, "%d / %d", spo2, bpm_avg);
	USART_Printf(value);
	canRead = false;
}

static void continuosRead(void) {

	uint8_t spo2 = 0;
	uint16_t bpm_avg = 0;

	initVariableToProcess();
	while(!processData(&spo2, &bpm_avg));

	sprintf(value, "%d / %d", spo2, bpm_avg);
	USART_Printf(value);

}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/
// TODO como chamar isso daqui automaticamente ao realizar testes?
void initTestes(void) {
	isInitilizedTestes = true;
	ALL_LED_OFF();
}

void max30102Start(void)
{
	uint8_t id = 0;
	volatile bool result = false;
	result = MAX30102_Read_Part_Id(&id);

	if (result)
		printf("Sensor Encontrado\n");
	else
		printf("Sensor N encontrado\n");

	(void)MAX30102_Init();
}

void continuosReadToSerialIRLed(void) {
	/*
	 * @brief  continuosReadToSerialIRLed
	 * @details     Function to read data from led infra red and output it to serial reader
	 * The output is sensor data filtered
	 * @param[in] void
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 2 de jun de 2019
	 *
	 */

	float irValue = 0;

	while(!processIRData(&irValue));
    signed long int aux = (signed long int) (irValue*1000000);
	sprintf(value, "%d", aux);
	USART_Printf(value);
	USART_Printf(newLine);

}
/* leitura do dado a partir de um botao com escrita pela serial	*/
void oneShootByButtonToSerial(void) {
	if(BUTTON_VALUE()) {
		canRead = true;
		initVariableToProcess();
	}
	if (canRead) oneShootByButton();
}

/* leitura do dado a partir de um botao com escrita por bluetooth	*/
void oneShootByButtonToBLE(void) {
	BLE_ON();
	if(BUTTON_VALUE()) {
		canRead = true;
		initVariableToProcess();
	}
	if (canRead) oneShootByButton();
}

/* leitura continua dos dados com um delay de 3 segundos e escrita via serial	*/
void continuosReadToSerial(void) {

	clearMillis();
	LED_RED_ON();
	while(millis() < TIME_DELAY_SEC * 1000); /*	delay by 3 second	*/
	LED_RED_OFF();
	continuosRead();
	USART_Printf(newLine);
}

/* leitura continua dos dados com um delay de 3 segundos e escrita via bluetooth		*/
void continuosReadToBLE(void) {
	BLE_ON();
	clearMillis();
	BLINK_BLUE();
	while(millis() < TIME_DELAY_SEC * 1000); /*	delay by 3 second	*/
	continuosRead();
}

void rCalibration(void) {

	volatile float R = 0;
	LED_RED_ON();
	clearMillis();
	while(millis() < 1000); /*	wait for 1 second	*/
	LED_RED_OFF();
	while(!processR(&R));
	uint32_t aux = (uint32_t) (R*1000000);
	sprintf(value, "%d", aux);
	USART_Printf(value);
	USART_Printf(newLine);

}

/*	Teste para ajuste dos parametros dos leds	*/
//todo - implementar

/*	Rotinas com testes de consumo	*/
//todo - implementar
