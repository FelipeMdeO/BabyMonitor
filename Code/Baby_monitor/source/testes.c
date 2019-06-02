/*
 * testes.c
 *
 *  Created on: 31 de mai de 2019
 *      Author: dell-felipe
 */


#include "testes.h"

char beat_text[50] = { 0 };
static bool isInitilizedTestes = false;
static bool canRead = false;

/* Rotinas de com funcoes individuais para facilitar testes do produto*/

void executeTestes(void) {

	if (!isInitilizedTestes)
		initTestes();

	/*	Selecione aqui os testes a serem realizados	*/
	continuosReadToBLE();
	//	continuosReadToSerial();
	//		oneShootByButtonToBLE();
	//	oneShootByButtonToSerial();
	//	cointinuosReadToSerialIRLed();

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

	sprintf(beat_text, "%d / %d", spo2, bpm_avg);
	USART_Printf(beat_text);
	canRead = false;
}

static void continuosRead(void) {

	uint8_t spo2 = 0;
	uint16_t bpm_avg = 0;

	initVariableToProcess();
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

void cointinuosReadToSerialIRLed(void) {
	/*
	 * @brief  cointinuosReadToSerialIRLed
	 * @details     Function to read data from led infra red and output it to serial reader
	 * The output is sensor data filtered but
	 * @param[in] void
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 2 de jun de 2019
	 *
	 */
	// TODO Test this function!
	char value[20] = { 0 };

	initVariableToProcess();
	bool isValidSample = false;
	struct fifo_t sample;
	struct dcFilter_t acFilterIR;
	struct dcFilter_t acFilterRed;
	struct meanDiffFilter_t meanDiffIR;
	struct butterworthFilter_t filter;

	while(1) {
		isValidSample = MAX30100_Get_Sample(&sample.rawIR, &sample.rawRed);
		if( isValidSample )
		{
			acFilterIR = dcRemoval((float)sample.rawIR, acFilterIR.w, ALPHA);
			acFilterRed = dcRemoval((float)sample.rawRed, acFilterRed.w, ALPHA);
			float meanDiffResIR = meanDiff(acFilterIR.result, &meanDiffIR);
			/*	IF mean vector was fully filed	*/
			if (meanDiffIR.count >= MEAN_FILTER_SIZE)
			{
				/*	toggle a pin here if you want test loop performance	*/
				/*	GPIO_PortToggle(GPIOB, 1u << 8U);	*/

				/*	low pass filter implementation	*/
				lowPassFilter(meanDiffResIR, &filter);
				sprintf(value, "%d\t\n", (int)filter.result);
				USART_Printf(value);
			}
		}
	}
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
			initVariableToProcess();
			continuosRead();

		}

		/* leitura continua dos dados com um delay de 3 segundos e escrita via bluetooth		*/
		void continuosReadToBLE(void) {
			BLE_ON();
			clearMillis();
			LED_RED_ON();
			while(millis() < TIME_DELAY_SEC * 1000); /*	delay by 3 second	*/
			LED_RED_OFF();
			initVariableToProcess();
			continuosRead();
		}



		/*	Teste para ajuste dos parametros dos leds	*/
		//todo - implementar

		/*	Rotinas com testes de consumo	*/
		//todo - implementar
