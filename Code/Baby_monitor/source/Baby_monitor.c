/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"

#include "Driver_I2C.h"
#include "fsl_i2c_cmsis.h"

#include "fsl_lpsci_cmsis.h"
#include "Driver_USART.h"

#include "fsl_lptmr.h"

/* My Code includes */
#include "max30100/max30100.h"
#include "beat_detector.h"
#include "init.h"
#include "spo2.h"
#include "vars.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/



/*		DEBUG Defines	*/
/*TODO export defines below to file with global variables	*/
// #define MAX30100_DEBUG
//#define MAX30100_FIFO_RAW_OUTPUT
//#define MAX30100_FILTERED_RAW_OUTPUT
//#define MAX30100_DC_RAW_OUTPUT
// #define MAX30100_BEAT_DETECTOR_OUTPUT

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
bool processData(void);
void initVariableToProcess();
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*	TODO move the variables bellow to correct place	*/
bool isValidSample = false;
struct fifo_t sample;
struct dcFilter_t acFilterIR;
struct dcFilter_t acFilterRed;
struct meanDiffFilter_t meanDiffIR;
struct butterworthFilter_t filter;
struct simpleBeatDetector_t beat_detector_t;

bool isBeatDetected = false;
uint16_t beat_result = 0;
char beat_text[50] = { 0 };
uint16_t bmp_v[BPM_VECTOR_SIZE] = { 0 };
bool isConfiableOutput = false;
uint16_t bpm_avg = 0;

uint8_t spo2 = 0;
bool isSpo2Ready = false;

bool canCalculateSpo2 = true;
bool canCalculateBPM = true;

/*		LPTMR Variables		*/
uint32_t currentCounter = 0U;
lptmr_config_t lptmrConfig;

volatile uint32_t lptmrCounter = 0;
volatile uint32_t lptmrCounter2 = 0U;

bool canBlinkGreenLed = true;
bool finishedRead = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SysTick_Handler(void)
{                              	  /* SysTick interrupt Handler. */
	msTicks++;                                                /* See startup file startup_LPC17xx.s for SysTick vector */
}


void LPTMR_LED_HANDLER(void)
{
	LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);
	//millis_tick++;		/*	Counter to use in millis function	*/
	lptmrCounter++;		/*	Counter to timer of read fifo	*/
	lptmrCounter2++;	/*	Counter to timer of led current adjustment */

	__DSB();
	__ISB();
}


void initVariableToProcess(void)
{
	/*	Clean up structs to start filter process	*/
	dcFilterClear(&acFilterIR);
	dcFilterClear(&acFilterRed);
	meanDiffFilterClear(&meanDiffIR);
	initSimpleBeatDetector(&beat_detector_t);
	beat_detector_t.state = SIMPLE_BEAT_DETECTOR_WAITING_STABLE;
	init_tick();

}

bool processData(void)
{
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

#ifdef MAX30100_FILTERED_RAW_OUTPUT
			sprintf(value, "%d\t", (int)filter.result);
			USART_Printf(value);
			sprintf(value, "%d\r\n", (int)acFilterRed.result);
			USART_Printf(value);
#endif

#ifndef MAX30100_FIFO_RAW_OUTPUT
			/*	Beat Detector Algorithm	*/
			isBeatDetected = checkForSimpleBeat(filter.result, &beat_detector_t, &beat_result);
			if (isBeatDetected)
			{
				canBlinkGreenLed = false;
				BEAT_LED(); /*	indicate beat signal using red led	*/
				if (canCalculateSpo2)
				{
					isSpo2Ready = spo2Calculator(acFilterIR.result, acFilterRed.result, isBeatDetected, &spo2);
					if(isSpo2Ready)
						canCalculateSpo2 = false;
				}
				//					if (isSpo2Ready)
				//					{
				//						sprintf(beat_text, "spo2 = %d\r\n", spo2);
				//						USART_Printf(beat_text);
				//						isSpo2Ready = false;
				//					}

				isBeatDetected = false;
				if (canCalculateBPM)
				{
					isConfiableOutput = bpmAvgCalculator(beat_result, bmp_v, &bpm_avg);
					if (isConfiableOutput)
						canCalculateBPM = false;
				}

				if (!canCalculateSpo2 && !canCalculateBPM)
				{
					isConfiableOutput = false;
					isSpo2Ready = false;
					canCalculateBPM = true;
					canCalculateSpo2 = true;
//					sprintf(beat_text, "bpm_avg = %d \t sp02 = %d \r\n", bpm_avg, spo2);
					sprintf(beat_text, "%d / %d", spo2, bpm_avg);
					USART_Printf(beat_text);

					return true;
				}

				//					sprintf(beat_text, "beat_result = %d\r\n", beat_result);
				//					USART_Printf(beat_text);
			}
#endif
		}
	}
#ifdef MAX30100_FIFO_RAW_OUTPUT
	canPrint = readFIFO(&irRaw, &redRaw);
#endif

	return false;
}

int main(void)

{
	init_tick();

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_I2C_ReleaseBus();
	BOARD_InitDebugConsole();

	/* Init output LED GPIO. */
	init_gpio_pins();	/*	gpio init pins	*/

	/*			USART Config 					*/
	init_usart();

	/*	LPTMR Init	*/
	config_lptmr();

#ifdef MAX30100_DEBUG
	/*	TODO Separate #define for USART and one to USB	*/
	//USART_Printf("Application Starting\r\n");
	PRINTF("Application Starting\r\n");
#endif

	I2C_Init();

	/*	MAX30100 initialization	*/
	MAX30100_Init();

	initVariableToProcess();

	for(;;)
	{
		/*	TODO Verify if lptmrCounter and lptmrCounter2 can be short type	*/
		if (currentCounter != lptmrCounter)	/* lptmrCounter change when 10 ms pass */
		{
			currentCounter = lptmrCounter;
			if (lptmrCounter > 99)
			{
				lptmrCounter = 0;
#ifdef MAX30100_FIFO_RAW_OUTPUT
				balanceIntesities(redRaw, irRaw);
#endif
				/*Indicate life of system using Green Led	*/
				if (canBlinkGreenLed)
				{
					LED_GREEN_TOGGLE();
				}
				else
				{
					LED_GREEN_OFF();
				}
			}
			/*	Process new data at every ten seconds (10 ms * 10.000 = 10 s)	*/
			if (lptmrCounter2 > 1000)
			{
				BLE_ON();
				finishedRead = processData();

				if (finishedRead)
				{
					clearMillis();
					while(millis() < 3000);
					BLE_OFF();
					lptmrCounter2 = 0;
					BLINK_BLUE();
					delay();
					BLINK_BLUE();
					delay();
					BLINK_BLUE();
					initVariableToProcess();
					finishedRead = false;
					canBlinkGreenLed = true;
				}

			}
		}
	}
}
