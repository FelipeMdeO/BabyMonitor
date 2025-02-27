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

/*			LPTMR Definitions					*/
#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_LED_HANDLER LPTMR0_IRQHandler
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT 10000U /* 10 ms */

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


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*		LPTMR Variables		*/
uint32_t currentCounter = 0U;
lptmr_config_t lptmrConfig;

volatile uint32_t lptmrCounter = 0;
volatile uint32_t lptmrCounter2 = 0U;
//volatile uint32_t msTicks = 0;
/*	variable to count time in milliseconds passed
 *	NOTE
 *	volatile unsigned long can assume 0 to 4,294,967,295
 *	maybe it can be reduced to unsigned int					*/

bool canBlinkGreenLed = true;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SysTick_Handler(void)  {                              	  /* SysTick interrupt Handler. */
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

	/* Configure LPTMR */
	/*
	 * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
	 * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
	 * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
	 * lptmrConfig.enableFreeRunning = false;
	 * lptmrConfig.bypassPrescaler = true;
	 * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
	 * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
	 */

	LPTMR_GetDefaultConfig(&lptmrConfig);

	/* Initialize the LPTMR */
	LPTMR_Init(DEMO_LPTMR_BASE, &lptmrConfig);

	/*
	 * Set timer period.
	 * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
	 */
	LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

	/* Enable timer interrupt */
	LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(DEMO_LPTMR_IRQn);

	/* Start counting */
	LPTMR_StartTimer(DEMO_LPTMR_BASE);

	txOnGoing = true;

#ifdef MAX30100_DEBUG
	/*	TODO Separate define for USART and one to USB	*/
	//USART_Printf("Application Starting\r\n");
	PRINTF("Application Starting\r\n");
#endif

	I2C_Init();

	/*	MAX30100 initialization	*/
	MAX30100_Init();

#ifdef MAX30100_DEBUG
	PRINTF("\r\nEnd of I2C MAX30100 test .\r\n");
#endif

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

#ifdef MAX30100_FIFO_RAW_OUTPUT
	uint16_t irRaw, redRaw;
	bool canPrint = false;
#endif

#ifdef MAX30100_FILTERED_RAW_OUTPUT
	char value[50] = { 0 };
#endif

	/*	Clean up structs to start filter process	*/
	dcFilterClear(&acFilterIR);
	dcFilterClear(&acFilterRed);
	meanDiffFilterClear(&meanDiffIR);
	initSimpleBeatDetector(&beat_detector_t);
	beat_detector_t.state = SIMPLE_BEAT_DETECTOR_WAITING_STABLE;
	init_tick();

	for(;;)
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
					isSpo2Ready = spo2Calculator(acFilterIR.result, acFilterRed.result, isBeatDetected, &spo2);
					if (isSpo2Ready)
					{
						sprintf(beat_text, "spo2 = %d\r\n", spo2);
						USART_Printf(beat_text);
						isSpo2Ready = false;
					}

					isBeatDetected = false;
					isConfiableOutput = bpmAvgCalculator(beat_result, bmp_v, &bpm_avg);

					if (isConfiableOutput)
					{
						isConfiableOutput = false;

						sprintf(beat_text, "bpm_avg = %d\r\n", bpm_avg);
						USART_Printf(beat_text);
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
				//				LED_TOGGLE();	/*Indicate life of system using Green Led	*/
				if (canBlinkGreenLed)
				{
					LED_GREEN_TOGGLE();
				}
				else
				{
					LED_GREEN_OFF();
				}
			}
			/*	Adjust Red Led current balancing with 500 ms (10 ms * 50 = 500 ms)	*/
			if (lptmrCounter2 > 49)
			{
				lptmrCounter2 = 0;
				canAdjustRedCurrent = true;
				/*	Implement refresh led adjustment	*/

			}
		}


#ifdef MAX30100_DEBUG
		PRINTF("---------------------------------------------\r\n");
#endif
	}

}
