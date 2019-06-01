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
#include "process.h"
#include "testes.h"

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

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*		LPTMR Variables		*/
uint32_t currentCounter = 0U;
lptmr_config_t lptmrConfig;

volatile uint32_t lptmrCounter = 0;
volatile uint32_t lptmrCounter2 = 0U;

bool canBlinkGreenLed = true;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SysTick_Handler(void)
{                              	  /* SysTick interrupt Handler. */
	msTicks++;                   /* See startup file startup_LPC17xx.s for SysTick vector */
}


void LPTMR_LED_HANDLER(void)
{
	LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);
	//millis_tick++;	/*	Counter to use in millis function	*/
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
		executeTestes();
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

			}
		}
	}
}
