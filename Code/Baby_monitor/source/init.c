/*
 * init.c
 *
 *  Created on: 2 de mai de 2019
 *      Author: felipe
 */

#include "init.h"

volatile uint32_t msTicks = 0;                              /* Variable to store millisecond ticks */

void clearMillis(void)
{
	/*
	 * @brief  clearMillis
	 * @details     Function to clear variable that save time passed in milliseconds
	 * @param[in] void
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 2 de mai de 2019
	 *
	 */
	msTicks = 0;

}

uint32_t millis(void)
{
	/*
	 * @brief  millis
	 * @details  function to return the time passed in milliseconds
	 * @param[in] void
	 * @param[out]
	 * @return volatile unsigned long time passed in milliseconds
	 *
	 * author dell-felipe
	 * date 2 de mai de 2019
	 *
	 */
	return msTicks;
}


void init_tick(void)
{
	SysTick_Config(SystemCoreClock / 1000);
}

uint32_t UART0_GetFreq(void)
{
	return CLOCK_GetFreq(kCLOCK_CoreSysClk);
}

void USART_SignalEvent_t(uint32_t event)
{
	if (ARM_USART_EVENT_SEND_COMPLETE == event)
	{
		txOnGoing = false;
	}

}

void USART_Printf(const char* string)
{
	APP_USART.Send(string, strlen(string));
	while (txOnGoing)
	{

	}

	txOnGoing = 1;
}
