/*
 * init.h
 *
 *  Created on: 2 de mai de 2019
 *      Author: felipe
 */

#ifndef INIT_H_
#define INIT_H_

//#include <CMSIS/system_MKL02Z4.h>
#include <fsl_debug_console.h>
#include <Driver_USART.h>
#include <fsl_lpsci_cmsis.h>

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

#include "board.h"

/*								USART Definitions 														*/
#define APP_USART Driver_USART0
#define ECHO_BUFFER_LENGTH 8	/*	TODO maybe change it to more bytes	*/

#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()

extern volatile uint32_t msTicks;
/*								USART Variables 														*/
extern uint8_t g_txBuffer[ECHO_BUFFER_LENGTH]; 		/*	Data buffer 1 byte								*/
extern volatile bool txOnGoing; 					/*	Variable to identify if data finished transfer	*/

void init_tick(void);
uint32_t millis(void);
void clearMillis(void);

void init_gpio_pins(void);	/*	Function to init gpio pins	*/

/*								USART user Signal Event 												*/
void USART_SignalEvent_t(uint32_t event);
void USART_Printf(const char* string);

#endif /* INIT_H_ */
