/*
 * init.h
 *
 *  Created on: 2 de mai de 2019
 *      Author: felipe
 */

#ifndef INIT_H_
#define INIT_H_


#include <fsl_debug_console.h>
#include <Driver_USART.h>
#include <fsl_lpsci_cmsis.h>

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

#include "Driver_I2C.h"
#include "fsl_i2c_cmsis.h"

#include "fsl_lptmr.h"

#include "board.h"

#include "vars.h"

/*								USART Definitions 														*/
#define APP_USART Driver_USART0
#define ECHO_BUFFER_LENGTH 8

#define LED_TOGGLE() LED_RED_TOGGLE()

/* I2C Definitions */
#define I2C_MASTER Driver_I2C0
#define I2C_RELEASE_SDA_PORT PORTB
#define I2C_RELEASE_SCL_PORT PORTB
#define I2C_RELEASE_SDA_GPIO GPIOB
#define I2C_RELEASE_SDA_PIN 4U
#define I2C_RELEASE_SCL_GPIO GPIOB
#define I2C_RELEASE_SCL_PIN 3U
#define I2C_RELEASE_BUS_COUNT 100U

/*			LPTMR Definitions					*/
#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_LED_HANDLER LPTMR0_IRQHandler
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT 10000U /* 10 ms */

/*	I2C Prototypes	*/
void BOARD_I2C_ReleaseBus(void);
bool I2C_WriteReg(uint8_t device_addr, uint8_t reg_addr, uint8_t value);
bool I2C_ReadRegs(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
uint32_t I2C0_GetFreq(void);
void I2C_MasterSignalEvent_t(uint32_t event);
void I2C_Init(void);

/*	I2C Extern Variables	*/
extern volatile bool completionFlag;
extern volatile bool nakFlag;

extern volatile uint32_t msTicks;

/*								USART Variables 														*/
extern uint8_t g_txBuffer[ECHO_BUFFER_LENGTH]; 		/*	Data buffer 1 byte								*/
extern volatile bool txOnGoing; 					/*	Variable to identify if data finished transfer	*/

/*	USART CONFIG	*/
void init_usart(void);

/*	LPTMR CONFIG and VARIABLES*/
void init_tick(void);
uint32_t millis(void);
void clearMillis(void);
void config_lptmr(void);

extern lptmr_config_t lptmrConfig;
void delay(void);

void init_gpio_pins(void);	/*	Function to init gpio pins	*/

/*								USART user Signal Event 												*/
void USART_SignalEvent_t(uint32_t event);
void USART_Printf(const char* string);

void BEAT_LED(void);
void BLINK_BLUE(void);
void BLE_ON(void);
void BLE_OFF(void);
void ALL_LED_ON(void);
void ALL_LED_OFF(void);
#endif /* INIT_H_ */
