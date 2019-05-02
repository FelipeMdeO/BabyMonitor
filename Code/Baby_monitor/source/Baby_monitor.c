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

//#include "fsl_i2c.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

#include "Driver_I2C.h"
#include "fsl_i2c_cmsis.h"

#include "fsl_lpsci_cmsis.h"
#include "Driver_USART.h"

#include "fsl_lptmr.h"


/* My Code includes */
#include "max30100/max30100.h"
#include "beat_detector.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* I2C Definitions */
#define I2C_MASTER Driver_I2C0
#define I2C_RELEASE_SDA_PORT PORTB
#define I2C_RELEASE_SCL_PORT PORTB
#define I2C_RELEASE_SDA_GPIO GPIOB
#define I2C_RELEASE_SDA_PIN 4U
#define I2C_RELEASE_SCL_GPIO GPIOB
#define I2C_RELEASE_SCL_PIN 3U
#define I2C_RELEASE_BUS_COUNT 100U

/*								USART Definitions 														*/
#define APP_USART Driver_USART0
#define ECHO_BUFFER_LENGTH 8

/*			LPTMR Definitions					*/
#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_LED_HANDLER LPTMR0_IRQHandler
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT 1000U /* 1 ms */
#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()
#define PIN_TOGGLE() PTB8_TOGGLE()

/*		DEBUG Define	*/
//#define MAX30100_DEBUG
#define MAX30100_FIFO_RAW_OUTPUT
//#define MAX30100_FILTERED_RAW_OUTPUT

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

static bool I2C_WriteReg(uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadRegs(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
static bool MAX30100_Get_Sample(uint16_t *IR_sample, uint16_t *Red_sample);
static void MAX30100_Init(void);
static void MAX30100_ClearFIFO(void);
static void USART_Printf(const char* string);
static void setLEDCurrents(uint8_t redLedCurrent, uint8_t IRLedCurrent);
static void setSamplingRate(uint8_t samplingRate);
static void setHighresModeEnabled(bool enabled);
static void balanceIntesities(float redLedDC, float IRLedDC);
volatile unsigned long millis(void);
void clearMillis(void);

/*								USART user Signal Event 												*/
void USART_SignalEvent_t(uint32_t event);

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile bool completionFlag = false;
volatile bool nakFlag = false;
static LEDCurrent redLedCurrent = STARTING_RED_LED_CURRENT;
/* TODO pass IRLedCurrent to static variable too */
static bool canAdjustRedCurrent = false;

/*								USART Variables 														*/
uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = { 0 }; 	/*	Data buffer 1 byte								*/
volatile bool txOnGoing = false; 					/*	Variable to identify if data finished transfer	*/

/*		LPTMR Variables		*/
uint32_t currentCounter = 0U;
lptmr_config_t lptmrConfig;

volatile unsigned int lptmrCounter = 0;
volatile unsigned int lptmrCounter2 = 0U;
/*	variable to count time in milliseconds passed
*	NOTE
*	volatile unsigned long can assume 0 to 4,294,967,295
*	maybe it can be reduced to unsigned int					*/
volatile unsigned long millis_tick = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

void LPTMR_LED_HANDLER(void)
{
	LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);
	millis_tick++;		/*	Counter to use in millis function	*/
	lptmrCounter++;		/*	Counter to timer of read fifo	*/
	lptmrCounter2++;	/*	Counter to timer of led current adjustment */

	__DSB();
	__ISB();
}

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
	millis_tick = 0;

}

volatile unsigned long millis(void)
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
	return millis_tick;
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


static void i2c_release_bus_delay(void)
{
	uint32_t i = 0;
	for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
	{
		__NOP();
	}
}

void BOARD_I2C_ReleaseBus(void)
{
	uint8_t i = 0;
	gpio_pin_config_t pin_config;
	port_pin_config_t i2c_pin_config = {0};

	/* Config pin mux as gpio */
	i2c_pin_config.pullSelect = kPORT_PullUp;
	i2c_pin_config.mux = kPORT_MuxAsGpio;

	pin_config.pinDirection = kGPIO_DigitalOutput;
	pin_config.outputLogic = 1U;
	CLOCK_EnableClock(kCLOCK_PortB);
	PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
	PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

	GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
	GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

	/* Drive SDA low first to simulate a start */
	GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
	i2c_release_bus_delay();

	/* Send 9 pulses on SCL and keep SDA high */
	for (i = 0; i < 9; i++)
	{
		GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
		i2c_release_bus_delay();

		GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
		i2c_release_bus_delay();

		GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
		i2c_release_bus_delay();
		i2c_release_bus_delay();
	}

	/* Send stop */
	GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
	i2c_release_bus_delay();

	GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
	i2c_release_bus_delay();

	GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
	i2c_release_bus_delay();

	GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
	i2c_release_bus_delay();
}

uint32_t I2C0_GetFreq(void)
{
	return CLOCK_GetFreq(I2C0_CLK_SRC);
}

void I2C_MasterSignalEvent_t(uint32_t event)
{
	if (event == ARM_I2C_EVENT_TRANSFER_DONE)
	{
		completionFlag = true;
	}
	if (event == ARM_I2C_EVENT_ADDRESS_NACK)
	{
		nakFlag = true;
	}
}

static bool I2C_WriteReg(uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	uint8_t writedata[2] = {reg_addr, value};

	/*  direction=write : start+device_write;cmdbuff;xBuff; */
	/*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

	I2C_MASTER.MasterTransmit(device_addr, writedata, 2, false);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}

static bool I2C_ReadRegs(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	/*  direction=write : start+device_write;cmdbuff;xBuff; */
	/*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

	I2C_MASTER.MasterTransmit(device_addr, &reg_addr, 1, false);
	while ((!nakFlag) && (!completionFlag))
	{
	}
	nakFlag = false;
	completionFlag = false;
	I2C_MASTER.MasterReceive(device_addr, rxBuff, rxSize, false);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}

static void I2C_Init(void)
{
	I2C_MASTER.Initialize(I2C_MasterSignalEvent_t);
	I2C_MASTER.PowerControl(ARM_POWER_FULL);
	I2C_MASTER.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS);
}

static void Set_mode(uint8_t mode)
{
	/*
	 * author dell-felipe
	 *
	typedef enum Mode {
    	MAX30100_MODE_HR_ONLY                 = 0x02,
    	MAX30100_MODE_SPO2_HR                 = 0x03
	} Mode;
	 *
	 */

	/* First read the default value on register to verify at end if the value changed */
	uint8_t readBuff[1];
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_MODE_CONF, readBuff, 1);

#ifdef MAX30100_DEBUG
	PRINTF("The old value to mode is 0x%x\r\n", readBuff[0]);
#endif

	/* Write wanted value */
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_MODE_CONF, (readBuff[0] & 0xF8) | mode);

	/* Confirm if wanted value was be obtained */
#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_MODE_CONF, readBuff, 1);
	PRINTF("The new value to mode is 0x%x\r\n", readBuff[0]);
#endif

}

static void balanceIntesities(float redLedDC, float IRLedDC)
{
	/*
	 * @brief  balanceIntesities
	 * @details   Function to adjust value of intensity of red led
	 * @param[in] float redLedDC, float IRLedDC
	 * @param[out] float redLedDC, float IRLedDC updated
	 * @return void
	 *
	 * author dell-felipe
	 * date 25 de abr de 2019
	 *
	 */
	if (canAdjustRedCurrent)
	{
		if (redLedCurrent == MAX30100_LED_CURRENT_50MA)
		{
#ifdef MAX30100_DEBUG
			PRINTF("Red Led Current 50 MA\r\n");
#endif
		}
		else if (redLedCurrent == MAX30100_LED_CURRENT_0MA)
		{
#ifdef MAX30100_DEBUG
			PRINTF("Red Led 0 MA\r\n");
#endif
		}

		if (IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLedCurrent < MAX30100_LED_CURRENT_50MA)
		{
			redLedCurrent++;
			setLEDCurrents(redLedCurrent, DEFAULT_IR_LED_CURRENT);
#ifdef MAX30100_DEBUG
			PRINTF("Red Led Current +\r\n");
#endif
		}
		else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLedCurrent > 0)
		{
			redLedCurrent--;
			setLEDCurrents(redLedCurrent, DEFAULT_IR_LED_CURRENT);
		}
#ifdef MAX30100_DEBUG
		PRINTF("Red Led Current -\r\n");
#endif
		canAdjustRedCurrent = false;
	}
}

static void setHighresModeEnabled(bool enabled)
{

	uint8_t readBuff[1] = { 0 } ;
	uint8_t writeValue = 0;

	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);

#ifdef MAX30100_DEBUG
	PRINTF("The old value to SPO2_CONF is 0x%x\r\n = ", readBuff[0]);
#endif
	/* Write wanted value */
	if (enabled)
	{
		writeValue = 1 << 6U;
		writeValue = readBuff[0] | writeValue;
		I2C_WriteReg(MAX30100_DEVICE, MAX30100_SPO2_CONF, writeValue);
	}
	else
	{
		writeValue = 0 << 6U;
		writeValue = readBuff[0] | writeValue;
		I2C_WriteReg(MAX30100_DEVICE, MAX30100_SPO2_CONF, writeValue);
	}


#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);
	PRINTF("The new value to SPO2_CONF is 0x%x\r\n", readBuff[0]);
#endif

}

static void setSamplingRate(uint8_t rate)
{
	uint8_t readBuff[1] = { 0 } ;

	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);
#ifdef MAX30100_DEBUG
	PRINTF("The old value to ledCurrent is 0x%x\r\n", readBuff[0]);
#endif

	/* Write wanted value */
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_SPO2_CONF, ( readBuff[0] & 0xE3 ) | (rate<<2));

#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);
	PRINTF("The new value to ledCurrent is 0x%x\r\n", readBuff[0]);
#endif
}

static void setLEDCurrents( uint8_t redLedCurrent, uint8_t IRLedCurrent )
{
	/*
	 * @brief  setLEDCurrents
	 * @details     This function setup the led current
	 * @param[in] uint8_t, uint8_t
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 7 de abr de 2019
	 *
	 */

	// Shift redLed 4 positions and add in new last 4 positions IRLed bits
	uint8_t ledsCurrent = (redLedCurrent << 4 | IRLedCurrent);
	uint8_t readBuff[1] = { 0 };

	/* First read the default value on register to verify at end if the value changed */
#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_LED_CONF, readBuff, 1);
	PRINTF("The old value to ledCurrent is 0x%x\r\n", readBuff[0]);
#endif

	/* Write wanted value */
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_LED_CONF, ledsCurrent);

	/* Confirm if wanted value was be obtained */
#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_LED_CONF, readBuff, 1);
	PRINTF("The new value to ledCurrent is 0x%x\r\n", readBuff[0]);
#endif

}

static void setLEDPulseWidth(uint8_t pulseWidth)
{
	/*
	 * @brief  setLEDPulseWidth
	 * @details     this function set the led pulse width
	 * @param[in] uint8_t pulsewidth is value in hex that will be select
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 7 de abr de 2019
	 *
	 */

	uint8_t readBuff[1] = { 0 };
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);

#ifdef MAX30100_DEBUG
	PRINTF("The old value to pulseWidth is: 0x%x \r\n", readBuff[0]);
#endif

	/* MAX30100_SPO2_CONF setup to 1600 us  */
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_SPO2_CONF, ( readBuff[0] & 0xFC ) | pulseWidth );

#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);
	PRINTF("The new value to MAX30100_SPO2_CONF is: 0x%x \r\n", readBuff[0]);
#endif
}

static void MAX30100_ClearFIFO(void) {
	/*
	 * @brief  MAX30100_ClearFIFO
	 * @details     It clears the FIFO_WR_PTR, OVF_COUNTER, and FIFO_RD_PTR
	 *             	registers to all zeros (0x00) to ensure the FIFO is empty and in a known state.
	 *             	Resets all points to start in a known state
	 *				Page 15 recommends clearing FIFO before beginning a read
	 * @param[in] void
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 7 de abr de 2019
	 *
	 */

	uint8_t readBuff[3] = { 0 };

	/* Update the FIFO_WR_PTR, FIFO_WR_PTR and OVF_COUNTER registers    */
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_FIFO_WRITE_POINTER, readBuff[0]);
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_FIFO_OVERFLOW_COUNTER, readBuff[0]);
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_FIFO_READ, readBuff[0]);

	/* Get the FIFO_WR_PTR, OVF_COUNTER and FIFO_RD_PTR    */
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_WRITE_POINTER, readBuff, 3);

#ifdef MAX30100_DEBUG
	PRINTF("FIFO cleaned\r\n");
#endif


}

static void MAX30100_Init(void)
{
	/* This function will be initialize register to do max30100 operational
	 * TODO
	 * receive parameters of this function to customize set mode
	 *
	 * add:
	 * 1- set sample rate	- OK
	 * 2- set HighresModeEnabled - Não sei se precisa!
	 */


	Set_mode(MAX30100_MODE_SPO2_HR);
	/* DEFAULT_IR_LED_CURRENT = */
	setLEDCurrents(MAX30100_LED_CURRENT_50MA, MAX30100_LED_CURRENT_50MA);
	//setLEDCurrents(MAX30100_LED_CURRENT_50MA, MAX30100_LED_CURRENT_50MA); /* TODO remember change here*/
	setLEDPulseWidth(DEFAULT_LED_PULSE_WIDTH);
	setSamplingRate(DEFAULT_SAMPLING_RATE);
	setHighresModeEnabled(true);

	MAX30100_ClearFIFO(); /*		TODO Test it more!	*/
}


static bool MAX30100_Get_Sample(uint16_t *IR_sample, uint16_t *Red_sample)
{
	/* TODO complete briefing
	 *
	 * @brief  MAX30100_Get_Sample
	 * @details
	 * @param[in] uint16_t*, uint16_t*
	 * @param[out]
	 * @return bool
	 *
	 * author dell-felipe
	 * date 30 de abr de 2019
	 *
	 */
	uint8_t buffer[4] = { 0 };
	uint8_t readBuff[3] = { 0 };
	uint8_t numberOfSamples = 0;

	/* Read Fifo write pointer */
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_WRITE_POINTER, readBuff, 3);

	numberOfSamples = readBuff[0] - readBuff[2];

	/*	Check for overflow situation	*/
	if (numberOfSamples < 0) {
		numberOfSamples += 16;
	}
	else if ( readBuff[1] == 0xF )
	{
		numberOfSamples = 16;
	}
	/*	Read available samples	*/
	if ( numberOfSamples == 0)
	{
		/*	No samples to read	*/
		return false;
	}
	else if ( numberOfSamples > 0 )
	{
		/*	Read only one sample per time	*/
		I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_DATA, buffer, 4);
		*IR_sample = (buffer[0] << 8) | buffer[1];
		*Red_sample = (buffer[2] << 8) | buffer[3];
		return true;
	}
	return false;
}

static void readFIFO()
{
	/*
	 * @brief  readFIFO
	 * @details     Function to read data of sensor
	 * @param[in]
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 25 de abr de 2019
	 *
	 */

	uint8_t  buffer[4 * 16] = { 0 };
	uint16_t rawIR = 0;
	uint16_t rawRed = 0;
	uint8_t readBuff[3] = { 0 };
	char value[50] = { 0 };

	/* Read Fifo write pointer */
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_WRITE_POINTER, readBuff, 3);

#ifdef MAX30100_DEBUG
	PRINTF("Value of WRP = 0x%x\r\n", readBuff[0]);
	PRINTF("Value of OVF = 0x%x\r\n", readBuff[1]);
	PRINTF("Value of RD_PTR = 0x%x\r\n", readBuff[2]);
#endif

#ifdef MAX30100_DEBUG
	sprintf(value, "Value of WRP = 0x%x\r\n", readBuff[0]);
	USART_Printf(value);
	sprintf(value, "Value of OVF = 0x%x\r\n", readBuff[1]);
	USART_Printf(value);
	sprintf(value, "Value of RD_PTR = 0x%x\r\n", readBuff[2]);
	USART_Printf(value);
#endif

	int numberOfSamples = readBuff[0] - readBuff[2];

	if (numberOfSamples == 0)
		return;

	if (numberOfSamples < 0) {
		numberOfSamples += 16;
	} else	if ( readBuff[1] == 0xF )	{
		numberOfSamples = 16;
	}

	if ( numberOfSamples > 0 )
	{
#ifdef MAX30100_DEBUG
		sprintf(value, "Number of samples = %d\r\n", numberOfSamples);
		USART_Printf(value);
#endif
		I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_DATA, buffer, numberOfSamples*4);
		for (uint8_t i = 0; i < numberOfSamples; i++)
		{
			/*		*/
			rawIR = (buffer[i * 4] << 8) | buffer[1 + i * 4];
			rawRed = (buffer[2 + i * 4] << 8) | buffer[3 + i * 4];;
			GPIO_PortToggle(GPIOB, 1u << 8U); /*	TODO Changed it to function call	*/

#ifdef MAX30100_DEBUG
			sprintf(value, " S[%d] = %04x\r\n", i, rawIR);
#else
			sprintf(value, "%04x\t%04x\r\n", rawIR, rawRed);
			USART_Printf(value);
#endif
			break;
		}
	}

#ifdef MAX30100_DEBUG
	PRINTF("IR output = 0x%x\r\n", rawIR);
	PRINTF("Red output = 0x%x\r\n", rawRed);
#endif

}

static void USART_Printf(const char* string)
{
	APP_USART.Send(string, strlen(string));
	while (txOnGoing)
	{

	}

	txOnGoing = 1;
}

int main(void)

{
	/*	TODO remove it from here		*/
	/* Define the init structure for the output toggle pin*/
	gpio_pin_config_t led_config = {
			kGPIO_DigitalOutput, 0,
	};

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_I2C_ReleaseBus();
	BOARD_InitDebugConsole();

	/* Init output LED GPIO. */
	/* TODO customize gpio in extern file */
	GPIO_PinInit(GPIOB, 8U, &led_config);

	/*			USART Config 					*/
	CLOCK_SetLpsci0Clock(0x1U);
	APP_USART.Initialize(USART_SignalEvent_t);
	APP_USART.PowerControl(ARM_POWER_FULL);
	APP_USART.Control(ARM_USART_MODE_ASYNCHRONOUS, BOARD_DEBUG_UART_BAUDRATE);


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

	/*	Change position of below function, join it with init toggle pin	*/
	LED_INIT();

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
	struct dcFilter_t dcFilterIR;
	struct dcFilter_t dcFilterRed;
	struct meanDiffFilter_t meanDiffIR;
	struct butterworthFilter_t filter;

	/*	Clean up structs to start filter process	*/
	dcFilterClear(&dcFilterIR);
	dcFilterClear(&dcFilterRed);
	meanDiffFilterClear(&meanDiffIR);

	for(;;)
	{
#ifndef MAX30100_FIFO_RAW_OUTPUT
		isValidSample = MAX30100_Get_Sample(&sample.rawIR, &sample.rawRed);
		if( isValidSample )
		{
			dcFilterIR = dcRemoval((float)sample.rawIR, dcFilterIR.w, ALPHA);
			dcFilterRed = dcRemoval((float)sample.rawRed, dcFilterRed.w, ALPHA);
			float meanDiffResIR = meanDiff(dcFilterIR.result, &meanDiffIR);
			/*	IF mean vector was fully filed	*/
			if (meanDiffIR.count >= MEAN_FILTER_SIZE)
			{
				/*	toggle a pin here if you want test loop performance	*/
				/*	GPIO_PortToggle(GPIOB, 1u << 8U);	*/
				/*	low pass filter implementation	*/
				lowPassFilter(meanDiffResIR, &filter);
#ifdef MAX30100_FILTERED_RAW_OUTPUT
				char value[50] = { 0 };
				sprintf(value, "%d\r\n", (int)filter.result*1000);
				USART_Printf(value);
#endif
			}
		}
#endif

		/*	TODO Verify if lptmrCounter and lptmrCounter2 can be short type	*/

		if (lptmrCounter > 9) /*	> 9 ms	*/
		{
#ifdef MAX30100_FIFO_RAW_OUTPUT
			readFIFO();
			lptmrCounter = 0;
#endif
		}
		if (lptmrCounter2 > 999) /* 999 ms		*/
		{
			lptmrCounter2 = 0;
			LED_TOGGLE();
			//GPIO_PortToggle(GPIOB, 1u << 8U);
		}
		/*	Adjust Red Led current balancing with 500 ms (10 ms * 50 = 500 ms)	*/


#ifdef MAX30100_DEBUG
		PRINTF("---------------------------------------------\r\n");
#endif
	}

}
