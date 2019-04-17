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


/* Code includes */
#include "max30100/max30100.h"

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
#define LPTMR_USEC_COUNT 10000U /* 10 ms */
#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()

/*		DEBUG Define	*/
//	#define MAX30100_DEBUG

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

static bool I2C_WriteReg(uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadRegs(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
static void MAX30100_ClearFIFO(void);
static void USART_Printf(const char* string);

/*								USART user Signal Event 												*/
void USART_SignalEvent_t(uint32_t event);

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile bool completionFlag = false;
volatile bool nakFlag = false;

/*								USART Variables 														*/
uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = { 0 }; 	/*	Data buffer 1 byte								*/
volatile bool txOnGoing = false; 					/*	Variable to identify if data finished transfer	*/

volatile uint32_t lptmrCounter = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/

void LPTMR_LED_HANDLER(void)
{
    LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);
    lptmrCounter++;
    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
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
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_MODE_CONF, mode);

	/* Confirm if wanted value was be obtained */
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_MODE_CONF, readBuff, 1);
#ifdef MAX30100_DEBUG
	PRINTF("The new value to mode is 0x%x\r\n", readBuff[0]);
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
#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);
	PRINTF("The old value to pulseWidth is: 0x%x \r\n", readBuff[0]);
#endif
	/* MAX30100_SPO2_CONF setup to 1600 us  */
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_SPO2_CONF, pulseWidth);
#ifdef MAX30100_DEBUG
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_SPO2_CONF, readBuff, 1);
	PRINTF("The new value to pulseWidth is: 0x%x \r\n", readBuff[0]);
#endif
}

static void MAX30100_Init(void)
{
	/* This function will be initialize register to do max30100 operational
	 * TODO
	 * receive parameters of this function to customize set mode
	 */


	Set_mode(MAX30100_MODE_HR_ONLY);
	/* DEFAULT_IR_LED_CURRENT = MAX30100_LED_CURRENT_50MA*/
	setLEDCurrents(MAX30100_LED_CURRENT_0MA, DEFAULT_IR_LED_CURRENT);
	setLEDPulseWidth(DEFAULT_LED_PULSE_WIDTH);
}

static void MAX30100_readFIFO(bool completeRead)
{
	/*
	 * @brief  MAX30100_readFIFO
	 * @details
	 * @param[in] bool completeRead indicates if will be read IR and Red, if completeRead = 1, will be read booth leds
	 * @param[out] void
	 * @return void
	 *
	 * author dell-felipe
	 * date 7 de abr de 2019
	 *
	 */

	uint8_t readBuff[3] = { 0 };
	uint8_t FIFO_buff[64] = { 0 }; 									 /*!<  FIFO buffer                           */
	uint16_t FIFO_IR_samples[16];                                    /*!<  FIFO IR buffer                        */
	uint16_t FIFO_RED_samples[16];                                   /*!<  FIFO RED buffer                       */
	uint32_t num_avaible_samples = 0;
	const unsigned short int num_sample_to_read = 16;

	uint8_t FIFO_WR_PTR = 0;
	uint8_t OVF_COUNTER = 0;
	uint8_t FIFO_RD_PTR = 0;

	char value[50] = { 0 };

	/* First Clear FIFO */
	//MAX30100_ClearFIFO();

	/* First Transaction : Get the FIFO_WR_PTR, OVR_COUNTER and FIFO_RD_PTR */
	/* Gets the location that max30100 writes the next sample */
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_WRITE_POINTER, readBuff, 3);

	FIFO_WR_PTR = readBuff[0];
	OVF_COUNTER = readBuff[1];
	FIFO_RD_PTR = readBuff[2];

	sprintf(value, "Value of WRP = 0x%x\r\n", FIFO_WR_PTR);
	USART_Printf(value);
	sprintf(value, "Value of OVF = 0x%x\r\n", OVF_COUNTER);
	USART_Printf(value);
	sprintf(value, "Value of RD_PTR = 0x%x\r\n", FIFO_RD_PTR);
	USART_Printf(value);

#ifdef MAX30100_DEBUG
	//PRINTF("The new value to WR_PTR[0] is 0x%x\r\n", FIFO_WR_PTR);
	//PRINTF("The new value to OVR_COUNT[1] is 0x%x\r\n", OVF_COUNTER);
	//PRINTF("The new value to FIFO_RD_PTR[2] is 0x%x\r\n", FIFO_RD_PTR);
#endif

	/* If the FIFO is full, just start reading data  */
	if ( OVF_COUNTER == 0x0F)
	{
		/* Get data from FIFO    */
		/* 16 Samples each one with 4 bytes */
		I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_DATA, FIFO_buff, num_sample_to_read*4);

		/* Parse the data */
		for (uint8_t i = 0; i < num_sample_to_read; i++)
		{
			FIFO_IR_samples[i] = FIFO_buff [ ( i << 2 ) ];
			FIFO_IR_samples[i] <<= 8U;
			FIFO_IR_samples[i] |= FIFO_buff [ ( i << 2) +1 ];

#ifdef MAX30100_DEBUG
			PRINTF("FIFO_IR_samples[%d] = 0x%x \r\n", i, FIFO_IR_samples[i]);
#endif

			if (completeRead)
			{

			FIFO_RED_samples[i] = FIFO_buff[ ( i << 2 ) + 2];

			FIFO_RED_samples[i] <<= 8U;
			FIFO_RED_samples[i] |= FIFO_buff[ ( (i << 2) + 2) + 1 ];
#ifdef MAX30100_DEBUG
			PRINTF("FIFO_RED_samples[%d] = 0x%x \r\n", i, FIFO_RED_samples[i]);
#endif
			}
		}
	}
	else
	{
		num_avaible_samples = (uint32_t)(FIFO_WR_PTR - FIFO_RD_PTR);

		/* Second Transaction: Read num_avaible_samples from the FIFO */
		if ( num_sample_to_read <= num_avaible_samples)
		{
			I2C_ReadRegs( MAX30100_DEVICE, MAX30100_FIFO_DATA, FIFO_buff, ( num_sample_to_read << 2U ) );

			for (uint8_t i = 0; i < num_sample_to_read; i++)
			{
				FIFO_IR_samples[i]     =   FIFO_buff[ ( i << 2 ) ];
				FIFO_IR_samples[i]   <<=   8U;
				FIFO_IR_samples[i]    |=   FIFO_buff[ ( i << 2 ) + 1 ];
#ifdef MAX30100_DEBUG
				PRINTF("FIFO_IR_samples[%d] = 0x%x \r\n", i, FIFO_IR_samples[i]);
#endif

				if(completeRead)
				{
					FIFO_RED_samples[i]    =   FIFO_buff[ ( i << 2 ) + 2 ];
					FIFO_RED_samples[i]  <<=   8U;
					FIFO_RED_samples[i]   |=   FIFO_buff[ ( ( i << 2 ) + 2 ) + 1 ];
#ifdef MAX30100_DEBUG
			PRINTF("FIFO_RED_samples[%d] = 0x%x \r\n", i, FIFO_RED_samples[i]);
#endif
				}

			}
		}
	}
}

static void MAX30100_ClearFIFO(void) {
	/*
	 * @brief  MAX30100_ClearFIFO
	 * @details     It clears the FIFO_WR_PTR, OVF_COUNTER, and FIFO_RD_PTR
 *             		registers to all zeros (0x00) to ensure the FIFO is empty and in a known state.
	 * @param[in] void
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 7 de abr de 2019
	 *
	 */

	uint8_t readBuff[3] = { 0 };

	uint8_t WR_PTR = 0;
	uint8_t OVF_COUNTER = 0;
	uint8_t FIFO_RD_PTR = 0;
	uint8_t NUM_AVAIBLE_SAMPLES = 0;

	/* Update the FIFO_WR_PTR, FIFO_WR_PTR and OVF_COUNTER registers    */
	I2C_WriteReg(MAX30100_DEVICE, MAX30100_FIFO_WRITE_POINTER, readBuff[0]);

	/* Get the FIFO_WR_PTR, OVF_COUNTER and FIFO_RD_PTR    */
	I2C_ReadRegs(MAX30100_DEVICE, MAX30100_FIFO_WRITE_POINTER, readBuff, 3);

	WR_PTR = readBuff[0];
	OVF_COUNTER = readBuff[1];
	FIFO_RD_PTR = readBuff[2];

#ifdef MAX30100_DEBUG
	PRINTF("The new value to WR_PTR[0] is 0x%x\r\n", WR_PTR);
	PRINTF("The new value to OVR_COUNT[1] is 0x%x\r\n", OVF_COUNTER);
	PRINTF("The new value to FIFO_RD_PTR[2] is 0x%x\r\n", FIFO_RD_PTR);
#endif
	NUM_AVAIBLE_SAMPLES = WR_PTR - FIFO_RD_PTR;
#ifdef MAX30100_DEBUG
	PRINTF("NUM_AVAIBLE_SAMPLES = 0x%x\r\n", NUM_AVAIBLE_SAMPLES);
#endif

}

static void readFIFO()
{
	uint8_t  buffer[4 * 16] = { 0 };
	uint16_t output = 0;
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
		for (uint8_t i = 0; i < numberOfSamples; ++i)
		{
			output = (buffer[i*4] << 8) | buffer[1 + i * 4];
#ifdef MAX30100_DEBUG
			sprintf(value, " S[%d] = %04x\r\n", i, output); /* Todo return to 4 bytes when use 2 leds*/
#else
			sprintf(value, "%04x\r\n", output);
			USART_Printf(value);
#endif
		}
	}

#ifdef MAX30100_DEBUG
	PRINTF("IR output = 0x%x\r\n", output);
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

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_I2C_ReleaseBus();
    BOARD_InitDebugConsole();

    /*			USART Config 					*/
    CLOCK_SetLpsci0Clock(0x1U);
    APP_USART.Initialize(USART_SignalEvent_t);
    APP_USART.PowerControl(ARM_POWER_FULL);
    APP_USART.Control(ARM_USART_MODE_ASYNCHRONOUS, BOARD_DEBUG_UART_BAUDRATE);

    /*		LPTMR Variables		*/
    uint32_t currentCounter = 0U;
    lptmr_config_t lptmrConfig;

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

    LED_INIT();

    txOnGoing = true;

    //USART_Printf("Application Starting\r\n");

#ifdef MAX30100_DEBUG
    PRINTF("\r\nI2C test -- Read MAX30100 Value\r\n");
#endif

    I2C_Init();

    MAX30100_Init();

    //MAX30100_ClearFIFO();
    //MAX30100_readFIFO(false);
    //readFIFO();

#ifdef MAX30100_DEBUG
    PRINTF("\r\nEnd of I2C MAX30100 test .\r\n");
#endif
    for(;;)
    {
    	if (currentCounter != lptmrCounter)
    	{
    		currentCounter = lptmrCounter;
    		readFIFO();
    		//MAX30100_readFIFO(0);
    		if (lptmrCounter > 99) {
    			lptmrCounter = 0;
    			 //USART_Printf("100 reads\r\n");
    			 LED_TOGGLE();
			}
    	}
    	//readFIFO();
    	//MAX30100_readFIFO(0);
#ifdef MAX30100_DEBUG
    	//PRINTF("---------------------------------------------\r\n");
#endif
    }

}
