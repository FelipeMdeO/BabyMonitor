/*
 * init.c
 *
 *  Created on: 2 de mai de 2019
 *      Author: felipe
 */

#include "init.h"

volatile uint32_t msTicks = 0;                              /* Variable to store millisecond ticks */
volatile bool completionFlag = false;
volatile bool nakFlag = false;

/*								USART Variables 														*/
uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = { 0 }; 	/*	Data buffer 1 byte								*/
volatile bool txOnGoing = false; 					/*	Variable to identify if data finished transfer	*/

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

void BLE_Pin_Init(uint8_t logic_level)
{
	GPIO_PinWrite(GPIOB, 1U << 8U, logic_level);
	GPIOB->PDDR |= (1U << 8U); /*!< Enable target LED_BLUE */
}

void LedInit(void)
{
	LED_GREEN_INIT(LOGIC_LED_ON);
	LED_RED_INIT(LOGIC_LED_OFF);
	LED_BLUE_INIT(LOGIC_LED_OFF);
	BLE_Pin_Init(1);	/*	Pin connected at p channel mosfet	*/
}

void buttonInit(void) {
	gpio_pin_config_t button_config =
	{
			kGPIO_DigitalInput,
			0,
	};
	port_pin_config_t button_config_port = {
			kPORT_PullUp, /*	Only this value is acceptable*/
			kPORT_PassiveFilterEnable,
			kPORT_LowDriveStrength,
			kPORT_MuxAsGpio,

	};

	GPIO_PinInit(BOARD_BUTTON_GPIO, BOARD_BUTTON_PIN, &button_config);
	PORT_SetPinConfig(BOARD_BUTTON_PORT, BOARD_BUTTON_PIN, &button_config_port);
}

void init_gpio_pins(void)
{
	/*
	 * @brief  init_gpio_pins
	 * @details     Function to initialize gpio pins
	 * @param[in] void
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 13 de mai de 2019
	 *
	 */
	/* Define the init structure for the output toggle pin*/
	gpio_pin_config_t toggle_config = {
			kGPIO_DigitalOutput, 0,
	};

	GPIO_PinInit(GPIOB, 8U, &toggle_config);

	BLE_OFF(); /*	Startup code with BLE power off	*/
	LedInit();

	buttonInit(); /* Init button pin		*/

}

void delay(void)
{
	volatile uint32_t i = 0;
	for (i = 0; i < 50000; ++i)
	{
		__asm("NOP"); /* delay */
	}
}

bool BUTTON_VALUE(void) {
	return !GPIO_ReadPinInput(BOARD_BUTTON_GPIO, BOARD_BUTTON_PIN);
}


void BEAT_LED(void)
{
	LED_RED_ON();
	delay();
	LED_RED_OFF();
}

void BLE_ON(void)
{
	GPIO_PortClear(GPIOB, 1U << 8U); /*!< Turn on target LED_RED */
//	GPIO_WritePinOutput(GPIOB, 1u << 8U, 0);
}

void BLE_OFF(void)
{
	GPIO_PortSet(GPIOB, 1U << 8U); /*!< Turn off VCC of mosfet */
//	GPIO_WritePinOutput(GPIOB, 1u << 8U, 1);
}

void ALL_LED_ON() {
	LED_RED_ON();
	LED_BLUE_ON();
	LED_GREEN_ON();
}

void ALL_LED_OFF() {
	LED_RED_OFF();
	LED_BLUE_OFF();
	LED_GREEN_OFF();
}

void BLINK_BLUE(void)
{
	LED_BLUE_ON();
	delay();
	delay();
	delay();
	delay();
	delay();
	delay();
	delay();
	LED_BLUE_OFF();
}

void init_usart(void)
{
	txOnGoing = true;

	CLOCK_SetLpsci0Clock(0x1U);
	APP_USART.Initialize(USART_SignalEvent_t);
	APP_USART.PowerControl(ARM_POWER_FULL);
	APP_USART.Control(ARM_USART_MODE_ASYNCHRONOUS, BOARD_DEBUG_UART_BAUDRATE);
}

void i2c_release_bus_delay(void)
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

bool I2C_WriteReg(uint8_t device_addr, uint8_t reg_addr, uint8_t value)
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

bool I2C_ReadRegs(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
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

void I2C_Init(void)
{
	I2C_MASTER.Initialize(I2C_MasterSignalEvent_t);
	I2C_MASTER.PowerControl(ARM_POWER_FULL);
	I2C_MASTER.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS);
}

void config_lptmr(void)
{
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
}
