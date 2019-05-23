/*
 * max30100.c
 *
 *  Created on: 1 de mai de 2019
 *      Author: dell-felipe
 */

#include "max30100.h"

LEDCurrent redLedCurrent = STARTING_RED_LED_CURRENT;
bool canAdjustRedCurrent = true;

void Set_mode(uint8_t mode)
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

void balanceIntesities(float redLedDC, float IRLedDC)
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

	char value[20] = { 0 };

	if (canAdjustRedCurrent)
	{
		if (redLedCurrent == MAX30100_LED_CURRENT_50MA)
		{
			sprintf(value, "Red Led Current 50 MA\r\n");
			USART_Printf(value);
		}
		else if (redLedCurrent == MAX30100_LED_CURRENT_0MA)
		{
			sprintf(value, "Red Led Current 0 MA\r\n");
			USART_Printf(value);
		}

		if (IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLedCurrent < MAX30100_LED_CURRENT_50MA)
		{
			redLedCurrent++;
			setLEDCurrents(redLedCurrent, DEFAULT_IR_LED_CURRENT);

			sprintf(value, "Red Led Current +\r\n");
			USART_Printf(value);
		}
		else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLedCurrent > 0)
		{
			redLedCurrent--;
			setLEDCurrents(redLedCurrent, DEFAULT_IR_LED_CURRENT);
			sprintf(value, "Red Led Current -\r\n");
			USART_Printf(value);
		}
		//		canAdjustRedCurrent = false;
	}
}

void setHighresModeEnabled(bool enabled)
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

void setSamplingRate(uint8_t rate)
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

void setLEDCurrents( uint8_t redLedCurrent, uint8_t IRLedCurrent )
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

	/* First read the default value on register to verify at end if the value changed */
#ifdef MAX30100_DEBUG
	uint8_t readBuff[1] = { 0 };
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

void setLEDPulseWidth(uint8_t pulseWidth)
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

void MAX30100_ClearFIFO(void) {
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

void MAX30100_Init(void)
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
	setLEDCurrents(STARTING_RED_LED_CURRENT, DEFAULT_IR_LED_CURRENT);
	setLEDPulseWidth(DEFAULT_LED_PULSE_WIDTH);
	setSamplingRate(DEFAULT_SAMPLING_RATE);
	setHighresModeEnabled(true);

	MAX30100_ClearFIFO(); /*		TODO Test it more!	*/
}


bool MAX30100_Get_Sample(uint16_t *IR_sample, uint16_t *Red_sample)
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

bool readFIFO(uint16_t *rawIR, uint16_t *rawRed)
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
	//	uint16_t rawIR = 0;
	//	uint16_t rawRed = 0;
	*rawIR = 0;
	*rawRed = 0;
	uint8_t readBuff[3] = { 0 };
	char value[20] = { 0 };

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
		return false;

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
			*rawIR = (buffer[i * 4] << 8) | buffer[1 + i * 4];
			*rawRed = (buffer[2 + i * 4] << 8) | buffer[3 + i * 4];

#ifdef MAX30100_DEBUG
			sprintf(value, " S[%d] = %04x\r\n", i, rawIR);
#else
			sprintf(value, "%04x\t%04x\r\n", *rawIR, *rawRed);
			USART_Printf(value);
#endif
			break;
		}
		return true;
	}
	return false;

#ifdef MAX30100_DEBUG
	PRINTF("IR output = 0x%x\r\n", *rawIR);
	PRINTF("Red output = 0x%x\r\n", *rawRed);
#endif

}
