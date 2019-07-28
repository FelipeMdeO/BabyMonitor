/*
 * max30102.c
 *
 *  Created on: 4 de jun de 2019
 *      Author: dell-felipe
 */



#include "max30102.h"


uint8_t redLedCurrent = 0x00;
uint8_t irLedCurrent  = 0x3F;
//char value[25] = { 0 };

//uint8_t redLedCurrent = MAX30102_LED_CURRENT_DEFAULT;
//uint8_t irLedCurrent  = MAX30102_LED_CURRENT_DEFAULT;

/*	Static prototypes	*/
static void Set_mode(uint8_t mode);
static void setLEDCurrents(uint8_t RedCurrent, uint8_t IrCurrent);
static void setSamplingRate(uint8_t rate);

/*	Static Functions	*/

/*******************  Set_mode  *********************/
static void Set_mode(uint8_t mode)
{
	/*
	 * @brief  Set_mode
	 * @details     Set mode of work of max30102
	 * 				it can be:
	 * 				MAX30102_MODE_HR_ONLY
	 * 				MAX30102_MODE_SPO2_HR
	 * 				MAX30102_MODE_RESET
	 * 				MAX30102_MODE_SHUTDOWN
	 * @param[in] uint8_t mode
	 * @param[out]
	 * @return bool
	 *
	 * author dell-felipe
	 * date 6 de jun de 2019
	 *
	 */

	uint8_t readBuff[1];
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_MODECONFIG, readBuff, 1);

#ifdef MAX30102_DEBUG
	/* Read Old value*/
	printf("The old value to mode is 0x%x \r\n", readBuff[0]);
#endif

	/* Write new value	*/
	if (mode == MAX30102_MODE_RESET)
		/*	Change only bit of relationship with reset */
		I2C_WriteReg(MAX30102_DEVICE, MAX30102_MODECONFIG, (readBuff[0] & 0xBF) | mode);
	else if (mode == MAX30102_MODE_SHUTDOWN)
		/*	Change only bit of relationship with shutdown */
		I2C_WriteReg(MAX30102_DEVICE, MAX30102_MODECONFIG, (readBuff[0] & 0x7F) | mode);
	else
		/*	Change Bits relationship with mode	*/
		I2C_WriteReg(MAX30102_DEVICE, MAX30102_MODECONFIG, (readBuff[0] & 0xF8) | mode);

	/* Read new value*/
#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_MODECONFIG, readBuff, 1);
	printf("The new value of mode register is: 0x%x \n", readBuff[0]);
#endif
}
/******************* Fim  Set_mode  *********************/


/*******************  setLEDCurrents  *********************/
static void setLEDCurrents(uint8_t RedLedCurrent, uint8_t IrLedCurrent)
{
	/*
	 * @brief  setLEDCurrents
	 * @details     Set led current, table 8 of datasheet
	 * @param[in] ,
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 6 de jun de 2019
	 *
	 */
	uint8_t readBuff[1];

	/* Read Old value*/
#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_LED1_PULSEAMP, readBuff, 1);
	printf("The old value to Red led current is 0x%x \r\n", readBuff[0]);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_LED2_PULSEAMP, readBuff, 1);
	printf("The old value to Ir led current is 0x%x \r\n", readBuff[0]);
#endif

	/* Write new value	*/
	/*	Change only bit of relationship with reset */
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_LED1_PULSEAMP, RedLedCurrent);
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_LED2_PULSEAMP, IrLedCurrent);

	/* Read new value*/
#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_LED1_PULSEAMP, readBuff, 1);
	printf("The new value of mode register is: 0x%x \n", readBuff[0]);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_LED2_PULSEAMP, readBuff, 1);
	printf("The new value of mode register is: 0x%x \n", readBuff[0]);
#endif

}

/*******************  end setLEDCurrents  *********************/

/*******************  setSampleRating  *********************/
static void setSamplingRate(uint8_t rate)
{
	/*
	 * @brief  setSampleRating
	 * @details     rate to sensor generate new data
	 * @param[in] uint8_t
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 6 de jun de 2019
	 *
	 */

	uint8_t readBuff[1];

	/* Read Old value*/
#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, readBuff, 1);
	printf("The old value of Sample Rate is 0x%x \r\n", ( (readBuff[0] & 0x1C) >> 2 ) );
#endif

	/* Write new value	*/
	/*	Change only bit of relationship with rate */
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_SPO2CONFIG, (readBuff[0] & 0xE3) | (rate<<2));

	/* Read new value*/
#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, readBuff, 1);
	printf("The new value of Sample Rate is: 0x%x \n", ( (readBuff[0] & 0x1C) >> 2 ));
#endif

}

/*******************  end setLEDCurrents  *********************/

/*******************  setPulseWidth  *********************/
static void setPulseWidth(uint8_t pulseWidth)
{
	uint8_t readBuff[1] = { 0 };
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, readBuff, 1);

#ifdef MAX30102_DEBUG
	PRINTF("The old value to pulseWidth is: 0x%x \r\n", (readBuff[0] & 0x3));
#endif

	/* MAX30100_SPO2_CONF setup to INPUT (us)  */
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_SPO2CONFIG, ( readBuff[0] & 0xFC ) | pulseWidth );

#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, readBuff, 1);
	PRINTF("The new value to pulseWidth is: 0x%x \r\n", (readBuff[0] & 0x3));
#endif
}
/*******************  end setPulseWidth  *********************/

/********************  setSampleAverage  **********************/
static void setSampleAverage(uint8_t nSamples)
{
	uint8_t readBuff[1] = { 0 };
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_FIFOCONFIG, readBuff, 1);

#ifdef MAX30102_DEBUG
	PRINTF("The old value to SMP_AVE is: 0x%x \r\n", (readBuff[0]));
#endif

	/* MAX30102_FIFOCONFIG setup to INPUT samples  */
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_FIFOCONFIG, ( readBuff[0] & 0x1F ) | (nSamples << 5) );

#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_FIFOCONFIG, readBuff, 1);
	PRINTF("The new value to SMP_AVE is: 0x%x \r\n", (readBuff[0]));
#endif
}
/********************  end Average  ************************/

/*******************  Read Part Id  *********************/
bool MAX30102_Read_Part_Id(uint8_t *id) {

	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_PARTID, id, 1);

	if (*id == 0x15)
		return true;
	else
		return false;
}

/*******************  Read Part Rev  *********************/
bool MAX30102_Read_Part_Rev(uint8_t *rev) {
	return false;
}
/*******************  end Read Part Rev  *********************/

/*******************  SPO2 ADC Range  *********************/
static void setSPO2_ADC_Range(uint8_t range)
{
	uint8_t readBuff[1] = { 0 };
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, readBuff, 1);

#ifdef MAX30102_DEBUG
	PRINTF("The old value to SPO2_ADC_RGE is: 0x%x \r\n", readBuff[0] >> 5);
#endif

	/* MAX30102_FIFOCONFIG setup to INPUT samples  */
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_SPO2CONFIG, ( readBuff[0] & 0x1F ) | (range << 5) );

#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, readBuff, 1);
	PRINTF("The new value to SPO2_ADC_RGE is: 0x%x \r\n", readBuff[0] >> 5);
#endif
}
/*******************  end SPO2 ADC Range  *********************/



static void MAX30102_RESET(void)
{
	/* MAX30102_FIFOCONFIG setup to INPUT samples  */
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_MODECONFIG, (1 << 6) );
}

static void MAX30102_CLEAR_FIFO(void)
{
	I2C_WriteReg(MAX30102_DEVICE, 0x4, 0x0);
	I2C_WriteReg(MAX30102_DEVICE, 0x05, 0x0);
	I2C_WriteReg(MAX30102_DEVICE, 0x06, 0x0);
}

/*******************  Read Part Id  *********************/
void MAX30102_Init(void)
{
	//	MAX30102_RESET(); // Uncomment to reset sensor !
	Set_mode(MAX30102_MODE_SPO2_HR);
	setLEDCurrents(redLedCurrent, irLedCurrent);
	setSamplingRate(MAX30102_SAMPLING_RATE_400HZ);
	setPulseWidth(MAX30102_PULSE_WIDTH_411US_ADC_18);
	setSampleAverage(MAX30102_SAMPLE_AVERAGE_4);
	setSPO2_ADC_Range(0b01);
	MAX30102_CLEAR_FIFO();
}

/*******************  end MAX30102_Init  *********************/

/*******************  MAX30102_Get_Sample  *********************/
bool MAX30102_Get_Sample(uint32_t *IR_sample, uint32_t *Red_sample)
{
	uint8_t buffer[6] = { 0 };
	uint8_t WR =  0;
	uint8_t OVF = 0;
	uint8_t RP = 0;
	int numberOfSamples = 0;

	/* Read Fifo wrute pointer */
	I2C_ReadRegs(MAX30102_DEVICE, 0x04, &WR, 1);
	I2C_ReadRegs(MAX30102_DEVICE, 0x05, &OVF, 1);
	I2C_ReadRegs(MAX30102_DEVICE, 0x06, &RP, 1);

	numberOfSamples = WR - RP;
	printf("%d\n", numberOfSamples);

	/* Check for overflow situation */
	//	if ( numberOfSamples < 0 )
	//		numberOfSamples +=32;
	//	else if ( OVF == 0x1F )
	//		numberOfSamples = 32;
	if (OVF == 0x1F)
	{
		numberOfSamples = 32;
	}


	/* Read available samples */
	if ( numberOfSamples == 0)
		return false;
	else if ( numberOfSamples > 0 )
	{
		/* Read only one sample per time */
		I2C_ReadRegs(MAX30102_DEVICE, MAX30102_FIFODATA, buffer, 6);
		*IR_sample = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]);
		*IR_sample &= 0x03FFFF; //Mask MSB [23:18]
		*Red_sample = ((buffer[3] << 16) | (buffer[4] << 8) | buffer[5]);
		*Red_sample &= 0x03FFFF; //Mask MSB [23:18]
		//		printf("%d %d\n", *IR_sample, *Red_sample);
		//		sprintf(value, "%d %d\n", *IR_sample, *Red_sample);
		return true;
	}
	return false;
}

void MAX30102_Print_Samples(void)
{
	uint8_t buffer[6 * 32] = { 0 };
	uint8_t readBuff[3] = { 0 };
	uint32_t IR_sample, Red_sample;

	char aux[20] = { 0 };

	/* Read Fifo write pointer */
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_FIFOWRITEPTR, readBuff, 3);

//	PRINTF("Value of WRP = 0x%x\r\n", readBuff[0]);
//	PRINTF("Value of OVF = 0x%x\r\n", readBuff[1]);
//	PRINTF("Value of RD_PTR = 0x%x\r\n", readBuff[2]);

	int numberOfSamples = readBuff[0] - readBuff[2];
//	PRINTF("Nsamples = %d\r\n", numberOfSamples);

	if(numberOfSamples < 0 )	{
		numberOfSamples += 32;
	} else if (readBuff[1] == 0x1F)	{
		numberOfSamples = 32;
	}

	if (numberOfSamples > 0) {
		I2C_ReadRegs(MAX30102_DEVICE, MAX30102_FIFODATA, buffer, numberOfSamples*6);
		for (uint8_t i = 0; i < numberOfSamples; i++)
		{
			Red_sample = ((buffer[i * 6] << 16) | (buffer[1 + i * 6] << 8 ) | (buffer[2 + i * 6]));
			Red_sample &= 0x03FFFF; //Mask MSB [23:18]
			IR_sample = ((buffer[3 + i * 6] << 16) | (buffer[4 + i * 6] << 8) | (buffer[5 + i * 6]));
			IR_sample &= 0x03FFFF; //Mask MSB [23:18]
			sprintf(aux, "%d;%d\n", IR_sample, Red_sample);
			USART_Printf(aux);
		}
	}
}
/******************* end MAX30102_Get_Sample  *********************/

/******************** MAX30102_Read_All_Reg  **********************/
void MAX30102_Read_All_Reg(void)
{
	/*
	 * @brief  MAX30102_Read_All_Reg
	 * Function to read all register of max30102 and print it
	 * @details
	 * @param[in] void
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 27 de jul de 2019
	 *
	 */
	uint8_t reg = 0;

	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_REVISIONID, &reg, 1);
	PRINTF("MAX30102_REVISIONID = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_PARTID, &reg, 1);
	PRINTF("MAX30102_PARTID = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_FIFOCONFIG, &reg, 1);
	PRINTF("MAX30102_FIFOCONFIG = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_MODECONFIG, &reg, 1);
	PRINTF("MAX30102_MODECONFIG = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, &reg, 1);
	PRINTF("MAX30102_SPO2CONFIG = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_LED1_PULSEAMP, &reg, 1);
	PRINTF("MAX30102_LED1_PULSEAMP = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_LED2_PULSEAMP, &reg, 1);
	PRINTF("MAX30102_LED2_PULSEAMP = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_MULTILEDCONFIG1, &reg, 1);
	PRINTF("MAX30102_MULTILEDCONFIG1 = 0x%x\n", reg);
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_MULTILEDCONFIG2, &reg, 1);
	PRINTF("MAX30102_MULTILEDCONFIG2 = 0x%x\n", reg);
}
/******************** end MAX30102_Read_All_Reg  **********************/
