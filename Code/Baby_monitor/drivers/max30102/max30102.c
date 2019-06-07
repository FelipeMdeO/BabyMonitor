/*
 * max30102.c
 *
 *  Created on: 4 de jun de 2019
 *      Author: dell-felipe
 */



#include "max30102.h"


uint8_t redLedCurrent = MAX30102_LED_CURRENT_MID;
uint8_t irLedCurrent  = MAX30102_LED_CURRENT_MID;

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
	printf("The old value of Sample Rate is 0x%x \r\n", ( (readBuff[0] & 0x1C) >> 3 ) );
#endif

	/* Write new value	*/
	/*	Change only bit of relationship with rate */
	I2C_WriteReg(MAX30102_DEVICE, MAX30102_SPO2CONFIG, ( (rate << 3) & 0x1C ) );

	/* Read new value*/
#ifdef MAX30102_DEBUG
	I2C_ReadRegs(MAX30102_DEVICE, MAX30102_SPO2CONFIG, readBuff, 1);
	printf("The new value of Sample Rate is: 0x%x \n", ( (readBuff[0] & 0x1C) >> 3 ));
#endif

}

/*******************  end setLEDCurrents  *********************/

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

/*******************  Read Part Id  *********************/
void MAX30102_Init(void)
{
	Set_mode(MAX30102_MODE_SPO2_HR);
	setLEDCurrents(redLedCurrent, irLedCurrent);
	setSamplingRate(MAX30102_SAMPLING_RATE_100HZ);
}
