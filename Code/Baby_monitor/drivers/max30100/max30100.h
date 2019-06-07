/*
 * max30100.h
 *
 *  Created on: 7 de abr de 2019
 *      Author: dell-felipe
 */



#ifndef MAX30100_MAX30100_H_
#define MAX30100_MAX30100_H_

#include "init.h"
#include "vars.h"

//#define MAX30100_DEBUG

#define MAX30100_DEVICE                   0x57

/* Config registers */
#define MAX30100_MODE_CONF                0x06
#define MAX30100_SPO2_CONF                0x07
#define MAX30100_LED_CONF                 0x09

/* Fifo registers */
#define MAX30100_FIFO_WRITE_POINTER       0x02					/*!<  FIFO Write Pointer        ( Read/Write ) ( Default 0x00 ) */
#define MAX30100_FIFO_OVERFLOW_COUNTER    0x03					/*!<  Over Flow Counter         ( Read/Write ) ( Default 0x00 ) */
#define MAX30100_FIFO_READ                0x04					/*!<  FIFO Read Pointer         ( Read/Write ) ( Default 0x00 ) */
#define MAX30100_FIFO_DATA                0x05					/*!<  FIFO Data Register        ( Read/Write ) ( Default 0x00 ) */

/* MAX30100 parameters */
#define DEFAULT_OPERATING_MODE            MAX30100_MODE_SPO2_HR

/*!!!IMPORTANT
 * You can't just throw these two values at random. Check Check table 8 in datasheet on page 19.
 * 100hz + 1600us is max for that resolution
 */

#ifdef MAX30100
#define MAX30100_SPO2_HI_RES_EN           (1 << 6)
#define DEFAULT_SAMPLING_RATE             MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH           MAX30100_PULSE_WIDTH_1600US_ADC_16

#define DEFAULT_IR_LED_CURRENT            MAX30100_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT          MAX30100_LED_CURRENT_27_1MA

/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         2000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500 /*	TODO	Use this value in loop of lptmrCounter2		*/

typedef enum Mode {
	MAX30100_MODE_HR_ONLY                 = 0x02,
	MAX30100_MODE_SPO2_HR                 = 0x03
} Mode;

typedef enum LEDCurrent {
	MAX30100_LED_CURRENT_0MA              = 0x00,
	MAX30100_LED_CURRENT_4_4MA            = 0x01,
	MAX30100_LED_CURRENT_7_6MA            = 0x02,
	MAX30100_LED_CURRENT_11MA             = 0x03,
	MAX30100_LED_CURRENT_14_2MA           = 0x04,
	MAX30100_LED_CURRENT_17_4MA           = 0x05,
	MAX30100_LED_CURRENT_20_8MA           = 0x06,
	MAX30100_LED_CURRENT_24MA             = 0x07,
	MAX30100_LED_CURRENT_27_1MA           = 0x08,
	MAX30100_LED_CURRENT_30_6MA           = 0x09,
	MAX30100_LED_CURRENT_33_8MA           = 0x0A,
	MAX30100_LED_CURRENT_37MA             = 0x0B,
	MAX30100_LED_CURRENT_40_2MA           = 0x0C,
	MAX30100_LED_CURRENT_43_6MA           = 0x0D,
	MAX30100_LED_CURRENT_46_8MA           = 0x0E,
	MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;

typedef enum LEDPulseWidth {
	MAX30100_PULSE_WIDTH_200US_ADC_13     = 0x00,
	MAX30100_PULSE_WIDTH_400US_ADC_14     = 0x01,
	MAX30100_PULSE_WIDTH_800US_ADC_15     = 0x02,
	MAX30100_PULSE_WIDTH_1600US_ADC_16    = 0x03,
} LEDPulseWidth;

typedef enum SamplingRate {
	MAX30100_SAMPLING_RATE_50HZ           = 0x00,
	MAX30100_SAMPLING_RATE_100HZ          = 0x01,
	MAX30100_SAMPLING_RATE_167HZ          = 0x02,
	MAX30100_SAMPLING_RATE_200HZ          = 0x03,
	MAX30100_SAMPLING_RATE_400HZ          = 0x04,
	MAX30100_SAMPLING_RATE_600HZ          = 0x05,
	MAX30100_SAMPLING_RATE_800HZ          = 0x06,
	MAX30100_SAMPLING_RATE_1000HZ         = 0x07
} SamplingRate;



/*	Functions Prototype	*/
bool MAX30100_Get_Sample(uint16_t *IR_sample, uint16_t *Red_sample);
void MAX30100_Init(void);
void MAX30100_ClearFIFO(void);
void setLEDCurrents(uint8_t redLedCurrent, uint8_t IRLedCurrent);
void setSamplingRate(uint8_t samplingRate);
void setHighresModeEnabled(bool enabled);
void balanceIntesities(float redLedDC, float IRLedDC);

extern LEDCurrent redLedCurrent;
extern bool canAdjustRedCurrent;

#endif
#endif /* MAX30100_MAX30100_H_ */
