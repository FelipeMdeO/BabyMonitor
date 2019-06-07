/*
 * max30102.h
 *
 *  Created on: 4 de jun de 2019
 *      Author: dell-felipe
 */

#include "init.h"
#include "vars.h"
#include <fsl_debug_console.h>

#ifndef MAX30102_MAX30102_H_
#define MAX30102_MAX30102_H_

#define MAX30102_DEVICE             0x57

#define MAX30102_INTSTAT1 			0x00
#define MAX30102_INTSTAT2 			0x01
#define MAX30102_INTENABLE1 		0x02
#define MAX30102_INTENABLE2 		0x03

// FIFO Registers
#define MAX30102_FIFOWRITEPTR  		0x04
#define MAX30102_FIFOOVERFLOW  		0x05
#define MAX30102_FIFOREADPTR  		0x06
#define MAX30102_FIFODATA 			0x07

// Configuration Registers
#define MAX30102_FIFOCONFIG  		0x08
#define MAX30102_MODECONFIG  		0x09
#define MAX30102_SPO2CONFIG  		0x0A
#define MAX30102_LED1_PULSEAMP  	0x0C
#define MAX30102_LED2_PULSEAMP  	0x0D
#define MAX30102_MULTILEDCONFIG1	0x11
#define MAX30102_MULTILEDCONFIG2  	0x12

// Die Temperature Registers
#define MAX30102_DIETEMPINT  		0x1F
#define MAX30102_DIETEMPFRAC  		0x20
#define MAX30102_DIETEMPCONFIG  	0x21

// Proximity Function Registers
//#define MAX30102_PROXINTTHRESH  	0x30; This register was removed in revision 1

// Part ID Registers
#define MAX30102_REVISIONID  		0xFE
#define MAX30102_PARTID  			0xFF

#ifdef MAX30102
/*	Page 18	*/
typedef enum Mode {
	MAX30102_MODE_HR_ONLY                 = 0x02,
	MAX30102_MODE_SPO2_HR                 = 0x03,
	MAX30102_MODE_RESET					  = 0x40,
	MAX30102_MODE_SHUTDOWN				  = 0x80
} Mode;

//LED Pulse Amplitude Configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Default is 0x1F which gets us 6.4mA
//powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
//powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
//powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
//powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
typedef enum LEDCurrent {
	MAX30102_LED_CURRENT_MIN          = 0x00,
	MAX30102_LED_CURRENT_MID          = 0x7F,
	MAX30102_LED_CURRENT_MAX          = 0xFF,
} LEDCurrent;

/* De acordo com a documentacao da Sparkfun, quanto maior o pulse with maior a distancia de leitura do sensor	*/

/*
 *  The longer the pulse width the longer range of detection you'll have
 *  At 69us and 0.4mA it's about 2 inches
 *  At 411us and 0.4mA it's about 6 inches
 *
 */
typedef enum LEDPulseWidth {
	MAX30102_PULSE_WIDTH_69US_ADC_15      = 0x00,
	MAX30102_PULSE_WIDTH_118US_ADC_16     = 0x01,
	MAX30102_PULSE_WIDTH_215US_ADC_17     = 0x02,
	MAX30102_PULSE_WIDTH_411US_ADC_18     = 0x03
} LEDPulseWidth;

/* Page 19*/
typedef enum SamplingRate {
	MAX30102_SAMPLING_RATE_50HZ           = 0x00,
	MAX30102_SAMPLING_RATE_100HZ          = 0x01,
	MAX30102_SAMPLING_RATE_200HZ          = 0x02,
	MAX30102_SAMPLING_RATE_400HZ          = 0x03,
	MAX30102_SAMPLING_RATE_800HZ          = 0x04,
	MAX30102_SAMPLING_RATE_1000HZ         = 0x05,
	MAX30102_SAMPLING_RATE_1600HZ         = 0x06,
	MAX30102_SAMPLING_RATE_3200HZ         = 0x07
} SamplingRate;



/*	Functions Prototype	*/
bool MAX30102_Read_Part_Id(uint8_t *id);
bool MAX30102_Read_Part_Rev(uint8_t *rev);
void MAX30102_Init(void);
void MAX30102_Read_All_Reg(void);

#endif

#endif /* MAX30102_MAX30102_H_ */
