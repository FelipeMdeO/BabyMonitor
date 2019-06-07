/*
 * vars.h
 *
 *  Created on: 18 de mai de 2019
 *      Author: dell-felipe
 */

#ifndef VARS_H_
#define VARS_H_

#include <stdbool.h>
#include <fsl_debug_console.h>

/*		DEBUG Defines	*/
//#define MAX30100_DEBUG
//#define MAX30100_FIFO_RAW_OUTPUT
//#define MAX30100_FILTERED_RAW_OUTPUT
//#define MAX30100_DC_RAW_OUTPUT
//#define MAX30100_BEAT_DETECTOR_OUTPUT

#define MAX30102
#define MAX30102_DEBUG

extern bool canBlinkGreenLed;

extern uint8_t redLedCurrent;
extern uint8_t irLedCurrent;

#endif /* VARS_H_ */
