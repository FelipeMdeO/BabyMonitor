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

extern volatile uint32_t samples_recorded;
extern uint32_t beat_detected_num;
extern float ir_AC_value_sq_sum;
extern float red_AC_value_sq_sum;

extern bool canBlinkGreenLed;

#endif /* VARS_H_ */
