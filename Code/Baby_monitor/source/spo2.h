/*
 * sp2.h
 *
 *  Created on: 13 de mai de 2019
 *      Author: dell-felipe
 */

#include <math.h>
#include <fsl_debug_console.h>
#include "vars.h"

#ifndef SPO2_H_
#define SPO2_H_

#define CALCULATE_EVERY_N_BEATS 4


bool spo2Calculator(float irACValue, float redACValue, bool beatDetected, uint8_t *spo2);
bool spo2Calculator2Method(float irACValue, float irDCValue, float redACValue, float redDCValue, bool beatDetected, uint8_t *spo2);
bool RCalculator(float irACValue, float redACValue, bool beatDetected, volatile float *R);

void resetVariables(void);

#endif /* SPO2_H_ */
