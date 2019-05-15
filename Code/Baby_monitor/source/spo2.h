/*
 * sp2.h
 *
 *  Created on: 13 de mai de 2019
 *      Author: dell-felipe
 */

#include <math.h>
#include <fsl_debug_console.h>

#ifndef SPO2_H_
#define SPO2_H_

#define CALCULATE_EVERY_N_BEATS 4

bool spo2Calculator(float irACValue, float redACValue, bool beatDetected, uint8_t *spo2);

void resetVariables(void);

#endif /* SPO2_H_ */
