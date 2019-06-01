/*
 * process.h
 *
 *  Created on: 31 de mai de 2019
 *      Author: dell-felipe
 */

#ifndef PROCESS_H_
#define PROCESS_H_

#include "init.h"
#include "beat_detector.h"
#include "spo2.h"
#include "max30100/max30100.h"

#include <stdbool.h>



void initVariableToProcess(void);
bool processData(uint8_t* spo2, uint16_t* bpm_avg);

#endif /* PROCESS_H_ */
