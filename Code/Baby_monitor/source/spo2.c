/*
 * spo2.c
 *
 *  Created on: 13 de mai de 2019
 *      Author: dell-felipe
 */

#include "spo2.h"

static uint32_t samples_recorded = 0;
static uint32_t beat_detected_num = 0;
static float ir_AC_value_sq_sum = 0;
static float red_AC_value_sq_sum = 0;

// SaO2 Look-up Table
// http://www.ti.com/lit/an/slaa274b/slaa274b.pdf
const uint8_t spO2LUT[43] = {100,100,100,100,99,99,99,99,99,99,98,98,98,98,
                                             98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,
                                             95,95,95,95,94,94,94,94,94,93,93,93,93,93};

void resetVariables(void)
{
	samples_recorded = 0;
	beat_detected_num = 0;
	ir_AC_value_sq_sum = 0;
	red_AC_value_sq_sum = 0;
}

bool spo2Calculator(float irACValue, float redACValue, bool beatDetected, uint8_t *spo2)
{
	/*
	 * @brief  spo2Calculator
	 * @details     Using TI documentation:
	 * first - calculate rms value of each led.
	 * the value is mean of measure^2/number_of_samples
	 * Second - do log of rms of each led and dived then
	 * @param[in] float, float, bool
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 14 de mai de 2019
	 *
	 */

	ir_AC_value_sq_sum = ir_AC_value_sq_sum + powf(irACValue, 2);
	red_AC_value_sq_sum = red_AC_value_sq_sum+ powf(redACValue, 2);
	samples_recorded++;

	if (beatDetected)
	{
		beat_detected_num++;
		if (beat_detected_num > CALCULATE_EVERY_N_BEATS)
		{
			beat_detected_num = 0;
			float irRMS = ir_AC_value_sq_sum / samples_recorded;
			float redRMS = red_AC_value_sq_sum / samples_recorded;
			float ac_sq_ratio = 100.0 * logf(redRMS) / logf(irRMS);
			uint8_t index = 0;

			if (ac_sq_ratio > 66)
			{
				index = (uint8_t)ac_sq_ratio - 66;
			} else if (ac_sq_ratio > 50)
			{
				index = (uint8_t)ac_sq_ratio - 50;
			}
			resetVariables();
			if (index < 0 || index > 42)
			{
				return false;
			}
			*spo2 = spO2LUT[index];
			return true;
		}
	}
	return false;
}
