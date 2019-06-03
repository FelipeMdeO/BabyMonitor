/*
 * spo2.c
 *
 *  Created on: 13 de mai de 2019
 *      Author: dell-felipe
 */

#include "spo2.h"

uint32_t volatile samples_recorded = 0;
uint32_t beat_detected_num = 0;
float ir_AC_value_sq_sum = 0;
float red_AC_value_sq_sum = 0;


void resetVariables(void)
{
	samples_recorded = 0;
	beat_detected_num = 0;
	ir_AC_value_sq_sum = 0;
	red_AC_value_sq_sum = 0;
}

bool spo2Calculator2Method
(
		float irACValue,
		float irDCValue,
		float redACValue,
		float redDCValue,
		bool beatDetected,
		uint8_t *spo2
)
{

	volatile float R = 0;
	if (beatDetected) {
		R = (redACValue/redDCValue)/(irACValue/irDCValue);
		*spo2 = 104-17*R;
		if (*spo2 > 0 && *spo2 < 100)
			return true;
	}
	return false;

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
			volatile float ac_sq_ratio = logf(redRMS) / logf(irRMS);
			*spo2 = (uint8_t)(110 - 25*ac_sq_ratio) + 10;

			if (*spo2 > 0 && *spo2 < 100) {
				resetVariables();
				return true;
			}
		}
	}
	return false;
}

bool RCalculator(float irACValue, float redACValue, bool beatDetected, volatile float *R)
{
	/*
	 * @brief  RCalculator
	 * @details     Using TI documentation:
	 * first - calculate rms value of each led.
	 * the value is mean of measure^2/number_of_samples
	 * Second - do log of rms of each led and dived then
	 * @param[in] float, float, bool
	 * @param[out]
	 * @return void
	 *
	 * author dell-felipe
	 * date 02 de jun de 2019
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
			*R = logf(redRMS) / logf(irRMS);

			if (*R > 0 && *R < 3.5) {
				resetVariables();
				return true;
			}
		}
	}
	return false;
}
