/*
 * process.c
 *
 *  Created on: 31 de mai de 2019
 *      Author: dell-felipe
 */

#include "process.h"

bool isValidSample = false;
struct fifo_t sample;
struct dcFilter_t acFilterIR;
struct dcFilter_t acFilterRed;
struct meanDiffFilter_t meanDiffIR;
struct butterworthFilter_t filter;
struct simpleBeatDetector_t beat_detector_t;

static bool isBeatDetected = false;
static uint16_t beat_result = 0;
static uint16_t bmp_v[BPM_VECTOR_SIZE] = { 0 };
static bool isConfiableOutput = false;
//uint16_t bpm_avg = 0;
//uint8_t spo2 = 0;
static bool isSpo2Ready = false;

static bool canCalculateSpo2 = true;
static bool canCalculateBPM = true;

void initVariableToProcess(void) {
	/*	Clean up structs to start filter process	*/
	dcFilterClear(&acFilterIR);
	dcFilterClear(&acFilterRed);
	meanDiffFilterClear(&meanDiffIR);
	initSimpleBeatDetector(&beat_detector_t);
	beat_detector_t.state = SIMPLE_BEAT_DETECTOR_WAITING_STABLE;

	//	isBeatDetected = false;
	//	beat_result = 0;
	memset(bmp_v, 0, BPM_VECTOR_SIZE);
	isConfiableOutput = false;
	isSpo2Ready = false;
	canCalculateSpo2 = true;
	canCalculateBPM = true;

	init_tick();
}

bool processData(uint8_t* spo2, uint16_t* bpm_avg)
{
	isValidSample = MAX30100_Get_Sample(&sample.rawIR, &sample.rawRed);
	if( isValidSample )
	{
		acFilterIR = dcRemoval((float)sample.rawIR, acFilterIR.w, ALPHA);
		acFilterRed = dcRemoval((float)sample.rawRed, acFilterRed.w, ALPHA);
		float meanDiffResIR = meanDiff(acFilterIR.result, &meanDiffIR);
		/*	IF mean vector was fully filed	*/
		if (meanDiffIR.count >= MEAN_FILTER_SIZE)
		{
			/*	toggle a pin here if you want test loop performance	*/
			/*	GPIO_PortToggle(GPIOB, 1u << 8U);	*/

			/*	low pass filter implementation	*/
			lowPassFilter(meanDiffResIR, &filter);

#ifdef MAX30100_FILTERED_RAW_OUTPUT
			sprintf(value, "%d\t", (int)filter.result);
			USART_Printf(value);
			sprintf(value, "%d\r\n", (int)acFilterRed.result);
			USART_Printf(value);
#endif

#ifndef MAX30100_FIFO_RAW_OUTPUT
			/*	Beat Detector Algorithm	*/
			isBeatDetected = checkForSimpleBeat(filter.result, &beat_detector_t, &beat_result);
			if (isBeatDetected)
			{
				canBlinkGreenLed = false;
				BEAT_LED(); /*	indicate beat signal using red led	*/
				// todo implementar aqui uma funcao que faz a verificacao de diferenca dc entre os leds.
				if (canCalculateSpo2)
				{
					isSpo2Ready = spo2Calculator(acFilterIR.result, acFilterRed.result, isBeatDetected, spo2);
					if(isSpo2Ready)
						canCalculateSpo2 = false;
				}
				isBeatDetected = false;
				if (canCalculateBPM)
				{
					isConfiableOutput = bpmAvgCalculator(beat_result, bmp_v, bpm_avg);
					if (isConfiableOutput)
						canCalculateBPM = false;
				}

				if (!canCalculateSpo2 && !canCalculateBPM)
				{
					isConfiableOutput = false;
					isSpo2Ready = false;
					canCalculateBPM = true;
					canCalculateSpo2 = true;
					//					sprintf(beat_text, "bpm_avg = %d \t sp02 = %d \r\n", bpm_avg, spo2);
					//					sprintf(beat_text, "%d / %d", spo2, bpm_avg);
					//					USART_Printf(beat_text);

					return true;
				}

				//					sprintf(beat_text, "beat_result = %d\r\n", beat_result);
				//					USART_Printf(beat_text);
			}
#endif
		}
	}
#ifdef MAX30100_FIFO_RAW_OUTPUT
	canPrint = readFIFO(&irRaw, &redRaw);
#endif

	return false;
}

bool processR(volatile float *R)
{

	isValidSample = MAX30100_Get_Sample(&sample.rawIR, &sample.rawRed);
	if( isValidSample )
	{
		acFilterIR = dcRemoval((float)sample.rawIR, acFilterIR.w, ALPHA);
		acFilterRed = dcRemoval((float)sample.rawRed, acFilterRed.w, ALPHA);
		float meanDiffResIR = meanDiff(acFilterIR.result, &meanDiffIR);
		/*	IF mean vector was fully filed	*/
		if (meanDiffIR.count >= MEAN_FILTER_SIZE)
		{
			/*	low pass filter implementation	*/
			lowPassFilter(meanDiffResIR, &filter);

			/*	Beat Detector Algorithm	*/
			isBeatDetected = checkForSimpleBeat(filter.result, &beat_detector_t, &beat_result);
			if (isBeatDetected)
			{
				canBlinkGreenLed = false;
				BEAT_LED(); /*	indicate beat signal using red led	*/
				// todo implementar aqui uma funcao que faz a verificacao de diferenca dc entre os leds.
				isSpo2Ready = RCalculator(acFilterIR.result, acFilterRed.result, isBeatDetected, R);
				isBeatDetected = false;
			}
		}
	}
	if (isSpo2Ready) {
		initVariableToProcess();
		return true;
	}

	return false;
}

bool processIRData(float *ir) {
	isValidSample = MAX30100_Get_Sample(&sample.rawIR, &sample.rawRed);
	if( isValidSample )
	{
		acFilterIR = dcRemoval((float)sample.rawIR, acFilterIR.w, ALPHA);
		float meanDiffResIR = meanDiff(acFilterIR.result, &meanDiffIR);
		/*	IF mean vector was fully filed	*/
		if (meanDiffIR.count >= MEAN_FILTER_SIZE)
		{
			/*	toggle a pin here if you want test loop performance	*/
			/*	GPIO_PortToggle(GPIOB, 1u << 8U);	*/

			/*	low pass filter implementation	*/
			lowPassFilter(meanDiffResIR, &filter);
			*ir = filter.result;
		}
	}
}
