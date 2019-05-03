/*
 * beat_detector.c
 *
 *  Created on: 30 de abr de 2019
 *      Author: dell-felipe
 */

#include "beat_detector.h"

#ifndef false
#define false 0
#endif
#ifndef true
#define true 1
#endif

#ifndef min
#define min(a,b) \
		({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a < _b ? _a : _b; })
#endif

//#define MAX30100_BEAT_DETECTOR_OUTPUT

struct dcFilter_t dcRemoval(float x, float prev_w, float alpha)
{
	struct dcFilter_t filtered;
	filtered.w = x + alpha * prev_w;
	filtered.result = filtered.w - prev_w;

	return filtered;
}

void dcFilterClear(struct dcFilter_t* dcFilter_t_)
{
	dcFilter_t_->result = 0;
	dcFilter_t_->w = 0;
}

void meanDiffFilterClear(struct meanDiffFilter_t* filterValues)
{
	for(int i=0; i<MEAN_FILTER_SIZE; i++)
	{
		filterValues->values[i] = 0;
	}
	filterValues->index = 0;
	filterValues->sum = 0;
	filterValues->count = 0;
}

float meanDiff(float M, struct meanDiffFilter_t* filterValues)
{
	float avg = 0;

	filterValues->sum -= filterValues->values[filterValues->index];
	filterValues->values[filterValues->index] = M;
	filterValues->sum += filterValues->values[filterValues->index];

	filterValues->index++;
	filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

	if (filterValues->count < MEAN_FILTER_SIZE)
		filterValues->count++;

	avg = filterValues->sum / filterValues->count;

	return avg - M;
}

void lowPassFilter(float x, struct butterworthFilter_t* filterResult)
{
	filterResult->v[0] = filterResult->v[1];

	/*	Fs = 100 Hz and Fc = 10 Hz	*/
	/*	filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);	*/

	/*	Fs = 100Hz and Fc = 4Hz	*/
	filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter

	filterResult->result = filterResult->v[0] + filterResult->v[1];
}

void initBeatDetector(struct beatDetector_t* pt_beat_detector)
{
	pt_beat_detector->state = BEATDETECTOR_STATE_INIT;
	pt_beat_detector->threshold = BEATDETECTOR_MIN_THRESHOLD;
	pt_beat_detector->lsSample = 0;
	pt_beat_detector->beatPeriod = 0;
	pt_beat_detector->lastMaxValue = 0;
	pt_beat_detector->tsLastBeat = 0;
	pt_beat_detector->count = 0;
}

bool checkForBeat(float sample, struct beatDetector_t* pt_beat_detector)
{

	switch(pt_beat_detector->state) {

	case BEATDETECTOR_STATE_INIT:
		initBeatDetector(pt_beat_detector);
		/*	Wait for startup sensor	*/
		if (sample < 600 && sample > -400)
		{
			pt_beat_detector->count++;
			if ( pt_beat_detector->count > 20)
			{
				pt_beat_detector->state = BEATDETECTOR_STATE_WAINTING_STABLE;
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
			char value[] = "State = BEATDETECTOR_STATE_WAINTING_STABLE";
			USART_Printf(value);
#endif
			}
		}
		break;
	case BEATDETECTOR_STATE_WAINTING_STABLE:
		/*	TODO find method to garanty that the millis() will be clear before checkForBeat was called for first time	*/
		if (millis() > BEATDETECTOR_INIT_HOLDOFF)
		{
			pt_beat_detector->state = BEATDETECTOR_STATE_WAITING_POSITIVE;
			pt_beat_detector->lsSample = sample;
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
			char value[] = "State = BEATDETECTOR_STATE_WAITING_POSITIVE";
			USART_Printf(value);
#endif
		}
		break;
	case BEATDETECTOR_STATE_WAITING_POSITIVE:
		if (pt_beat_detector->lsSample < 0 && sample > 0)
		{
			pt_beat_detector->state = BEATDETECTOR_STATE_WAITING_NEGATIVE;
			pt_beat_detector->tsLastBeat = millis();
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
			char value[] = "State = BEATDETECTOR_STATE_WAITING_NEGATIVE";
			USART_Printf(value);
#endif
		}
		pt_beat_detector->lsSample = sample;
		break;
	case BEATDETECTOR_STATE_WAITING_NEGATIVE:
		if (pt_beat_detector->lsSample > 0 && sample < 0 )
		{
			if (millis() - pt_beat_detector->tsLastBeat > 100)	/*	10 samples * 10 ms per sample = 100 ms	*/
			{
				pt_beat_detector->state = BEATDETECTOR_STATE_DETECTED;
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
			char value[] = "State = BEATDETECTOR_STATE_DETECTED";
			USART_Printf(value);
#endif
			}
			else
			{
				pt_beat_detector->state = BEATDETECTOR_STATE_ERROR;
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
			char value[] = "State = BEATDETECTOR_STATE_ERROR";
			USART_Printf(value);
#endif
			}

		}
		pt_beat_detector->lsSample = sample;
		break;
	case BEATDETECTOR_STATE_DETECTED:
		initBeatDetector(pt_beat_detector);
		pt_beat_detector->state = BEATDETECTOR_STATE_WAITING_POSITIVE;
		return true;
	case BEATDETECTOR_STATE_ERROR:
		initBeatDetector(pt_beat_detector);
		pt_beat_detector->state = BEATDETECTOR_STATE_WAITING_POSITIVE;
		return false;
	default:
		return false;
		break;
	}
}

