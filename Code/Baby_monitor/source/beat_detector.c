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

void initBeatDetector(struct beatDetector_t* beatDetectorStruct)
{
	beatDetectorStruct->state = BEATDETECTOR_STATE_INIT;
	beatDetectorStruct->threshold = BEATDETECTOR_MIN_THRESHOLD;
	beatDetectorStruct->beatPeriod = 0;
	beatDetectorStruct->lastMaxValue = 0;
	beatDetectorStruct->tsLastBeat = 0;
}

unsigned short checkForBeat(float sample, struct beatDetector_t* beatDetectorStruct, volatile unsigned long millis)
{
	/*	TODO change millis to pointer to have updated values	*/
	unsigned short beatDetected = false;

	switch(beatDetectorStruct->state) {
	case BEATDETECTOR_STATE_INIT:
		if(millis >= BEATDETECTOR_INIT_HOLDOFF)
		{
			beatDetectorStruct->state = BEATDETECTOR_STATE_WAITING;
		}
		break;

	case BEATDETECTOR_STATE_WAITING:
		if(sample > beatDetectorStruct->threshold)
		{
			beatDetectorStruct->threshold = min(sample, BEATDETECTOR_MAX_THRESHOLD);
			beatDetectorStruct->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
		}
		/*	Tracking lost, reseting	*/
		if(millis - beatDetectorStruct->tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY)
		{
			beatDetectorStruct->beatPeriod = 0;
			beatDetectorStruct->lastMaxValue = 0;
		}
		/*	TODO implement function decreaseThreshold	*/
		//	decreaseThreshould();
		break;
	}
	return 1;
}








