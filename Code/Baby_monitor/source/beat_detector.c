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

static uint32_t b_times[2] = { 0 };
static uint8_t last_p_fill = 0;
static bool isFullFill = false;
static bool isNeedMoreData = false;
static uint8_t min_index;
static uint8_t max_index;

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
	return false;
}

void initSimpleBeatDetector(struct simpleBeatDetector_t* simple_beat_detector_struct)
{
	simple_beat_detector_struct->state = SIMPLE_BEATDETECTOR_INIT;
	simple_beat_detector_struct->sample = 0;
	simple_beat_detector_struct->count = 0;

	b_times[0] = 0;
	b_times[1] = 0;

}

void reStartSimpleBeatDetector(struct simpleBeatDetector_t* simple_beat_detector_struct)
{
	simple_beat_detector_struct->state = SIMPLE_BEATDETECTOR_INIT;
	simple_beat_detector_struct->count = 1;
	simple_beat_detector_struct->sample = 0;

	b_times[0] = b_times[1];
}

bool checkForSimpleBeat(float sample, struct simpleBeatDetector_t* simple_beat_detector_struct, uint16_t* beat_result_p)
{

	uint16_t Bpm = 0;
	uint32_t beat_duration = 0;

#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
	char value[50] = { 0 };
#endif

	switch (simple_beat_detector_struct->state) {
	case SIMPLE_BEAT_DETECTOR_WAITING_STABLE:
		if (millis() > BEATDETECTOR_INVALID_READOUT_DELAY)
			simple_beat_detector_struct->state = SIMPLE_BEATDETECTOR_INIT;
		break;

	case SIMPLE_BEATDETECTOR_INIT:
		if (sample > 0 && simple_beat_detector_struct->sample < 0)
		{
			/*	Positive Zero Cross Detected	*/
			simple_beat_detector_struct->state = SIMPLE_BEATDETECTOR_WAITING_MAX;
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
			char aux[] = "State = SIMPLE_BEATDETECTOR_WAITING_MAX\r\n";
			USART_Printf(aux);
#endif
		}
		simple_beat_detector_struct->sample = sample;
		break;
	case SIMPLE_BEATDETECTOR_WAITING_MAX:
		/*	TODO Adjust in real time BEATDETECTOR_MAX_THRESHOLD	*/
		if (sample > BEATDETECTOR_MAX_THRESHOLD && sample < simple_beat_detector_struct->sample)
		{
			b_times[simple_beat_detector_struct->count] = millis();
			simple_beat_detector_struct->count++;
			simple_beat_detector_struct->state = SIMPLE_BEATDETECTOR_INIT;
			if (simple_beat_detector_struct->count >= 2)
			{
				simple_beat_detector_struct->state = SIMPLE_BEATDETECTOR_CALCULATION;
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
				char aux[] = "State = SIMPLE_BEATDETECTOR_CALCULATION\r\n";
				USART_Printf(aux);
#endif
			}
		}
		simple_beat_detector_struct->sample = sample;
		break;
	case SIMPLE_BEATDETECTOR_CALCULATION:

		beat_duration = b_times[1] - b_times[0];
		Bpm = 60000/beat_duration;
#ifdef MAX30100_BEAT_DETECTOR_OUTPUT
		sprintf(value, "b_[0]=%d b_[1]=%d Bpm=%d\r\n", b_times[0], b_times[1], Bpm);
		USART_Printf(value);
#endif
		reStartSimpleBeatDetector(simple_beat_detector_struct);

		if (Bpm > 30 && Bpm < 280)
		{
			*beat_result_p = Bpm;
			return true;
		}

		else
			return false;
		break;
	default:
		return false;
		break;
	}
	return false;
}

bool bpmAvgCalculator(uint16_t bpm, uint16_t* bpm_vector, uint16_t* bpm_avg)
{
	/*
	 * @brief  BpmAvgCalculator
	 * @details     Function that receives last calculated bpm value and put it in circular fifo
	 * with 5 positions, when all positions were be filled this function consider discard some data
	 * and calculate bpm average
	 * @param[in] uint16 bpm: last value of bpm calculated
	 * @param[in] uint16 bpm_vector: vector with last bpm values
	 * @param[out]
	 * @return uint16_t bpm avg
	 *
	 * author dell-felipe
	 * date 5 de mai de 2019
	 *
	 */

	uint16_t minimum;
	uint16_t maximum;
	float sum = 0;
	float avg = 0;
	float variance = 0;
	uint8_t i;
	static uint8_t index_to_fill = 0;

	/* Verify if all five positions was filled, if yes, reset last_p_fill	*/
	if (last_p_fill > BPM_VECTOR_SIZE - 1 )
	{
		last_p_fill = 0;
		isFullFill = true;
	}

	/*	insert new bpm in last_p_fill position if no more data is needed	*/
	if (!isNeedMoreData)
	{
		bpm_vector[last_p_fill] = bpm;
		last_p_fill++;
	}

	/*	if more data is needed insert it when is needed	*/
	else
	{
		if (index_to_fill == 0)
		{
			bpm_vector[min_index] = bpm;
			index_to_fill++;
		}
		if (index_to_fill >= 1)
		{
			bpm_vector[max_index] = bpm;
			index_to_fill = 0;
			isNeedMoreData = false;
		}
	}

	if(isFullFill && !isNeedMoreData)
	{
		minimum = bpm_vector[0];
		maximum = bpm_vector[0];
		min_index = 0;
		max_index = 0;

		/*	calculate average of vector	*/
		for (i = 0; i < BPM_VECTOR_SIZE; i++)
		{
			sum += bpm_vector[i];
		}

		avg = sum / (float) BPM_VECTOR_SIZE;

		/*	calculate standard deviation of vector	*/
		sum = 0;
		for (i = 0; i < BPM_VECTOR_SIZE; i++)
		{
			sum += pow((bpm_vector[i] - avg), 2);
		}
		variance = sqrtf(sum / (float) BPM_VECTOR_SIZE);
		if (variance < 2.5)
		{
			*bpm_avg =  (int) avg;
			return true;
		}
		else
		{
			/*	remove minimum and maximum value in vector	*/
			/* 1. identify index of min and max values in vector */
			for (i=0; i < BPM_VECTOR_SIZE; i++)
			{
				if (bpm_vector[i] < minimum)
				{
					minimum = bpm_vector[i];
					min_index = i;
				}
				if (bpm_vector[i] > maximum)
				{
					maximum = bpm_vector[i];
					max_index = i;
				}
			}
			/*	clear position in vector with disparate values	*/
			bpm_vector[min_index] = 0;
			bpm_vector[max_index] = 0;
			isNeedMoreData = true;
		}
	}
	return false;
}






