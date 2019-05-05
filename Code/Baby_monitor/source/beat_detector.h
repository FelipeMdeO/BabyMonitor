/*
 * beat_detector.h
 *
 *  Created on: 30 de abr de 2019
 *      Author: dell-felipe
 */

#include <math.h>
#include <fsl_debug_console.h>
#include "init.h"

#ifndef BEAT_DETECTOR_H_
#define BEAT_DETECTOR_H_

#define ALPHA 0.95
#define MEAN_FILTER_SIZE 25

// Debug defines
// #define MAX30100_BEAT_DETECTOR_OUTPUT

#define BEATDETECTOR_INIT_HOLDOFF					2000    // in ms, how long to wait before counting
#define BEATDETECTOR_MASKING_HOLDOFF				150     // in ms, non-retriggerable window after beat detection
#define BEATDETECTOR_BPFILTER_ALPHA					0.6     // EMA factor for the beat period value
#define BEATDETECTOR_MIN_THRESHOLD					20      // minimum threshold (filtered) value
#define BEATDETECTOR_MAX_THRESHOLD					350     // maximum threshold (filtered) value
#define BEATDETECTOR_STEP_RESILIENCY				30      // maximum negative jump that triggers the beat edge
#define BEATDETECTOR_THRESHOLD_FALLOFF_TARGET		0.3     // thr chasing factor of the max value when beat
#define BEATDETECTOR_THRESHOLD_DECAY_FACTOR			0.99    // thr chasing factor when no beat
#define BEATDETECTOR_INVALID_READOUT_DELAY			2000    // in ms, no-beat time to cause a reset
#define BEATDETECTOR_SAMPLES_PERIOD					10      // in ms, 1/Fs
#define BPM_VECTOR_SIZE								5

typedef enum BeatDetectorState_ {
    BEATDETECTOR_STATE_INIT,
	BEATDETECTOR_STATE_WAINTING_STABLE,
    BEATDETECTOR_STATE_WAITING_POSITIVE,
	BEAT_DETECTOR_STATE_FOLLOWING_SLOPE,
    BEATDETECTOR_STATE_WAITING_NEGATIVE,
    BEATDETECTOR_STATE_DETECTED,
    BEATDETECTOR_STATE_ERROR
} BeatDetectorState_;

typedef enum SimpleBeatDetectorState_ {
	SIMPLE_BEAT_DETECTOR_WAITING_STABLE,
	SIMPLE_BEATDETECTOR_INIT,
	SIMPLE_BEATDETECTOR_WAITING_MAX,
	SIMPLE_BEATDETECTOR_CALCULATION
} SimpleBeatDetectorState_;

//extern enum BeatDetectorState_ BeatDetectorState;

struct fifo_t{
	uint16_t rawIR;
	uint16_t rawRed;
};

extern struct fifo_t result;

struct dcFilter_t {
	float w;
	float result;
};

extern struct dcFilter_t dcFilterIR;
extern struct dcFilter_t dcFilterRed;

struct meanDiffFilter_t
{
	float values[MEAN_FILTER_SIZE];
	uint8_t index;
	float sum;
	uint8_t count;
};

extern struct meanDiffFilter_t meanDiffIR;

struct butterworthFilter_t
{
	float v[2];
	float result;
};

extern struct butterworthFilter_t filter;

struct beatDetector_t
{
	uint8_t state;
	float lsSample;
	float threshold;
	float beatPeriod;
	float lastMaxValue;
	uint32_t tsLastBeat;
	uint8_t count;
};

extern struct beatDetector_t beat_detector_t;

struct simpleBeatDetector_t
{
	uint8_t state;
	float sample;
	uint8_t count;
};

extern struct simpleBeatDetector_t simple_beat_detector_t;

/*	vector size of bpm values	*/
extern uint16_t bpm_v[BPM_VECTOR_SIZE];

/*	Functions declaration	*/
struct dcFilter_t dcRemoval(float x, float prev_w, float alpha);
void dcFilterClear(struct dcFilter_t* dcFilter_t_);
float meanDiff(float M, struct meanDiffFilter_t* filterValues);
void meanDiffFilterClear(struct meanDiffFilter_t* filterValues);
void lowPassFilter(float x, struct butterworthFilter_t* filterResult);
void initBeatDetector(struct beatDetector_t* pt_beat_detector);
bool addSample(float sample);
bool checkForBeat(float sample, struct beatDetector_t* beatDetectorStruct);	/*	TODO Correct param name of this function to standard	*/
void initSimpleBeatDetector(struct simpleBeatDetector_t* simple_beat_detector_struct);
bool checkForSimpleBeat(float sample, struct simpleBeatDetector_t* simple_beat_detector_struct, uint16_t* beat_result_p);
bool bpmAvgCalculator(uint16_t bpm, uint16_t* bpm_vector, uint16_t* bpm_avg);

#endif /* BEAT_DETECTOR_H_ */
