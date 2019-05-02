/*
 * beat_detector.h
 *
 *  Created on: 30 de abr de 2019
 *      Author: dell-felipe
 */

#ifndef BEAT_DETECTOR_H_
#define BEAT_DETECTOR_H_

#define ALPHA 0.95
#define MEAN_FILTER_SIZE 25

#define BEATDETECTOR_INIT_HOLDOFF                2000    // in ms, how long to wait before counting
#define BEATDETECTOR_MASKING_HOLDOFF             200     // in ms, non-retriggerable window after beat detection
#define BEATDETECTOR_BPFILTER_ALPHA              0.6     // EMA factor for the beat period value
#define BEATDETECTOR_MIN_THRESHOLD               20      // minimum threshold (filtered) value
#define BEATDETECTOR_MAX_THRESHOLD               800     // maximum threshold (filtered) value
#define BEATDETECTOR_STEP_RESILIENCY             30      // maximum negative jump that triggers the beat edge
#define BEATDETECTOR_THRESHOLD_FALLOFF_TARGET    0.3     // thr chasing factor of the max value when beat
#define BEATDETECTOR_THRESHOLD_DECAY_FACTOR      0.99    // thr chasing factor when no beat
#define BEATDETECTOR_INVALID_READOUT_DELAY       2000    // in ms, no-beat time to cause a reset
#define BEATDETECTOR_SAMPLES_PERIOD              10      // in ms, 1/Fs


typedef enum BeatDetectorState_ {
    BEATDETECTOR_STATE_INIT,
    BEATDETECTOR_STATE_WAITING,
    BEATDETECTOR_STATE_FOLLOWING_SLOPE,
    BEATDETECTOR_STATE_MAYBE_DETECTED,
    BEATDETECTOR_STATE_MASKING
} BeatDetectorState_;

//extern enum BeatDetectorState_ BeatDetectorState;

struct fifo_t{
	short unsigned int rawIR;
	short unsigned int rawRed;
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
	unsigned short int index;
	float sum;
	unsigned short int count;
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
	unsigned short int state;
	unsigned short int threshold;
	float beatPeriod;
	float lastMaxValue;
	unsigned int tsLastBeat;
};

extern struct beatDetector_t beat_detector_t;

/*	Functions declaration	*/
struct dcFilter_t dcRemoval(float x, float prev_w, float alpha);
void dcFilterClear(struct dcFilter_t* dcFilter_t_);
float meanDiff(float M, struct meanDiffFilter_t* filterValues);
void meanDiffFilterClear(struct meanDiffFilter_t* filterValues);
void lowPassFilter(float x, struct butterworthFilter_t* filterResult);
void initBeatDetector(struct beatDetector_t* beatDetectorStruct);
unsigned short addSample(float sample);
unsigned short checkForBeat(float sample, struct beatDetector_t* beatDetectorStruct, volatile unsigned long millis);

#endif /* BEAT_DETECTOR_H_ */
