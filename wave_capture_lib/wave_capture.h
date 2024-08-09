

#ifndef _WAVE_CAPTURE_H_
#define _WAVE_CAPTURE_H_

#include <stdint.h>

/* return number of read/written bytes */

typedef int (*WaveCapture_SerialWrite_f)(const uint8_t *buf, int length);


typedef enum
{
	WAVECAPTURE_TYPE_FLOAT = 0,
	WAVECAPTURE_TYPE_INT32 = 1,
	WAVECAPTURE_TYPE_INT16 = 2,
}WaveCapture_Type_e;

typedef enum
{
	WAVECAPTURE_TRIG_SLOPE_RISE = 1,
	WAVECAPTURE_TRIG_SLOPE_FALL = 2,
}WaveCaptureTrigSlope_e;

typedef enum
{
	WAVECAPTURE_MODE_AUTO = 0,
	WAVECAPTURE_MODE_NORMAL = 1,
	WAVECAPTURE_MODE_SINGLE = 2,
}WaveCapture_Mode_e;

typedef enum
{
	WAVECAPTURE_SAMPLE_FREERUN,
	WAVECAPTURE_SAMPLE_TOTRIG,
	WAVECAPTURE_SAMPLE_TOEND,
	WAVECAPTURE_SAMPLE_STOPPED,
}WaveCapture_SamplingStatus_e;

typedef struct
{
	char* name;
	WaveCapture_Type_e type;
	void* var_ptr;
}WaveCapture_Init_ChannelInfo_t;

typedef struct
{
	WaveCapture_SerialWrite_f func_write;
	uint32_t sampling_length;
	float sampling_freq;
	uint32_t channel_num;
	WaveCapture_Init_ChannelInfo_t* ch_info_array;
}WaveCapture_Init_t;

typedef struct
{
	WaveCapture_Init_t init;
	uint32_t ch_select;
	float trig_level_f;
	int trig_level_i;
	uint32_t trig_ch;
	int32_t trig_pos;
	WaveCaptureTrigSlope_e trig_slope;
	WaveCapture_Mode_e trig_mode;
	uint32_t decimate;
	void** wavedata;
	uint32_t cursor;
	uint32_t cursor_prev;
	uint32_t cursor_trig;
	uint32_t cursor_end;
	uint32_t timeout;
	uint32_t timeout_count;
	uint32_t presample_count;
	uint32_t decimate_counter;
	WaveCapture_SamplingStatus_e status;
}WaveCapture_t;


int WaveCapture_Init(WaveCapture_t *h, WaveCapture_Init_t *init);

void WaveCapture_Sampling(WaveCapture_t *h);

int WaveCapture_Set_Channel(WaveCapture_t *h, uint32_t channel_select);

int WaveCapture_Set_TriggerLevel(WaveCapture_t *h, float trig_level_f, int trig_level_i);

int WaveCapture_Set_TriggerChannel(WaveCapture_t *h, uint32_t trig_channel);

int WaveCapture_Set_TriggerPos(WaveCapture_t *h, int32_t trig_pos);

int WaveCapture_Set_TriggerEdgeSlope(WaveCapture_t *h, WaveCaptureTrigSlope_e slope);

int WaveCapture_Set_TriggerMode(WaveCapture_t *h, WaveCapture_Mode_e mode);

int WaveCapture_Set_Decimate(WaveCapture_t *h, uint32_t decimate);

int WaveCapture_Set_Timeout(WaveCapture_t *h, uint32_t timeout);

int WaveCapture_Start_Sampling(WaveCapture_t *h);

int WaveCapture_Get_WaveForm(WaveCapture_t *h);

int WaveCapture_Get_WaveInfo(WaveCapture_t *h);

void WaveCapture_Dispose(WaveCapture_t *h);



#endif /* _WAVE_CAPTURE_H_ */
