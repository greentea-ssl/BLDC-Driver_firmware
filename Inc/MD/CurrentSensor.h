

#ifndef _CURRENT_SENSOR_H_
#define _CURRENT_SENSOR_H_


#include "main.h"


#define MEDIAN_ORDER	1



typedef enum
{
	CS_Type_3shunt,
	CS_Type_1shunt,
	CS_Type_DCCT,

}CurrentSensor_Type;



typedef struct
{

	CurrentSensor_Type CS_Type;

	ADC_HandleTypeDef *hadc[3];

	float Iu_Gain;
	float Iv_Gain;
	float Iw_Gain;
	float Vdc_Gain;

	float V_Iu_offset;
	float V_Iv_offset;
	float V_Iw_offset;
	float V_Vdc_offset;

}CurrentSensor_InitTypeDef;



typedef struct
{

	CurrentSensor_InitTypeDef Init;

	volatile uint16_t AD_Iu[1];
	volatile uint16_t AD_Iv[1];
	volatile uint16_t AD_Iw[1];
	volatile uint16_t AD_Vdc[1];

	int32_t pos_MEDF_I;

	int32_t AD_Iu_buf[MEDIAN_ORDER];
	int32_t AD_Iv_buf[MEDIAN_ORDER];
	int32_t AD_Iw_buf[MEDIAN_ORDER];
	int32_t AD_Vdc_buf[MEDIAN_ORDER];

	float V_Iu, V_Iv, V_Iw, V_Vdc;

	float Iu, Iv, Iw, Vdc;

}CurrentSensor_TypeDef;



extern CurrentSensor_TypeDef mainCS;




void CurrentSensor_Init();

void CurrentSensor_Start(CurrentSensor_TypeDef *hCS);


/*
 * UVWの電流値を更新するだけ
 * 座標変換とかはやらない
 */
void CurrentSensor_Refresh(CurrentSensor_TypeDef *hCS);


/*
 * 回転座標系における電流を算出
 */
void CurrentSensor_getIdq(CurrentSensor_TypeDef *hCS, float *Id, float *Iq, float cos_theta_re, float sin_theta_re);




#endif /* _CURRENT_SENSOR_H_ */



