

#ifndef _CURRENT_SENSOR_H_
#define _CURRENT_SENSOR_H_


#include "adc.h"


#define MEDIAN_ORDER	5



typedef enum
{
	CS_Type_3shunt,
	CS_Type_1shunt,
	CS_Type_DCCT,

}CurrentSensor_Type;



typedef struct
{

	CurrentSensor_Type CS_Type;

	ADC_HandleTypeDef *hadc_Iu;
	ADC_HandleTypeDef *hadc_Iv;
	ADC_HandleTypeDef *hadc_Iw;
	ADC_HandleTypeDef *hadc_Idc;

	float Iu_Gain;
	float Iv_Gain;
	float Iw_Gain;

	float V_Iu_offset;
	float V_Iv_offset;
	float V_Iw_offset;

	float V_Idc_offset;


}CurrentSensor_InitTypeDef;



typedef struct
{

	CurrentSensor_InitTypeDef Init;

	uint16_t AD_Iu[1];
	uint16_t AD_Iv[1];
	uint16_t AD_Iw[1];

	int32_t pos_MEDF_I;

	int32_t AD_Iu_buf[MEDIAN_ORDER];
	int32_t AD_Iv_buf[MEDIAN_ORDER];
	int32_t AD_Iw_buf[MEDIAN_ORDER];

	float V_Iu, V_Iv, V_Iw;

	float Iu, Iv, Iw;

}CurrentSensor_TypeDef;



extern CurrentSensor_TypeDef mainCS;




void CurrentSensor_Init();

void CurrentSensor_Start(CurrentSensor_TypeDef *hCS);


/*
 * UVWの電流値を更新するだけ
 * 座標変換とかはやらない
 */
void CurrentSensor_Refresh(CurrentSensor_TypeDef *hCS, uint8_t SVM_sector);


/*
 * 回転座標系における電流を算出
 */
void CurrentSensor_getIdq(CurrentSensor_TypeDef *hCS, float *Id, float *Iq, float cos_theta_re, float sin_theta_re);




#endif /* _CURRENT_SENSOR_H_ */


