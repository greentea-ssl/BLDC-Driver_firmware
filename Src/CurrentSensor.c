

#include "CurrentSensor.h"

#include "string.h"
#include "stdlib.h"


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

const float Vref_AD = 3.3f;

const int32_t AD_Range = 4096;


CurrentSensor_TypeDef mainCS;


int32_t median3(int32_t *buf);

int32_t median5(int32_t *buf);


void CurrentSensor_Init()
{

	memset(&mainCS, 0x00, sizeof(mainCS));

	mainCS.Init.CS_Type = CS_Type_3shunt;
	mainCS.Init.Iu_Gain = -10.0f;//9.0f;	// 1 / ( R * OPAmpGain) [A / V]
	mainCS.Init.Iv_Gain = -10.0f;//5.3f;	// 1 / ( R * OPAmpGain) [A / V]
	mainCS.Init.Iw_Gain = -10.0f;//5.5f;	// 1 / ( R * OPAmpGain) [A / V]
	mainCS.Init.Iw_Gain = -10.0f;//5.5f;	// 1 / ( R * OPAmpGain) [A / V]
	mainCS.Init.Vdc_Gain = 12.5385f; //[V / V]

	mainCS.Init.V_Iu_offset = 1.65;
	mainCS.Init.V_Iv_offset = 1.65;
	mainCS.Init.V_Iw_offset = 1.65;
	mainCS.Init.V_Vdc_offset = 0.0;

	mainCS.Init.hadc[0] = &hadc1;
	mainCS.Init.hadc[1] = &hadc2;
	mainCS.Init.hadc[2] = &hadc3;

	mainCS.pos_MEDF_I = 0;

}


void CurrentSensor_Start(CurrentSensor_TypeDef *hCS)
{

	HAL_ADCEx_InjectedStart_IT(hCS->Init.hadc[0]);
	HAL_ADCEx_InjectedStart_IT(hCS->Init.hadc[1]);
	HAL_ADCEx_InjectedStart_IT(hCS->Init.hadc[2]);

}


/*
 * UVWの電流値を更新するだけ
 * 座標変換とかはやらない
 */
inline void CurrentSensor_Refresh(CurrentSensor_TypeDef *hCS)
{

	static int32_t AD_Iu_MEDF = 0;
	static int32_t AD_Iv_MEDF = 0;
	static int32_t AD_Iw_MEDF = 0;
	static int32_t AD_Vdc_MEDF = 0;

	static CurrentSensor_InitTypeDef *hCS_Init;


	hCS_Init = &hCS->Init;


	hCS->AD_Iu[0] = hCS->Init.hadc[0]->Instance->JDR1;
	hCS->AD_Iv[0] = hCS->Init.hadc[1]->Instance->JDR1;
	hCS->AD_Iw[0] = hCS->Init.hadc[2]->Instance->JDR1;
	hCS->AD_Vdc[0] = hCS->Init.hadc[0]->Instance->JDR2;

	hCS->AD_Iu_buf[hCS->pos_MEDF_I] = (int32_t)hCS->AD_Iu[0];
	hCS->AD_Iv_buf[hCS->pos_MEDF_I] = (int32_t)hCS->AD_Iv[0];
	hCS->AD_Iw_buf[hCS->pos_MEDF_I] = (int32_t)hCS->AD_Iw[0];
	hCS->AD_Vdc_buf[hCS->pos_MEDF_I] = (int32_t)hCS->AD_Vdc[0];

	// メディアンフィルタ用バッファ書き込み位置更新
	hCS->pos_MEDF_I += 1;
	if(hCS->pos_MEDF_I >= MEDIAN_ORDER)
	{
		hCS->pos_MEDF_I = 0;
	}

	AD_Iu_MEDF = hCS->AD_Iu[0];
	AD_Iv_MEDF = hCS->AD_Iv[0];
	AD_Iw_MEDF = hCS->AD_Iw[0];

	AD_Vdc_MEDF = hCS->AD_Vdc[0];

	// 端子電圧更新
	hCS->V_Iu = (float)AD_Iu_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iu_offset;
	hCS->V_Iv = (float)AD_Iv_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iv_offset;
	hCS->V_Iw = (float)AD_Iw_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iw_offset;
	hCS->V_Vdc = (float)AD_Vdc_MEDF / AD_Range * Vref_AD - hCS_Init->V_Vdc_offset;


	hCS->Iu = hCS->V_Iu * hCS->Init.Iu_Gain;
	hCS->Iv = hCS->V_Iv * hCS->Init.Iv_Gain;
	hCS->Iw = hCS->V_Iw * hCS->Init.Iw_Gain;
	hCS->Vdc = hCS->V_Vdc * hCS->Init.Vdc_Gain;


}





