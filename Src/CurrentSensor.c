

#include "CurrentSensor.h"

#include "adc.h"
#include "string.h"


const float Vref_AD = 3.3f;

const int32_t AD_Range = 4096;


CurrentSensor_TypeDef mainCS;


int32_t median3(int32_t *buf);


void CurrentSensor_Init()
{

	memcpy(&mainCS, 0x00, sizeof(mainCS));

	mainCS.Init.CS_Type = CS_Type_3shunt;
	mainCS.Init.Gain_currentSense = -10.0f; // 1 / ( R * OPAmpGain) [A / V]
	mainCS.Init.V_Iu_offset = 1.67497551f;
	mainCS.Init.V_Iv_offset = 1.67578125f;
	mainCS.Init.V_Iw_offset = 1.67819822f;
	mainCS.Init.hadc_Iu = &hadc1;
	mainCS.Init.hadc_Iv = &hadc2;
	mainCS.Init.hadc_Iw = &hadc3;

	mainCS.pos_MEDF_I = 0;

}


void CurrentSensor_Start(CurrentSensor_TypeDef *hCS)
{

	switch(hCS->Init.CS_Type)
	{
	case CS_Type_3shunt:

		HAL_ADC_Start_DMA(hCS->Init.hadc_Iu, hCS->AD_Iu, 1);
		HAL_ADC_Start_DMA(hCS->Init.hadc_Iv, hCS->AD_Iv, 1);
		HAL_ADC_Start_DMA(hCS->Init.hadc_Iw, hCS->AD_Iw, 1);

		break;
	}



}


/*
 * UVWの電流値を更新するだけ
 * 座標変換とかはやらない
 */
inline void CurrentSensor_Refresh(CurrentSensor_TypeDef *hCS, uint8_t SVM_sector)
{

	static int32_t AD_Iu_MEDF = 0;
	static int32_t AD_Iv_MEDF = 0;
	static int32_t AD_Iw_MEDF = 0;

	static CurrentSensor_InitTypeDef *hCS_Init;

	hCS_Init = &hCS->Init;

	switch(hCS->Init.CS_Type)
	{
	case CS_Type_3shunt:


		HAL_ADC_Start_DMA(hCS_Init->hadc_Iu, hCS->AD_Iu, 1);
		HAL_ADC_Start_DMA(hCS_Init->hadc_Iv, hCS->AD_Iv, 1);
		HAL_ADC_Start_DMA(hCS_Init->hadc_Iw, hCS->AD_Iw, 1);


		hCS->AD_Iu_buf[hCS->pos_MEDF_I] = (int32_t)hCS->AD_Iu[0];
		hCS->AD_Iv_buf[hCS->pos_MEDF_I] = (int32_t)hCS->AD_Iv[0];
		hCS->AD_Iw_buf[hCS->pos_MEDF_I] = (int32_t)hCS->AD_Iw[0];


		// メディアンフィルタ用バッファ書き込み位置更新
		hCS->pos_MEDF_I += 1;
		if(hCS->pos_MEDF_I >= 3)
		{
			hCS->pos_MEDF_I = 0;
		}

		AD_Iu_MEDF = median3(hCS->AD_Iu_buf);
		AD_Iv_MEDF = median3(hCS->AD_Iv_buf);
		AD_Iw_MEDF = median3(hCS->AD_Iw_buf);

		// 端子電圧更新
		hCS->V_Iu = (float)AD_Iu_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iu_offset;
		hCS->V_Iv = (float)AD_Iv_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iv_offset;
		hCS->V_Iw = (float)AD_Iw_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iw_offset;


		switch(SVM_sector)
		{
		case 0: case 5:
			hCS->Iv = hCS->V_Iv * hCS->Init.Gain_currentSense;
			hCS->Iw = hCS->V_Iw * hCS->Init.Gain_currentSense;
			hCS->Iu = - hCS->Iv - hCS->Iw;
			break;

		case 1: case 2:
			hCS->Iw = hCS->V_Iw * hCS->Init.Gain_currentSense;
			hCS->Iu = hCS->V_Iu * hCS->Init.Gain_currentSense;
			hCS->Iv = - hCS->Iw - hCS->Iu;
			break;

		case 3: case 4:
			hCS->Iu = hCS->V_Iu * hCS->Init.Gain_currentSense;
			hCS->Iv = hCS->V_Iv * hCS->Init.Gain_currentSense;
			hCS->Iw = - hCS->Iu - hCS->Iv;
			break;
		}

		break; /* CS_Type_3shunt */


		default:
			break;


	}




}


/*
 * 回転座標系における電流を算出
 */
inline void CurrentSensor_getIdq(CurrentSensor_TypeDef *hCS, float *Id, float *Iq, float cos_theta_re, float sin_theta_re)
{

	*Id = 0.8165f * (
			+ hCS->Iu * cos_theta_re
			+ hCS->Iv * (-0.5f * cos_theta_re + 0.855f * sin_theta_re)
			+ hCS->Iw * (-0.5f * cos_theta_re - 0.855f * sin_theta_re));

	*Iq = 0.8165f * (
			- hCS->Iu * sin_theta_re
			+ hCS->Iv * (0.5f * sin_theta_re + 0.855f * cos_theta_re)
			+ hCS->Iw * (0.5f * sin_theta_re - 0.855f * cos_theta_re));

}


/*
 * Length:3 のメディアンフィルタ
 */
inline int32_t median3(int32_t *buf)
{

	if(buf[0] < buf[1])
	{
		if(buf[2] < buf[0])			return buf[0];
		else if(buf[2] < buf[1])	return buf[2];
		else						return buf[1];
	}
	else
	{
		if(buf[2] < buf[1])			return buf[1];
		else if(buf[2] < buf[1])	return buf[2];
		else						return buf[0];
	}

	return 0;
}






