

#include "CurrentSensor.h"

#include "string.h"
#include "stdlib.h"


extern ADC_HandleTypeDef hadc1;

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

	mainCS.Init.hadc = &hadc1;

	mainCS.pos_MEDF_I = 0;

}


void CurrentSensor_Start(CurrentSensor_TypeDef *hCS)
{

	switch(hCS->Init.CS_Type)
	{
	case CS_Type_3shunt:


		HAL_ADCEx_InjectedStart(hCS->Init.hadc);

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
	static int32_t AD_Vdc_MEDF = 0;

	static CurrentSensor_InitTypeDef *hCS_Init;


	hCS_Init = &hCS->Init;

	switch(hCS->Init.CS_Type)
	{
	case CS_Type_3shunt:


		hCS->AD_Iu[0] = hCS->Init.hadc->Instance->JDR1;
		hCS->AD_Iv[0] = hCS->Init.hadc->Instance->JDR2;
		hCS->AD_Iw[0] = hCS->Init.hadc->Instance->JDR3;
		hCS->AD_Vdc[0] = hCS->Init.hadc->Instance->JDR4;

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

#if MEDIAN_ORDER == 3
		AD_Iu_MEDF = median3(hCS->AD_Iu_buf);
		AD_Iv_MEDF = median3(hCS->AD_Iv_buf);
		AD_Iw_MEDF = median3(hCS->AD_Iw_buf);
#elif MEDIAN_ORDER == 5
		AD_Iu_MEDF = median5(hCS->AD_Iu_buf);
		AD_Iv_MEDF = median5(hCS->AD_Iv_buf);
		AD_Iw_MEDF = median5(hCS->AD_Iw_buf);
#else
		AD_Iu_MEDF = hCS->AD_Iu[0];
		AD_Iv_MEDF = hCS->AD_Iv[0];
		AD_Iw_MEDF = hCS->AD_Iw[0];
#endif

		AD_Vdc_MEDF = hCS->AD_Vdc[0];

		// 端子電圧更新
		hCS->V_Iu = (float)AD_Iu_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iu_offset;
		hCS->V_Iv = (float)AD_Iv_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iv_offset;
		hCS->V_Iw = (float)AD_Iw_MEDF / AD_Range * Vref_AD - hCS_Init->V_Iw_offset;
		hCS->V_Vdc = (float)AD_Vdc_MEDF / AD_Range * Vref_AD - hCS_Init->V_Vdc_offset;


		/*
		hCS->Iu = hCS->V_Iu * hCS->Init.Iu_Gain;
		hCS->Iv = hCS->V_Iv * hCS->Init.Iv_Gain;
		hCS->Iw = hCS->V_Iw * hCS->Init.Iw_Gain;
		*/

		/*
		switch(SVM_sector)
		{
		case 0: case 5:
			hCS->Iv = hCS->V_Iv * hCS->Init.Iv_Gain;
			hCS->Iw = hCS->V_Iw * hCS->Init.Iw_Gain;
			hCS->Iu = - hCS->Iv - hCS->Iw;
			break;

		case 1: case 2:
			hCS->Iw = hCS->V_Iw * hCS->Init.Iw_Gain;
			hCS->Iu = hCS->V_Iu * hCS->Init.Iu_Gain;
			hCS->Iv = - hCS->Iw - hCS->Iu;
			break;

		case 3: case 4:
			hCS->Iu = hCS->V_Iu * hCS->Init.Iu_Gain;
			hCS->Iv = hCS->V_Iv * hCS->Init.Iv_Gain;
			hCS->Iw = - hCS->Iu - hCS->Iv;
			break;
		}
		*/

		hCS->Iu = hCS->V_Iu * hCS->Init.Iu_Gain;
		hCS->Iv = hCS->V_Iv * hCS->Init.Iv_Gain;
		hCS->Iw = hCS->V_Iw * hCS->Init.Iw_Gain;
		hCS->Vdc = hCS->V_Vdc * hCS->Init.Vdc_Gain;


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


inline int32_t median5(int32_t *buf)
{
	static uint32_t winCount[5] = {0};

	winCount[0] = 0;
	winCount[1] = 0;
	winCount[2] = 0;
	winCount[3] = 0;
	winCount[4] = 0;

	if (buf[0] > buf[1]) winCount[0]++; else winCount[1]++;
	if (buf[0] > buf[2]) winCount[0]++; else winCount[2]++;
	if (buf[0] > buf[3]) winCount[0]++; else winCount[3]++;
	if (buf[0] > buf[4]) winCount[0]++; else winCount[4]++;

	if (winCount[0] == 2) return buf[0];

	if (buf[1] > buf[2]) winCount[1]++; else winCount[2]++;
	if (buf[1] > buf[3]) winCount[1]++; else winCount[3]++;
	if (buf[1] > buf[4]) winCount[1]++; else winCount[4]++;

	if (winCount[1] == 2) return buf[1];

	if (buf[2] > buf[3]) winCount[2]++; else winCount[3]++;
	if (buf[2] > buf[4]) winCount[2]++; else winCount[4]++;

	if (winCount[2] == 2) return buf[2];

	if (buf[3] > buf[4]) winCount[3]++; else winCount[4]++;

	if (winCount[3] == 2) return buf[3];

	return buf[4];

}



