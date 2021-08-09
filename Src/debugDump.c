
#if 0
#include "debugDump.h"


#include <stdio.h>
#include <string.h>


#include "ACR.h"
#include "ASR.h"
#include "APR.h"
#include "Encoder.h"
#include "CurrentSensor.h"
#include "pwm.h"
#include "parameters.h"
#include "main.h"






#if DUMP_ENABLE

DumpRecord_TypeDef record[DUMP_STEPS];

uint32_t DumpCount = 0;


#endif


inline void Dump_Refresh()
{

#if DUMP_ENABLE

	if(DumpCount >= DUMP_STEPS) return;

	// 時刻取得
	record[DumpCount].t = DumpCount * DUMP_CYCLETIME;



#if 0
	// 電流センサの電圧（オフセット処理済み）
#define DATA_NAME_0		"V_Iu"
#define DATA_NAME_1		"V_Iv"
#define DATA_NAME_2		"V_Iw"
	record[DumpCount].data[0] = mainCS.V_Iu;
	record[DumpCount].data[1] = mainCS.V_Iv;
	record[DumpCount].data[2] = mainCS.V_Iw;
#endif


#if 0
	// 電流制御の応答

	mainACR.Iq_ref = (DumpCount & 0x100)? 5.0: -5.0;


#define DATA_NAME_0		"Id_ref"
#define DATA_NAME_1		"Iq_ref"
#define DATA_NAME_2		"Id"
#define DATA_NAME_3		"Iq"
#define DATA_NAME_4		"Vd_ref"
#define DATA_NAME_5		"Vq_ref"

	record[DumpCount].data[0] = mainACR.Id_ref;
	record[DumpCount].data[1] = mainACR.Iq_ref;
	record[DumpCount].data[2] = mainACR.Id;
	record[DumpCount].data[3] = mainACR.Iq;
	record[DumpCount].data[4] = mainACR.Vd_ref;
	record[DumpCount].data[5] = mainACR.Vq_ref;
#endif


#if 1
	// 速度制御の応答

	mainASR.omega_ref = (DumpCount & 0x400)? 30.0: -30.0;

#define DATA_NAME_0		"omega_ref"
#define DATA_NAME_1		"omega"
#define DATA_NAME_2		"Iq_ref"
#define DATA_NAME_3		"Iq"

	record[DumpCount].data[0] = mainASR.omega_ref;
	record[DumpCount].data[1] = mainASR.omega;
	record[DumpCount].data[2] = mainACR.Id_ref;
	record[DumpCount].data[3] = mainACR.Iq;
#endif


	//record[DumpCount].theta_re = mainEncoder.theta_re;

	//record[DumpCount].omega = mainEncoder.omega;


	DumpCount++;

#endif

}


#ifndef DATA_NAME_0
#define DATA_NAME_0 "data[0]"
#endif
#ifndef DATA_NAME_1
#define DATA_NAME_1 "data[1]"
#endif
#ifndef DATA_NAME_2
#define DATA_NAME_2 "data[2]"
#endif
#ifndef DATA_NAME_3
#define DATA_NAME_3 "data[3]"
#endif
#ifndef DATA_NAME_4
#define DATA_NAME_4 "data[4]"
#endif
#ifndef DATA_NAME_5
#define DATA_NAME_5 "data[5]"
#endif
#ifndef DATA_NAME_6
#define DATA_NAME_6 "data[6]"
#endif
#ifndef DATA_NAME_7
#define DATA_NAME_7 "data[7]"
#endif



void Dump_Print()
{

#if DUMP_ENABLE


	int count, dataCount, strLenTemp;

	float Id;

	char printStr[100] = "";


	printf("t, "DATA_NAME_0", "DATA_NAME_1", "DATA_NAME_2", "DATA_NAME_3", "DATA_NAME_4", "DATA_NAME_5", "DATA_NAME_6", "DATA_NAME_7"\n");

	for(count = 0; count < DUMP_STEPS; count++)
	{

		sprintf(printStr, "%.5f, ", record[count].t);


		for(dataCount = 0; dataCount < DUMP_NUM; dataCount++)
		{
			strLenTemp = strlen(printStr);
			sprintf(printStr + strLenTemp, "%.5f, ", record[count].data[dataCount]);
		}

		strLenTemp = strlen(printStr);
		sprintf(printStr + strLenTemp, "\n");


		printf("%s", printStr);


	}

#endif


}

#endif




