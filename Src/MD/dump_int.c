


#include "dump_int.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "md_main.h"


uint32_t dump_record[DUMP_LENGTH][DUMP_CHANNELS];
uint32_t dump_counter;


void Dump_Init()
{

	dump_counter = 0;

}


inline void Dump_Update(MD_Handler_t* sys)
{

#if 1
	dump_record[dump_counter][0] = sys->motor.Id_ref_pu_2q13;
	dump_record[dump_counter][1] = sys->motor.Iq_ref_pu_2q13;
	dump_record[dump_counter][2] = sys->motor.Id_pu_2q13;
	dump_record[dump_counter][3] = sys->motor.Iq_pu_2q13;
	dump_record[dump_counter][4] = sys->motor.Vd_pu_2q13;
	dump_record[dump_counter][5] = sys->motor.Vq_pu_2q13;
	dump_record[dump_counter][6] = sys->motor.raw_theta_14bit;
	dump_record[dump_counter][7] = sys->motor.omega_q5;
#endif

	if(dump_counter < DUMP_LENGTH)
	{
		dump_counter++;
	}

}


uint8_t Dump_isFull()
{
	return (dump_counter >= DUMP_LENGTH);
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

	int count, dataCount, strLenTemp;

	char printStr[100] = "";


	printf("t, "DATA_NAME_0", "DATA_NAME_1", "DATA_NAME_2", "DATA_NAME_3", "DATA_NAME_4", "DATA_NAME_5", "DATA_NAME_6", "DATA_NAME_7"\n");

	for(count = 0; count < DUMP_LENGTH; count++)
	{

		sprintf(printStr, "%d, ", count);


		for(dataCount = 0; dataCount < DUMP_CHANNELS; dataCount++)
		{
			strLenTemp = strlen(printStr);
			sprintf(printStr + strLenTemp, "%d, ", dump_record[count][dataCount]);
		}

		strLenTemp = strlen(printStr);
		sprintf(printStr + strLenTemp, "\n");

		printf("%s", printStr);


	}

}








