
#ifndef _DEBUG_DUMP_H_
#define _DEBUG_DUMP_H_


#include "main.h"
#include "stdint.h"


#define DUMP_ENABLE		0


#define DUMP_NUM	(8)


#define DUMP_STEPS		(2000)


#define DUMP_CYCLETIME	(100E-6)



typedef struct
{

	float t;

	float data[DUMP_NUM+1];

}DumpRecord_TypeDef;


extern DumpRecord_TypeDef record[DUMP_STEPS];


extern uint32_t DumpCount;



void Dump_Refresh();


void Dump_Print();





#endif /* _DEBUG_DUMP_H_ */

