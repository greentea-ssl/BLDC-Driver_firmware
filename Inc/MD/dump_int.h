

#ifndef _DUMP_INT_H_
#define _DUMP_INT_H_

#include <stdint.h>



#define DUMP_CHANNELS (8)
#define DUMP_LENGTH	(2000)


extern uint32_t dump_record[DUMP_LENGTH][DUMP_CHANNELS];
extern uint32_t dump_counter;


void Dump_Init();


void Dump_Update();


uint8_t Dump_isFull();


void Dump_Print();






#endif /* _DUMP_INT_H_ */
