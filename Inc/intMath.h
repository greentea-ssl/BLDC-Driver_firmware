#ifndef _INT_MATH_H_
#define _INT_MATH_H_


#include <stdint.h>


int32_t intSqrt(int32_t a);


typedef struct
{
	int32_t Ts_q28;
	int32_t limit;
	int16_t gainShift;
	int32_t integ;
	int32_t error;
}IntInteg_TypeDef;


void IntInteg_Init(IntInteg_TypeDef *hInteg, int16_t gainShift, int32_t Ts_q28, int32_t limit);

int32_t IntInteg_Update(IntInteg_TypeDef *hInteg, int16_t u);




#endif /* _INT_MATH_H_ */
