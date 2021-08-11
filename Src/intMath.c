



#include "intMath.h"



int32_t intSqrt(int32_t a)
{

	// http://senta.s112.xrea.com/senta/Tips/000/c6/index.html

	if (a<=0)
		return 0;
	if (a==1)
		return 1;
	int x;

	if (a <= 38408)
		x = (a >> 7)+11;
	else if (a<=1411319)
		x = (a >> 10)+210;
	else if (a<=70459124)
		x = (a >> 13)+1414;
	else if (a<=794112116)
		x = (a >> 15)+7863;
	else
		x = (a>>17)+26038;

	x = (x +a/x) >> 1;
	x = (x +a/x) >> 1;

	return x;
}


void IntInteg_Init(IntInteg_TypeDef *hInteg, int16_t gainShift, int32_t Ts_q28, int32_t limit)
{
	hInteg->gainShift = gainShift;
	hInteg->Ts_q28 = Ts_q28;
	hInteg->limit = limit;
}

int32_t IntInteg_Update(IntInteg_TypeDef *hInteg, int16_t u)
{
	int32_t prd_q28 = u * hInteg->Ts_q28 + hInteg->error;
	hInteg->error = prd_q28 & ( ( 1 << (28 - hInteg->gainShift) ) - 1 );
	hInteg->integ += prd_q28 >> (28 - hInteg->gainShift);
	if(hInteg->integ > hInteg->limit) hInteg->integ = hInteg->limit;
	else if(hInteg->integ < -hInteg->limit) hInteg->integ = -hInteg->limit;
	return hInteg->integ;
}







