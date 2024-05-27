
#ifndef _SIN_T_H_
#define _SIN_T_H_


#include <stdint.h>


#define SIN_TBL_LEN		(8192)

#define SIN_TBL_MASK	(SIN_TBL_LEN - 1)
#define COS_T_SHIFT		(SIN_TBL_LEN >> 2)


extern const int16_t sin_table_q14[SIN_TBL_LEN];


#define COS_Q14(IDX)	(sin_table_q14[((uint32_t)(IDX) + COS_T_SHIFT) & SIN_TBL_MASK])
#define SIN_Q14(IDX)	(sin_table_q14[(uint32_t)(IDX) & SIN_TBL_MASK])


//const float sin_table2[];

/*
const float sin_table[];



float sin_t(float rad);

*/


#endif

