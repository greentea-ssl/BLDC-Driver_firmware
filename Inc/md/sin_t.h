
#ifndef _SIN_T_H_
#define _SIN_T_H_


#include <stdint.h>


#define SIN_TBL_LEN		(8192)

#define SIN_TBL_MASK	(SIN_TBL_LEN - 1)
#define COS_T_SHIFT		(SIN_TBL_LEN >> 2)


extern const int16_t sin_table_q14[SIN_TBL_LEN];


#define COS_Q14(IDX)	(sin_table_q14[((uint32_t)(IDX) + COS_T_SHIFT) & SIN_TBL_MASK])
#define SIN_Q14(IDX)	(sin_table_q14[(uint32_t)(IDX) & SIN_TBL_MASK])


#define ATAN_TBL_LEN 	(2048)
#define ATAN_TBL_MASK	(ATAN_TBL_LEN - 1)
#define ATAN_T_SHIFT	(ATAN_TBL_LEN >> 2)

int16_t atan2_int(int16_t b, int16_t a);

#endif

