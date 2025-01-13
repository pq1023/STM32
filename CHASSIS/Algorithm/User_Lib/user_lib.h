#ifndef __USER_LIB_H
#define __USER_LIB_H

#include "sys.h"

#define VAL_LIMIT(val, min, max)\
    if(val<=min)\
    {\
        val = min;\
    }\
    else if(val>=max)\
    {\
        val = max;\
    }\


fp32 fp32_map(fp32 a, fp32 amin, fp32 amax, fp32 bmin, fp32 bmax);
int int_map(int a, int amin, int amax, int bmin, int bmax);

//¸¡µãËÀÇø
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26ËÀÇø
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);

extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
extern fp32 fp32_limit(fp32 raw, fp32 limit_min, fp32 limit_max);

#endif

