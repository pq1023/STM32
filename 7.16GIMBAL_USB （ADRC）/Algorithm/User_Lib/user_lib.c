#include "user_lib.h"


/**
  * @brief			范围映射
  * @author			RW
  * @param[in]		原始数值
  * @param[in]		原始最小值
  * @param[in]		原始最大值
  * @param[in]		映射后的最小值
  * @param[in]		映射后的最大值
  * @retval			映射后的数值
  *///										0								50			0				3
fp32 fp32_map(fp32 a, fp32 amin, fp32 amax, fp32 bmin, fp32 bmax)
{
	//			50								3
	fp32 adel = amax - amin, bdel = bmax - bmin;
	if(a > amax)a = amax;
	if(a < amin)a = amin;
	//		3*(0-50)/50
	return (fp32)(bdel * ((fp32)(a-amin) / adel))+bmin;
}
int int_map(int a, int amin, int amax, int bmin, int bmax)
{
	int adel = amax - amin, bdel = bmax - bmin;
	if(a > amax)a = amax;
	if(a < amin)a = amin;
	return (bdel * ((float)(a-amin) / adel))+bmin;
}

//浮点死区0			
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value > maxValue && Value < minValue)
    {
        Value = 0.0f;
    }

    return Value;
}

//int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value > maxValue && Value < minValue)
    {
        Value = 0;
    }

    return Value;
}

int16_t int16_limit(int16_t raw, int16_t limit_min, int16_t limit_max)
{
	if(raw > limit_max)
		return limit_max;
	else if(raw < limit_min)
		return limit_min;
	else
		return raw;
}

fp32 fp32_limit(fp32 raw, fp32 limit_min, fp32 limit_max)
{
	if(raw > limit_max)
		return limit_max;
	else if(raw < limit_min)
		return limit_min;
	else
		return raw;
}

