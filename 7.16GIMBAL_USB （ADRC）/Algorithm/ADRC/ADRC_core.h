#ifndef ADRC_CORE_H
#define ADRC_CORE_H
#include "ADRC_user.h"

typedef struct
{
	//中间变量区，不需要用户管理以及赋值
/****************TD*******************/
float 	x1,		//跟踪输入
		x2;		//跟踪输入的微分
/****************ESO******************/
float	e,		//误差
		z1,		//跟踪反馈值
		z2,		//跟踪反馈值的而微分
		z3,		//跟踪系统的扰动（总扰动）
		z3_limit;
/**************NLSEF******************/
float	u,		//输出值
		u0,		//非线性组合的输出
		e1,		//z1和x1的误差
		e2;		//z2和x2的误差
}state_param;

//			v: x1的设定值		y：当前x1
float ADRC(ADRC_t *ADRC,state_param *trans_num, float v,float y);

#endif
