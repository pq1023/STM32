#if 0
#include "ADRC_core.h"
#include "ADRC_user.h"
#include "arm_math.h"
#include "user_lib.h"
#include "gimbal_task.h"
#include <math.h>

static float sign(float x);
static float fhan(float x1,float x2,float r,float h);
static float fal(float e,float alpha,float delta);

extern fp32 pitch_err;
float frequency = F;
double sine_wave(void)
{
    double signal[Lenth] = {0}; // array to store the signal
    double t; // time variable
    static int i; // loop index
		
		
		t = i / FS; // calculate the time for each sample
		signal[i] = A * sin(2 * PI * frequency * t + P); // calculate the signal value for each sample
    i++;
		i%=Lenth;
		
    return signal[i];
}


static float sign(float x)
{
	if(x>0)
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}


/*******************************fhan函数**********************************/
float fhan(float x1,float x2,float r,float h)
{
	float 	a    = 0,
		    a0   = 0,
		    y    = 0,
		    d	 = 0,
		    d0   = 0,
		    absy = 0,
		    absa = 0,
			fhan = 0;
	absy =  fabsf(y);
	absa =  fabsf(a);
	d    =  r*h;
	d0   =  h*d;
	y    =  x1+h*x2;
	a0   =  sqrtf(d*d+8*r*absy);
	if(absy > d0)
		a = x2 + (a0 - d)/2 * sign(y);
	else
		a = x2 + y / h;
	if(absa > d)
		fhan = -1*r*sign(a);
	else
		fhan = -1*a/d;
  return fhan;
}

/*******************************fal函数**********************************/
float fal(float e,float alpha,float delta)
{
  float result = 0,fabsf_e = 0;
  fabsf_e = fabsf(e);
	if(fabsf_e <= delta)
    result = e/pow(delta,1-alpha);
  else if(fabsf_e > delta)
    result = sign(e)*pow(fabsf_e,alpha);
	return result;    
}

extern state_param pitch_trans_num;
extern state_param yaw_trans_num;
int32_t offset = 5000;
/********************************ADRC************************************/
float ADRC(ADRC_t *ADRC,state_param *trans_num, float v,float y)
{
//	static float e1_off = 0;
/******************************TD****************************************/
    trans_num->x1 = trans_num->x1 + ADRC->h* trans_num->x2;
		if(trans_num->x1 > 20)		trans_num->x1 = 20;
		if(trans_num->x1 < -15)		trans_num->x1 = -15;
    trans_num->x2 = trans_num->x2 + ADRC->h* fhan(trans_num->x1-v, trans_num->x2, ADRC->r, ADRC->h0);
/******************************ESO***************************************/
    trans_num->e = trans_num->z1 - y;
    trans_num->z1 = trans_num->z1 + ADRC->h*(trans_num->z2 - ADRC->belta01* trans_num->e );
    trans_num->z2 = trans_num->z2 + ADRC->h*(trans_num->z3 - ADRC->belta02*fal(trans_num->e,0.5,ADRC->delta) + ADRC->b* trans_num->u);
    trans_num->z3 = trans_num->z3 + ADRC->h*(   - ADRC->belta03*fal(trans_num->e,0.25,ADRC->delta));
//	if(fabs(trans_num->e1) > ADRC->z3_seperate)
	if(fabs(pitch_err) > ADRC->z3_seperate)
		trans_num->z3_limit = 80000;
	else
		trans_num->z3_limit = 250000;
/******************限幅，ADRC正常的话不会达到限幅条件********************/
    if(trans_num->z1 >= 30000) 					trans_num->z1 = 30000;
    if(trans_num->z1 <= -30000) 				trans_num->z1 = -30000;
    if(trans_num->z2 >= 30000) 					trans_num->z2 = 30000;
    if(trans_num->z2 <= -30000) 				trans_num->z2 = -30000;
//    if(trans_num->z3 >= trans_num->z3_limit) 	trans_num->z3 = trans_num->z3_limit;
//    if(trans_num->z3 <= -trans_num->z3_limit) 	trans_num->z3 = -trans_num->z3_limit;
/******************************NLSEF*************************************/
    trans_num->e1 = trans_num->x1 - trans_num->z1;
    trans_num->e2 = trans_num->x2 - trans_num->z2;
		
//		trans_num->u0 = ADRC->KP * trans_num->e1 + ADRC->KD * trans_num->e2;
//		trans_num->u0 -= ADRC->KI * trans_num->e2;
    trans_num->u0 = ADRC->belta1*fal(trans_num->e1, ADRC->alpha1,ADRC->delta) + ADRC->belta2*fal(trans_num->e2,ADRC->alpha2,ADRC->delta);//其中0<alpha1<1<alpha2
	VAL_LIMIT(trans_num->u0, -30000, 30000);
	
	trans_num->u = trans_num->u0 - trans_num->z3/ADRC->b;
	VAL_LIMIT(trans_num->u, -30000, 30000);
  
    return trans_num->u - offset;
}
#endif


