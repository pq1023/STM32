/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 			PID_DELTA: 	 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  *///                                                                                                         死区             0                0             0               积分分离  
void PID_GIMBAL_Init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		if (fabs(pid->Dead_Zone) < 1e-5)
    {
        pid->Dead_Zone = 0;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
		pid->I_Separation = I_Separation;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */

fp32 PID_GIMBAL_Calc(pid_type_def *pid, fp32 ref, fp32 set)
{		
	  uint8_t index;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref; //错误的产生
    if (pid->angle_max != pid->angle_min)//使积分增长变为梯形，如果angle_min和angle_max相等则不启用
    {
        if (pid->error[0] > (pid->angle_max + pid->angle_min) / 2)
            pid->error[0] -= (pid->angle_max + pid->angle_min);
        else if (pid->error[0] < -(pid->angle_max + pid->angle_min) / 2)
            pid->error[0] += (pid->angle_max + pid->angle_min);
    }
	  if (fabs(pid->error[0]) < pid->Dead_Zone) //死区，可以使pid系统更快的稳定下来，但是不能给太大，会导致系统滞后，且存在较大静差
    {
        pid->error[0] = pid->error[1] = 0;
    }
    if (fabs(pid->error[0]) > pid->I_Separation) //误差过大，采用积分分离
    {
        index = 0;
    }
    else
    {
			
        index = 1;
    }
    //只写增量式pid
    pid->Pout = pid->Kp * pid->error[0];

    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);

    //计算微分项增量带不完全微分，可以减少高频干扰，gama应是一个0-1的值		//低通滤波
    pid->Dout = pid->Kd * (1 - pid->gama) * (pid->Dbuf[0]) + pid->gama * pid->lastdout;
    LimitMax(pid->Iout, pid->max_iout);

    pid->out = pid->Pout + index * pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->lastdout = pid->Dout;
    pid->lastout = pid->out;
    return pid->out;
}

 /*实现前馈控制器*/
fp32 FeedforwardController(FFC *vFFC, fp32 A, fp32 B, fp32 T)
{
  float result;
	float q, w;
	vFFC->A = A;
	vFFC->B = B;
	vFFC->T = T;
	q = vFFC->A /(vFFC->B * vFFC->T);
	w = 1 / (vFFC->B * vFFC->T * vFFC->T);
  result = q * (vFFC->rin-vFFC->lastRin) + w * (vFFC->rin-2*vFFC->lastRin+vFFC->perrRin);
  vFFC->perrRin = vFFC->lastRin;
  vFFC->lastRin = vFFC->rin;
  return result;
}


fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
				if(fabs(pid->error[0]) > pid->I_Separation)
					pid->Iout = 0;
				else
					pid->Iout += pid->Ki * pid->error[0];
        pid->Pout = pid->Kp * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
			
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/*用于切换PID*/
 void pid_reset(pid_type_def *pid, fp32 PID[3],float Dead_Zone, float gama, float I_Separation )
{
	pid->Kp = PID[0];
	pid->Ki = PID[1];
	pid->Kd = PID[2];

	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->out  = 0;
	
	pid->Dead_Zone=Dead_Zone;
	pid->gama=gama;
	pid->I_Separation=I_Separation;
}

/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
void Pid_reset(PidTypeDef *pid, float p, float i, float d, float I_Separation)
{
  pid->Kp = p;
  pid->Ki = i;
  pid->Kd = d;
  
  pid->Pout = 0;
  pid->Iout = 0;
  pid->Dout = 0;
  pid->out  = 0;
  pid->I_Separation = I_Separation;
}


