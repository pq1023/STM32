/**
  ****************************(C) COPYRIGHT 2022 NCIST****************************
  * @file       chassis_task.c/h
  * @brief      底盘控制
  * @note       FreeRTOS任务
  * @auther     PQ
  * @history
  *  Version    Date            Author          Remarks
  *  V1.0.0     Dec-5-2023    	 PQ	             1. 完成
  ****************************(C) COPYRIGHT 2022 NCIST****************************
  */
#if 0
#include "chassis_task.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "judgement_info.h"
#include "pid.h"
#include "positioning.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

#include "user_lib.h"
#include "math.h"
#include "stdlib.h"
#include "arm_math.h"

#define CHASSIS_TASK_PRIO 20
#define CHASSIS_STK_SIZE 1024
TaskHandle_t ChassisTask_Handler;

#define Chassis_free_time 50
chassis_move_state chassis_move;

extern Distances_T Distances;

void Chassis_task(void);
static void chassis_init(chassis_move_state *chassis_init);//初始化函数
static void chassis_behaviour_mode_set(chassis_move_state *chassis_move_mode);//底盘模式设置
static void chassis_feedback_update(chassis_move_state *chassis_move_update);//底盘数据更新
static void chassis_Behaviour_update(chassis_move_state *chassis_behaviour_update);//底盘状态更新
static void mecanum_calc(float vx, float vy, float vz, float speed[], chassis_move_state *power_ctrl);//全向算法解算函数
static void Chassis_Power_Limit(chassis_move_state *power_ctrl);//裁判系统限制功率

/**
  * @brief	底盘任务创建
  */
void task_Chsssis_Create(void)
{
	xTaskCreate((TaskFunction_t)Chassis_task,
	            (const char *)"Chassisl_task",
	            (uint16_t)CHASSIS_STK_SIZE,
	            (void *)NULL,
	            (UBaseType_t)CHASSIS_TASK_PRIO,
	            (TaskHandle_t *)&ChassisTask_Handler);
}


/**
  * @brief	底盘任务
  */
int bisai_start=0,ones=1;
void Chassis_task(void)
{
	vTaskDelay(Chassis_free_time);
	chassis_init(&chassis_move);//底盘初始化
	while(1)
	{
		if(chassis_move.chassis_game_state->game_progress == 4)
		{
			bisai_start=1;	
		}else;	
		
		Distance_State();//里程计解算函数
		chassis_behaviour_mode_set(&chassis_move);//底盘模式设置
		chassis_feedback_update(&chassis_move);//底盘数据更新
		chassis_Behaviour_update(&chassis_move);//底盘状态更新
		
		CAN_CMD_CHASSIS(chassis_move.motor_speed_pid[0].out, chassis_move.motor_speed_pid[1].out, chassis_move.motor_speed_pid[2].out, chassis_move.motor_speed_pid[3].out);
		
		vTaskDelay(1);
	}
}
/**
  * @brief	底盘初始化
  */
u8 i;
static void chassis_init(chassis_move_state *chassis_init)
{
	if (chassis_init == NULL)
	{
			return;
	}
	
	const static 	fp32 chassis_move_pid[3] = {M3508_MOVE_KP, M3508_MOVE_KI, M3508_MOVE_KD};//底盘3508运动PID
	for (i = 0; i < 4; i++)
	{
		chassis_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);//底盘电机数据获取
		PID_Init(&chassis_init->motor_speed_pid[i], PID_POSITION, chassis_move_pid, M3505_MOVE_PID_MAX_OUT, M3505_MOVE_PID_MAX_IOUT);//底盘电机运动PID初始化
	}
	const static 	fp32 chassis_follow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
  PID_Init(&chassis_init->chassis_angle_pid, PID_POSITION, chassis_follow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	
	chassis_init->chassis_mode = CHASSIS_INIT; //底盘开机状态为停止
  chassis_init->chassis_rc_ctrl = get_can_remote_control_point();//获取遥控器指针
//	chassis_init->chassis_rc_ctrl = get_remote_control_point();//获取遥控器指针
	
  chassis_init->Chassis_Angular = get_Gyro_Angle_Point();//陀螺仪姿态指针   
	
	chassis_init->chassis_game_state = get_game_state_t();//比赛开始标志
	chassis_init->chassis_status_measure = get_game_robot_state_t();//底盘裁判系统功率读取
	chassis_init->chassis_power_measure = get_power_heat_data_t();//底盘裁判系统功率读取
	chassis_init->infrared_distance = get_sensor_Distance_point();
	
	
	chassis_init->chassis_monitor_point = getErrorListPoint();//获取检测系统数据指针
	
}

/**
  * @brief	底盘模式设置
  */
static void chassis_behaviour_mode_set(chassis_move_state *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
			return;
	}	
	
	if(switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左下右中
		chassis_move_mode->chassis_mode = CHASSIS_INIT; //初始化
	}
	else if(switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右下
		chassis_move_mode->chassis_mode = CHASSIS_SEPARATE_GIMBAL; //底盘分离 云台遥控器控制
	}
	else if(switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右中
		chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL; //底盘跟随 云台遥控器控制
	}
	else if(switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右上
		chassis_move_mode->chassis_mode = CHASSIS_RELAX; //底盘无力 云台手动自瞄
	}	
	else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右下
		chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE; //底盘小陀螺 云台手动自瞄
	}
	else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右中
		chassis_move_mode->chassis_mode = CHASSIS_RELAX; //底盘无力 云台自动自瞄
	}
	else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右上
		chassis_move_mode->chassis_mode = CHASSIS_AUTOMATIC; //底盘小陀螺 云台自动自瞄
	}
	else if(switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{    
		//双下
		chassis_move_mode->chassis_mode = CHASSIS_RELAX;
	}	
	else 
	{
		chassis_move_mode->chassis_mode = CHASSIS_RELAX;		
	}
}

/**
  * @brief 底盘数据更新
  */
static void chassis_feedback_update(chassis_move_state *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    //速度反馈
    chassis_move_update->wheel_spd_fdb[0] = chassis_move_update->motor_chassis[0].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[1] = chassis_move_update->motor_chassis[1].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[2] = chassis_move_update->motor_chassis[2].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[3] = chassis_move_update->motor_chassis[3].chassis_motor_measure->speed_rpm;

    //遥控器数据更新
    chassis_move_update->RC_X_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL] / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
    chassis_move_update->RC_Y_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL] / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
    chassis_move_update->RC_Z_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_VZ_RC_SEN;
}

/**
  * @brief 底盘状态更新
  */
fp32 Rotation_rate = 0.0f;
int chassis_state = 0;
static void chassis_Behaviour_update(chassis_move_state *chassis_behaviour_update)
{
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	float Distances_Yset = 100.0f, Distances_Xset = 100.0f;
	
	if (chassis_behaviour_update == NULL)
	{
			return;
	}	
	if(chassis_behaviour_update->chassis_mode == CHASSIS_RELAX)
	{
			for(i = 0; i < 4; i++)
			{
					chassis_move.motor_speed_pid[i].out = 0;
			}
	}
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_INIT)
	{
			chassis_behaviour_update->vx = 0;
			chassis_behaviour_update->vy = 0;
			chassis_behaviour_update->vz = 0;
			mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
	}	
   //底盘跟随
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
	{
//			float FOLLOW_GIMBAL_Z;//左右旋转
		
			//相对角度
			sin_yaw = arm_sin_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
			cos_yaw = arm_cos_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);

			chassis_behaviour_update->vx = (cos_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + sin_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
			chassis_behaviour_update->vy = (-sin_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + cos_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
//			chassis_behaviour_update->chassis_relative_angle_set = 0.0f;
//			FOLLOW_GIMBAL_Z = PID_Calc(&chassis_behaviour_update->chassis_angle_pid, chassis_behaviour_update->chassis_yaw_motor->relative_angle, chassis_behaviour_update->chassis_relative_angle_set);
//			/* 扭头速度越快,前后速度越慢,防止转弯半径过大 */
//					if( fabs(FOLLOW_GIMBAL_Z) > 160.0f)//210
//					{
//							Rotation_rate = ((CHASSIS_KB_RC_MAX_SPEED - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / CHASSIS_KB_RC_MAX_SPEED) * ((CHASSIS_KB_RC_MAX_SPEED - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / CHASSIS_KB_RC_MAX_SPEED);
//					}
//					else
//					{
//							Rotation_rate = 1.0f;
//					}
									
					chassis_behaviour_update->vx = Rotation_rate * fp32_constrain(chassis_behaviour_update->vx, -CHASSIS_KB_RC_MAX_SPEED, CHASSIS_KB_RC_MAX_SPEED);
					chassis_behaviour_update->vy = Rotation_rate * fp32_constrain(chassis_behaviour_update->vy, -CHASSIS_KB_RC_MAX_SPEED, CHASSIS_KB_RC_MAX_SPEED);
//					chassis_behaviour_update->vz = fp32_constrain(FOLLOW_GIMBAL_Z, -CHASSIS_KB_RC_MAX_SPEED, CHASSIS_KB_RC_MAX_SPEED);  //deg/s
			
			mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
	}	
	//底盘分离
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_SEPARATE_GIMBAL)
	{
		if(chassis_behaviour_update->infrared_distance->sensor[0] < 200 || chassis_behaviour_update->infrared_distance->sensor[1] < 200 || chassis_behaviour_update->infrared_distance->sensor[2] < 200)
		{
			chassis_behaviour_update->vx = 0;
			chassis_behaviour_update->vy = 0;
			chassis_behaviour_update->vz = 0;			
		}
		else
		{
			chassis_behaviour_update->vx = chassis_behaviour_update->RC_X_ChassisSpeedRef;
			chassis_behaviour_update->vy = chassis_behaviour_update->RC_Y_ChassisSpeedRef;
			chassis_behaviour_update->vz = chassis_behaviour_update->RC_Z_ChassisSpeedRef;	
		}
			mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
	}
	//小陀螺
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_DODGE_MODE)
	{
		if(chassis_behaviour_update->chassis_game_state->game_progress == 4 || shoot_state == 1)
		{
			sin_yaw = arm_sin_f32(-(chassis_behaviour_update->chassis_yaw_motor->relative_angle + chassis_behaviour_update->Chassis_Angular->YAW)* ANGLE_TO_RAD);
			cos_yaw = arm_cos_f32(-(chassis_behaviour_update->chassis_yaw_motor->relative_angle + chassis_behaviour_update->Chassis_Angular->YAW)* ANGLE_TO_RAD);	
			if((bisai_start && ones) || chassis_state == 1)
			{
				
				ones=0;
				chassis_state=0;
			}
			else
			{	
        chassis_behaviour_update->vx = (cos_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + sin_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef)*0.27f;
        chassis_behaviour_update->vy = (-sin_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + cos_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef)*0.27f;
				chassis_behaviour_update->vz = 350.0f;
			}
		 mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}
		else 
		{
			chassis_behaviour_update->vx = chassis_behaviour_update->RC_X_ChassisSpeedRef;
			chassis_behaviour_update->vy = chassis_behaviour_update->RC_Y_ChassisSpeedRef;
			chassis_behaviour_update->vz = chassis_behaviour_update->RC_Z_ChassisSpeedRef;

			mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}
	}
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_AUTOMATIC)
	{
		if(chassis_behaviour_update->chassis_game_state->game_progress == 4 || shoot_state == 1)
		{
			sin_yaw = arm_sin_f32(-(chassis_behaviour_update->chassis_yaw_motor->relative_angle + chassis_behaviour_update->Chassis_Angular->YAW)* ANGLE_TO_RAD);
			cos_yaw = arm_cos_f32(-(chassis_behaviour_update->chassis_yaw_motor->relative_angle + chassis_behaviour_update->Chassis_Angular->YAW)* ANGLE_TO_RAD);	
			if((bisai_start && ones) || chassis_state == 1)
			{
				
				ones=0;
				chassis_state=0;
			}
			else
			{
				if(chassis_behaviour_update->infrared_distance->sensor[0] <= 200 || chassis_behaviour_update->infrared_distance->sensor[1] <= 200 || chassis_behaviour_update->infrared_distance->sensor[2] <= 200)
				{
					chassis_behaviour_update->vx = 0;
					chassis_behaviour_update->vy = 0;
					chassis_behaviour_update->vz = 350.0f;
				}
				else
				{
					chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_angle_pid, Distances.Real_Distance_X, Distances_Xset);
					chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_angle_pid, Distances.Real_Distance_Y, Distances_Yset);
					chassis_behaviour_update->vz = 350.0f;					
				}
			}
		 mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}
		else 
		{
			chassis_behaviour_update->vx = chassis_behaviour_update->RC_X_ChassisSpeedRef;
			chassis_behaviour_update->vy = chassis_behaviour_update->RC_Y_ChassisSpeedRef;
			chassis_behaviour_update->vz = chassis_behaviour_update->RC_Z_ChassisSpeedRef;

			mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}

	}
	for (u8 m = 0; m < 4; m++)
	{
		PID_Calc(&chassis_behaviour_update->motor_speed_pid[m], chassis_behaviour_update->wheel_spd_fdb[m], chassis_behaviour_update->wheel_spd_ref[m]);
	}
	Chassis_Power_Limit(chassis_behaviour_update);	
}

/**
  * @brief 底盘全向算法
  */
float   wheel_rpm[4];
static void mecanum_calc(float vx, float vy, float vz, float speed[], chassis_move_state *power_ctrl)
{
    static float rotate_ratio_fr;//前右
		static float wheel_rpm_ratio;
	
		uint16_t MAX_WHEEL_RPM;
    float   max = 0;
	
    rotate_ratio_fr 	= WHEELSPACING / RADIAN_COEF;
    wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO);
	
		if(power_ctrl->chassis_status_measure->chassis_power_limit == 100)	
			MAX_WHEEL_RPM = 7000 ;
		else 
			MAX_WHEEL_RPM = 5000;
		
    wheel_rpm[0] = (+vx - vy + vz * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] = (+vx + vy + vz * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[2] = (-vx + vy + vz * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[3] = (-vx - vy + vz * rotate_ratio_fr) * wheel_rpm_ratio;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabsf(wheel_rpm[i]) > max) 
            max = fabsf(wheel_rpm[i]);
    }

    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;

        for (uint8_t i = 0; i < 4; i++)
            wheel_rpm[i] *= rate;
    }
    memcpy(speed, wheel_rpm, 4 * sizeof(float));
}

/**
  * @brief 底盘功率限制
  */
#define WARNING_ENERGY	60
float toatl_speed_err,out_speed_err[4],finaal_out[4];
static void Chassis_Power_Limit(chassis_move_state *power_ctrl)
{
    /*********************祖传算法*************************/
    float    kLimit = 0.0f;//功率限制系数
    float 	 fTotalCurrentLimit;
    float    chassis_totaloutput = 0.0f;//统计总输出电流
    float    Joule_Residue = 0.0f;//剩余焦耳缓冲能量
    static 	 int16_t judgDataError_Time = 0;

    Joule_Residue = power_ctrl->chassis_power_measure->chassis_power_buffer;//剩余焦耳能量
 
    //统计底盘总输出
	for(int i=0;i<4;i++)
	{
	out_speed_err[i]=power_ctrl->wheel_spd_ref[i]-power_ctrl->wheel_spd_fdb[i];
	}
	toatl_speed_err=abs((int16_t)out_speed_err[0])+abs((int16_t)out_speed_err[1])+abs((int16_t)out_speed_err[2])+abs((int16_t)out_speed_err[3]);
    chassis_totaloutput = abs((int16_t)power_ctrl->motor_speed_pid[0].out) + abs((int16_t)power_ctrl->motor_speed_pid[1].out) 
																						+ abs((int16_t)power_ctrl->motor_speed_pid[2].out) + abs((int16_t)power_ctrl->motor_speed_pid[3].out);//统计总输出电流

    if(power_ctrl->chassis_monitor_point[JudgementTOE].errorExist == 1)//裁判系统无效时强制限速
    {
        judgDataError_Time++;

        if(judgDataError_Time > 100)
        {
            for (u8 i = 0; i < 4; i++)
            {
                VAL_LIMIT(power_ctrl->motor_speed_pid[i].out, -6000, 6000);
            }
        }
    }

    if(power_ctrl->chassis_monitor_point[JudgementTOE].errorExist == 0)
    {
        judgDataError_Time = 0;

        //剩余焦耳量过小,开始限制输出,限制系数为平方关系
        if(Joule_Residue < WARNING_ENERGY)
        {
            kLimit = (float)(Joule_Residue / WARNING_ENERGY)	* (float)(Joule_Residue / WARNING_ENERGY);//系数为 (当前剩余焦耳/60)的平方
            fTotalCurrentLimit = (kLimit * M3505_MOVE_PID_MAX_OUT * 4);
        }
        else //焦耳能量恢复到一定数值
        {
            fTotalCurrentLimit = (M3505_MOVE_PID_MAX_OUT * 4);
        }

        //底盘各电机电流重新分配
        if (chassis_totaloutput > fTotalCurrentLimit)
        {
            //赋值电流值
//            power_ctrl->motor_speed_pid[0].out = ((power_ctrl->motor_speed_pid[0].out) / chassis_totaloutput * fTotalCurrentLimit);
//            power_ctrl->motor_speed_pid[1].out = ((power_ctrl->motor_speed_pid[1].out) / chassis_totaloutput * fTotalCurrentLimit);
//            power_ctrl->motor_speed_pid[2].out = ((power_ctrl->motor_speed_pid[2].out) / chassis_totaloutput * fTotalCurrentLimit);
//            power_ctrl->motor_speed_pid[3].out = ((power_ctrl->motor_speed_pid[3].out) / chassis_totaloutput * fTotalCurrentLimit);
					    power_ctrl->motor_speed_pid[0].out = out_speed_err[0]/toatl_speed_err* fTotalCurrentLimit;
				     	power_ctrl->motor_speed_pid[1].out = out_speed_err[1]/toatl_speed_err* fTotalCurrentLimit;
				    	power_ctrl->motor_speed_pid[2].out = out_speed_err[2]/toatl_speed_err* fTotalCurrentLimit;
				    	power_ctrl->motor_speed_pid[3].out = out_speed_err[3]/toatl_speed_err* fTotalCurrentLimit;
        }
				for(int i=0;i<4;i++)
				{
					finaal_out[i]=power_ctrl->motor_speed_pid[i].out;
				}
    }
}



#endif

