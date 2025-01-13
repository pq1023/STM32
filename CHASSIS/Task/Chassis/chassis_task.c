/**
  ****************************(C) COPYRIGHT 2022 NCIST****************************
  * @file       chassis_task.c/h
  * @brief      底盘控制
  * @note       FreeRTOS任务
  * @auther     PQ
  * @history
  *  Version    Date            Author          Remarks
  *  V1.0.0     Dec-8-2023     	  PQ            1. 完成
  ****************************(C) COPYRIGHT 2022 NCIST****************************
  */

#include "chassis_task.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "judgement_info.h"
#include "pid.h"
#include "positioning.h"
#include "Ramp_Control.h"
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
#include "timer.h"

#define CHASSIS_TASK_PRIO 20
#define CHASSIS_STK_SIZE 1024
TaskHandle_t ChassisTask_Handler;

#define Chassis_free_time 50
chassis_move_state chassis_move;

extern Distances_T Distances;
extern float MPU_R_YAW;

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
u8 bisai_start=0, outpost=0;
int color = 0;
void Chassis_task(void)
{
	vTaskDelay(Chassis_free_time);
	chassis_init(&chassis_move);//底盘初始化
	while(1)
	{
		color = is_red_or_blue();
		chassis_move.outpost_hp = Remain_outpost_HP();
		if(chassis_move.chassis_game_state->game_progress == 4)
			bisai_start = 1;	
		else
			bisai_start = 0;	
		if(chassis_move.outpost_hp <= 500 && chassis_move.outpost_hp > 0)
			outpost = 1;
		else
			outpost = 0;
		Distance_State();//里程计解算函数
 		chassis_behaviour_mode_set(&chassis_move);//底盘模式设置
		chassis_feedback_update(&chassis_move);//底盘数据更新
		chassis_Behaviour_update(&chassis_move);//底盘状态更新
		if(chassis_move.chassis_mode == CHASSIS_RELAX)
		{
			CAN_CMD_CHASSIS(0, 0, 0, 0);
		}
		else	
		{			
			CAN_CMD_CHASSIS(chassis_move.motor_speed_pid[0].out, chassis_move.motor_speed_pid[1].out, chassis_move.motor_speed_pid[2].out, chassis_move.motor_speed_pid[3].out);
//			CAN_CMD_CHASSIS(0, 0, 0, 0);
		}	
		vTaskDelay(1);
	}
}
/**
  * @brief	底盘初始化
  */
u8 i;
int kin = 0; 
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
	}
	PID_Init(&chassis_init->motor_speed_pid[0], PID_POSITION, chassis_move_pid, M3505_MOVE_PID_MAX_OUT, M3505_MOVE_PID_MAX_IOUT);//底盘电机运动PID初始化
	PID_Init(&chassis_init->motor_speed_pid[1], PID_POSITION, chassis_move_pid, M3505_MOVE_PID_MAX_OUT, M3505_MOVE_PID_MAX_IOUT);//底盘电机运动PID初始化
	PID_Init(&chassis_init->motor_speed_pid[2], PID_POSITION, chassis_move_pid, M3505_MOVE_PID_MAX_OUT, M3505_MOVE_PID_MAX_IOUT);//底盘电机运动PID初始化
	PID_Init(&chassis_init->motor_speed_pid[3], PID_POSITION, chassis_move_pid, M3505_MOVE_PID_MAX_OUT, M3505_MOVE_PID_MAX_IOUT);//底盘电机运动PID初始化

	const static 	fp32 chassis_follow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	PID_Init(&chassis_init->chassis_angle_pid, PID_POSITION, chassis_follow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

	const static fp32 chassis_positi_pid[3] = {CHASSIS_AOUT_KP, CHASSIS_AOUT_KI, CHASSIS_AOUT_KD};//全场定位的PID
	PID_Init(&chassis_init->chassis_auto_pid, PID_POSITION, chassis_positi_pid, CHASSIS_AOUT_PID_MAX_OUT, CHASSIS_AOUT_PID_MAX_IOUT);	
	
	const static fp32 chassis_X_PID[3] = {CHASSIS_X_KP, CHASSIS_X_KI, CHASSIS_X_KD};
	PID_Init(&chassis_init->chassis_X_pid, PID_POSITION, chassis_X_PID, CHASSIS_X_PID_MAX_OUT, CHASSIS_X_PID_MAX_IOUT);
	
	const static fp32 chassis_Y_PID[3] = {CHASSIS_Y_KP, CHASSIS_Y_KI, CHASSIS_Y_KD};
	PID_Init(&chassis_init->chassis_Y_pid, PID_POSITION, chassis_Y_PID, CHASSIS_Y_PID_MAX_OUT, CHASSIS_Y_PID_MAX_IOUT);
	
	const static fp32 chassis_Z_PID[3] = {CHASSIS_Z_KP, CHASSIS_Z_KI, CHASSIS_Z_KD};
	PID_Init(&chassis_init->chassis_Z_pid, PID_POSITION, chassis_Z_PID, CHASSIS_Z_PID_MAX_OUT, CHASSIS_Z_PID_MAX_IOUT);
	
	chassis_init->chassis_mode = CHASSIS_RELAX; //底盘开机状态为停止4
	
	chassis_init->chassis_rc_ctrl = get_can_remote_control_point();//获取上板遥控器指针
//	chassis_init->chassis_rc_ctrl = get_remote_control_point();//获取遥控器指针
	
	chassis_init->Chassis_Angular = get_Gyro_Angle_Point();//陀螺仪姿态指针   
	
	chassis_init->chassis_game_state = get_game_state_t();//比赛开始标志
	chassis_init->chassis_status_measure = get_game_robot_state_t();//底盘裁判系统功率读取
	chassis_init->chassis_power_measure = get_power_heat_data_t();//底盘裁判系统功率读取
	chassis_init->chassis_self_location = get_Robot_Pos_t();
	chassis_init->chassis_command = get_Robot_Command_t();
	
	chassis_init->infrared_distance = get_sensor_Distance_point();
	chassis_init->chassis_yaw_motor = get_yaw_motor_point();
	
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
		chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE; //底盘自杀 云台手动自瞄
	}	
	else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右下
//		chassis_move_mode->chassis_mode = CHASSIS_SEPARATE_GIMBAL; //底盘无力 云台手动自瞄
		chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE;
	}
	else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右中
		chassis_move_mode->chassis_mode = CHASSIS_RELAX; //底盘无力 云台自动自瞄
	}
	else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右上
		chassis_move_mode->chassis_mode = CHASSIS_ARTIFICIAL; //底盘自动 云台自动自瞄
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
fp32 autoX,autoY,autoZ = 0.0f;
int bit1,bit2=0;
int bit3,bit4=0;
int chassis_return = 0;
int chassis_s = 0;
int ain = 0;
//static float  command_x,command_y;
extern float chassis_vy, chassis_vz, chassis_vx;
float Ramp_vx, Ramp_vy, Ramp_vz;
int dodge = 0;
int h=0;
static void chassis_Behaviour_update(chassis_move_state *chassis_behaviour_update)
{
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	
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
		float FOLLOW_GIMBAL_Z;//左右旋转
	
		//相对角度
		sin_yaw = arm_sin_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
		cos_yaw = arm_cos_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);

		chassis_behaviour_update->vx = (cos_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + sin_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
		chassis_behaviour_update->vy = (-sin_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + cos_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
		chassis_behaviour_update->chassis_relative_angle_set = 0.0f;
		FOLLOW_GIMBAL_Z = PID_Calc(&chassis_behaviour_update->chassis_angle_pid, chassis_behaviour_update->chassis_yaw_motor->relative_angle, chassis_behaviour_update->chassis_relative_angle_set);
		/* 扭头速度越快,前后速度越慢,防止转弯半径过大 */
		if( fabs(FOLLOW_GIMBAL_Z) > 160.0f)//210
		{
				Rotation_rate = ((CHASSIS_KB_RC_MAX_SPEED - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / CHASSIS_KB_RC_MAX_SPEED) * ((CHASSIS_KB_RC_MAX_SPEED - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / CHASSIS_KB_RC_MAX_SPEED);
		}
		else
		{
				Rotation_rate = 1.0f;
		}							
		chassis_behaviour_update->vx = Rotation_rate * fp32_constrain(chassis_behaviour_update->vx, -CHASSIS_KB_RC_MAX_SPEED, CHASSIS_KB_RC_MAX_SPEED);
		chassis_behaviour_update->vy = Rotation_rate * fp32_constrain(chassis_behaviour_update->vy, -CHASSIS_KB_RC_MAX_SPEED, CHASSIS_KB_RC_MAX_SPEED);
		chassis_behaviour_update->vz = fp32_constrain(FOLLOW_GIMBAL_Z, -CHASSIS_KB_RC_MAX_SPEED, CHASSIS_KB_RC_MAX_SPEED);
		
		mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
	}	
	//底盘分离
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_SEPARATE_GIMBAL)
	{		
		chassis_behaviour_update->vx = chassis_behaviour_update->RC_X_ChassisSpeedRef;
		chassis_behaviour_update->vy = chassis_behaviour_update->RC_Y_ChassisSpeedRef;
		chassis_behaviour_update->vz = chassis_behaviour_update->RC_Z_ChassisSpeedRef;
		mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
	}
	//小陀螺
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_DODGE_MODE)
	{
		sin_yaw = arm_sin_f32(-(chassis_behaviour_update->chassis_yaw_motor->relative_angle + MPU_R_YAW)* ANGLE_TO_RAD);
		cos_yaw = arm_cos_f32(-(chassis_behaviour_update->chassis_yaw_motor->relative_angle + MPU_R_YAW)* ANGLE_TO_RAD);	

        chassis_behaviour_update->vx = 0;
        chassis_behaviour_update->vy = 0;
		chassis_behaviour_update->vz = 250.0f;
		gumbal_judge.Start_Dodge = 1;		
		mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
	}
	//自动模式
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_AUTOMATIC)//去前哨战
	{
		if(bisai_start == 1 || shoot_state == 1)
		{	
			if(outpost == 1 && chassis_return ==1)
			{
				switch(bit3)
				{//			345		763
					case 1: autoX = 104.0f;autoY = 529.0f;break;
					case 2: autoX = 111.0f;autoY = 142.0f;break; 
					case 3: autoX =	0.0f;autoY = 140.0f;break;
					case 4: autoX =	0.0f;autoY = 0.0f;break;
					default:break ;
				}
				switch(bit3)
				{				
					case 0:	if(Distances.Real_Distance_Y>523.0f&&Distances.Real_Distance_Y<536.0f){bit3++;}break;
					case 1:	if(Distances.Real_Distance_Y>136.0f&&Distances.Real_Distance_Y<148.0f){bit3++;}break;
					case 2:	if(Distances.Real_Distance_X>-6.0f&&Distances.Real_Distance_X<6.0f){bit3++;}break;
					case 3:	if(Distances.Real_Distance_Y>134.0f&&Distances.Real_Distance_Y<146.0f){bit3++;}break;
					case 4:if(Distances.Real_Distance_Y>-5.0f&&Distances.Real_Distance_Y<5.0f)
						{
							bit3++;
							chassis_s=1;
						}break;
					default:break ;
				}
				if(chassis_s == 0)
				{
					chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
					chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
					chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);	
				}
				else if(chassis_s == 1)
				{
					chassis_behaviour_update->vx = 0;
					chassis_behaviour_update->vy = 0;
					chassis_behaviour_update->vz = 250.0f;
					gumbal_judge.Start_Dodge = 1;					
				}		
			}
			else
			{  
				switch(bit1)
				{
					case 1:		autoX = 0.0f;autoY = 140.0f;break;
					case 2:		autoX = 111.0f;autoY = 140.0f;break;
					case 3:		autoX = 104.0f;autoY = 529.0f;break;
					case 4:		autoX = 270.0f;autoY = 539.0f;break;
					default:break ;
				}
				switch(bit1)
				{
					case 0:	if(Distances.Real_Distance_Y>-2.0f&&Distances.Real_Distance_Y<2.0f)bit1++;break;
					case 1:	if(Distances.Real_Distance_Y>134.0f&&Distances.Real_Distance_Y<146.0f)bit1++;break;
					case 2:	if(Distances.Real_Distance_X>106.0f&&Distances.Real_Distance_X<116.0f)bit1++;break;
					case 3:	if(Distances.Real_Distance_Y>522.0f&&Distances.Real_Distance_Y<536.0f)bit1++;break;
					case 4: if(Distances.Real_Distance_X>264.0f&&Distances.Real_Distance_X<276.0f)
						{
							bit1++;
							chassis_return = 1;
							break;
						}
					default:break ;
				}
			chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
			chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
			chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);
			}
		  mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}
		else ;
	}
/***********************************************************************/	
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_CHARGED)//原地右走
	{
		if(bisai_start == 1 || shoot_state == 1)
		{	
			if(outpost == 1 && chassis_return ==1)
			{
				switch(bit3)
				{//			345		763
					case 1: autoX = 0.0f;autoY = 0.0f;break;
					default:break ;
				}
				switch(bit3)
				{				
					case 0:	if(Distances.Real_Distance_Y>146.0f&&Distances.Real_Distance_Y<154.0f){bit3++;}break;
					case 1:	if(Distances.Real_Distance_Y>-4.0f&&Distances.Real_Distance_Y<4.0f)
						{
							bit3++;
							chassis_s=1;
						}break;
					default:break ;
				}
				if(chassis_s == 0)
				{
					chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
					chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
					chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);	
				}
				else if(chassis_s == 1)
				{
					chassis_behaviour_update->vx = 0;
					chassis_behaviour_update->vy = 0;
					chassis_behaviour_update->vz = 250.0f;				
				}		
			}
			else
			{  
				switch(bit1)
				{
					case 1:		autoX = 0.0f;autoY = 150.0f;break;
					default:break ;
				}
				switch(bit1)
				{
					case 0:	if(Distances.Real_Distance_Y>-2.0f&&Distances.Real_Distance_Y<2.0f)bit1++;break;
					case 1:	if(Distances.Real_Distance_Y>146.0f&&Distances.Real_Distance_Y<154.0f)
						{
							bit1++;
							chassis_return = 1;
							break;
						}
					default:break ;
				}
			chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
			chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
			chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);
			}
		  mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}
		else ;
	}
	/**************************************************************************/
//自杀模式
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_SUICIDE)
	{
		if(bisai_start == 1 || shoot_state == 1)
		{	
			if(outpost == 1 && chassis_return ==1)
			{
				switch(bit3)
				{	//						185								484
					case 1: autoX = 185.0f;autoY = 412.0f;break;
					case 2: autoX = 0.0f;autoY = 176.0f;break; 
					case 3: autoX =	0.0f;autoY = 0.0f;break;
					default:break ;
				}
				switch(bit3)
				{				
					case 0:	if(Distances.Real_Distance_Y>482.0f&&Distances.Real_Distance_Y<486.0f){bit3++;}break;
					case 1:	if(Distances.Real_Distance_Y>409.0f&&Distances.Real_Distance_Y<415.0f){bit3++;}break;
					case 2:	if(Distances.Real_Distance_Y>173.0f&&Distances.Real_Distance_Y<179.0f){bit3++;}break;
					case 3:	if(Distances.Real_Distance_Y>-3.0f&&Distances.Real_Distance_Y<3.0f)
						{
							bit3++;
							chassis_s=1;
						}break;
					default:break ;
				}
				if(chassis_s == 0)
				{
					chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
					chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
					chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);	
				}
				else if(chassis_s == 1)
				{
					chassis_behaviour_update->vx = 0;
					chassis_behaviour_update->vy = 0;
					chassis_behaviour_update->vz = 250.0f;				
				}		
			}
			else
			{  
				switch(bit1)
				{
					case 1:		autoX = 0.0f;autoY = 176.0f;break;
					case 2:		autoX = 185.0f;autoY = 412.0f;break;
					case 3:		autoX = 185.0f;autoY = 484.0f;break;
					default:break ;
				}
				switch(bit1)
				{
					case 0:	if(Distances.Real_Distance_Y>-3.0f&&Distances.Real_Distance_Y<3.0f)bit1++;break;
					case 1:	if(Distances.Real_Distance_Y>173.0f&&Distances.Real_Distance_Y<179.0f)bit1++;break;
					case 2:	if(Distances.Real_Distance_Y>409.0f&&Distances.Real_Distance_Y<415.0f)bit1++;break;
					case 3:	if(Distances.Real_Distance_Y>482.0f&&Distances.Real_Distance_Y<486.0f)
						{
							bit1++;
							chassis_return = 1;
							break;
						}
					default:break ;
				}
			chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
			chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
			chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);
			}
		  mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}
		else ;		
	}
	else if(chassis_behaviour_update->chassis_mode == CHASSIS_ARTIFICIAL)//云台手控制
	{
		if(bisai_start == 1 || shoot_state == 1)
		{	
			if(color == 1)//红
			{
				if(chassis_behaviour_update->chassis_command->commd_keyboard == 'Q')
				{
					autoX = 747 - chassis_behaviour_update->chassis_command->target_position_y * 100 ;
					autoY = chassis_behaviour_update->chassis_command->target_position_x * 100 - 1259;
//					autoX = 354 - chassis_behaviour_update->chassis_command->target_position_y * 100 ;
//					autoY = chassis_behaviour_update->chassis_command->target_position_x * 100 - 1259;
					chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
					chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
					chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);
					dodge = 1;
				}
				else if(chassis_behaviour_update->chassis_command->commd_keyboard == 'R')
				{
					chassis_behaviour_update->vx = 0;
					chassis_behaviour_update->vy = 0;
					chassis_behaviour_update->vz = 250.0f; 
					dodge = 1;
					gumbal_judge.Start_Dodge = 1;
				}
//				else if(chassis_behaviour_update->chassis_command->commd_keyboard == 'T')
//				{									
//					if(fabs(chassis_behaviour_update->chassis_angle_pid.error[0]) < 0.1 && dodge == 1)
//					{
//						dodge = 0;
//						CAN_Send_Positioning();
//						delay_ms(500);
//					}
//					gumbal_judge.Start_Dodge = 0;
//				}
				else 
				{
					chassis_behaviour_update->vx = chassis_behaviour_update->RC_X_ChassisSpeedRef;
					chassis_behaviour_update->vy = chassis_behaviour_update->RC_Y_ChassisSpeedRef;
					chassis_behaviour_update->vz = 0;				
				}
			}
			else if(color == 0)//蓝
			{
				if(chassis_behaviour_update->chassis_command->commd_keyboard == 'Q')
				{
					autoX = chassis_behaviour_update->chassis_command->target_position_y * 100 - 600;
					autoY = 3492 - chassis_behaviour_update->chassis_command->target_position_x * 100;
//					autoX = chassis_behaviour_update->chassis_command->target_position_y * 100 - 747;
//					autoY = 1386 - chassis_behaviour_update->chassis_command->target_position_x * 100;
					chassis_behaviour_update->vx = PID_Calc(&chassis_behaviour_update->chassis_X_pid, Distances.Real_Distance_X, autoX);
					chassis_behaviour_update->vy = PID_Calc(&chassis_behaviour_update->chassis_Y_pid, Distances.Real_Distance_Y, autoY);
					chassis_behaviour_update->vz = PID_Calc(&chassis_behaviour_update->chassis_Z_pid, chassis_behaviour_update->Chassis_Angular->YAW, autoZ);
					dodge = 1;
					gumbal_judge.Start_Dodge = 0;
				} 
				else if(chassis_behaviour_update->chassis_command->commd_keyboard == 'R')
				{
					chassis_behaviour_update->vx = 0;
					chassis_behaviour_update->vy = 0;
					chassis_behaviour_update->vz = 250.0f;
					dodge = 1;
					gumbal_judge.Start_Dodge = 1;
				}
//				else if(chassis_behaviour_update->chassis_command->commd_keyboard == 'T')
//				{
//					chassis_behaviour_update->vx = 0;
//					chassis_behaviour_update->vy = 0;
//					chassis_behaviour_update->vz = -PID_Calc(&chassis_behaviour_update->chassis_angle_pid, Positioning_t.Angle, 0);
//					if(fabs(chassis_behaviour_update->chassis_angle_pid.error[0]) < 0.1 && dodge == 1)
//					{
//						dodge = 0;
//						CAN_Send_Positioning();
//						delay_ms(500);
//					}
//					gumbal_judge.Start_Dodge = 0;
//				}
				else
				{
					chassis_behaviour_update->vx = chassis_behaviour_update->RC_X_ChassisSpeedRef;
					chassis_behaviour_update->vy = chassis_behaviour_update->RC_Y_ChassisSpeedRef;
					chassis_behaviour_update->vz = 0;				
				}
			}
			mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
		}
		else;
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

	if(power_ctrl->chassis_power_measure->chassis_power == 130)	
		MAX_WHEEL_RPM = 9000 ;
	else 
		MAX_WHEEL_RPM = 6000;
	
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
                VAL_LIMIT(power_ctrl->motor_speed_pid[i].out, -8000, 8000);
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
			power_ctrl->motor_speed_pid[0].out = out_speed_err[0]/toatl_speed_err* fTotalCurrentLimit;
			power_ctrl->motor_speed_pid[1].out = out_speed_err[1]/toatl_speed_err* fTotalCurrentLimit;
			power_ctrl->motor_speed_pid[2].out = out_speed_err[2]/toatl_speed_err* fTotalCurrentLimit;
			power_ctrl->motor_speed_pid[3].out = out_speed_err[3]/toatl_speed_err* fTotalCurrentLimit;
        }
    }
}

