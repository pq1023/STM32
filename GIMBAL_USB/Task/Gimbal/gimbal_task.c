#include "gimbal_task.h"
#include "chassis_task.h"
#include "sys.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "ADRC_core.h"
#include "ADRC_user.h"

#include "Ramp_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

#include "user_lib.h"
#include "Smooth_Filter.h"
#define gimbal_free_time 50

#define GIMBAL_TASK_PRIO 25
#define GIMBAL_STK_SIZE 512
TaskHandle_t GimbalTask_Handler;

gimbal_control_state gimbal_control;
void Gimbal_task(void);
static void Gimbal_Init(gimbal_control_state *gimbal_init);
static void Gimbal_Behavour(gimbal_control_state *gimbal_behavour);
static void GIMBAL_Feedback_Update(gimbal_control_state *gimbal_feedback_update);
static void GIMBAL_Behaviour_update(gimbal_control_state *gimbal_behaviour_update);
static void gimbal_PID_compute(gimbal_control_state *gimbal_pid_compute);
static void GIMBAL_Handoff_Pid(gimbal_control_state *handoff_pid);
static void gimbal_PID_init(gimbal_control_state *gimbal_pid_init);
static void gimbal_track_armor(gimbal_control_state *gimbal_motor_armor);
static void Gimbal_Rc_Ctrl_Armor(gimbal_control_state *gimbal_rc_ctrl_armor);
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);


void task_Gimbal_Create(void)
{
	xTaskCreate((TaskFunction_t)Gimbal_task,
                (const char *)"Gimbal_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GimbalTask_Handler);
}

int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0;
SmoothFilter_t Pitch_Motor_Smooth, Yaw_Motor_Smooth;
extern RC_ctrl_t rc_ctrl;
void Gimbal_task(void)
{
	vTaskDelay(gimbal_free_time);
	Gimbal_Init(&gimbal_control);
	while(1)
	{
		Gimbal_Behavour(&gimbal_control);
		GIMBAL_Feedback_Update(&gimbal_control);
		GIMBAL_Handoff_Pid(&gimbal_control);
		GIMBAL_Behaviour_update(&gimbal_control);
		
		Yaw_Can_Set_Current = SmoothFilter_Calc(&Yaw_Motor_Smooth, gimbal_control.gimbal_yaw_motor.given_current);
		Pitch_Can_Set_Current = SmoothFilter_Calc(&Pitch_Motor_Smooth, gimbal_control.gimbal_pitch_motor.given_current);
		CAN_Gimbal(Yaw_Can_Set_Current, Pitch_Can_Set_Current);
//		CAN_Gimbal(0,0);
 		CAN_CMD_RC(&rc_ctrl, 0);
		CAN_MPU();
		vTaskDelay(1);  //系统延时
	}
}

/**
  * @brief 云台初始化
  */
static void Gimbal_Init(gimbal_control_state *gimbal_init)
{	
	SmoothFilter_Init(&Pitch_Motor_Smooth, 16);
	SmoothFilter_Init(&Yaw_Motor_Smooth, 16);	
	
	gimbal_init->Gimbal_Angular = get_Gyro_Angle_Point();//陀螺仪数据指针获取
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();//yaw轴电机数据指针获取
	gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();//pitch轴电机数据指针获取
	
	gimbal_init->gimbal_pc_data = get_PC_Data_Point();	//PC数据指针获取

	gimbal_init->gimbal_rc_ctrl = get_remote_control_point();		//遥控器数据指针获取
	gimbal_init->gimbal_monitor_point = getErrorListPoint();	  //监测系统指针获取

	gimbal_init->gimbal_mode = GIMBAL_ZERO_FORCE;		//初始化电机模式	
   //归中初始化
	gimbal_control.gimbal_pitch_motor.offset_ecd = Glimbal_Pitch_Offset;
	gimbal_control.gimbal_yaw_motor.offset_ecd = Glimbal_Yaw_Offset;
	gimbal_init->auto_pitch_del   = GIMBAL_AUTO_PITCH_DEL;
	gimbal_init->auto_yaw_del     = GIMBAL_AUTO_YAW_DEL;		
	

	fp32 Pitch_angle_pid[3] = {15.0f, 0.0f, 0.0f};
	PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_angle_pid, PID_POSITION, Pitch_angle_pid, 29999, 16000);//pitch轴角度环初始化
	fp32 Pitch_speed_pid[3] = {100.0f, 0.0f, 0.0f};
	PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_speed_pid, PID_POSITION, Pitch_speed_pid, 29999, 16000);//pitch轴速度环初始化
		
	fp32 YAw_angle_pid[3] = {17.0f, 0.0f, 300.0f};
	PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_angle_pid, PID_POSITION, YAw_angle_pid, 29999, 16000);//pitch轴角度环初始化
	fp32 YAw_speed_pid[3] = {260.0f, 0.0f, 100.0f};
	PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_speed_pid, PID_POSITION, YAw_speed_pid, 29999, 16000);//pitch轴速度环初始化	
}

/**
  * @brief 云台模式切换
  */	
static void Gimbal_Behavour(gimbal_control_state *gimbal_behavour)
{
	if (gimbal_behavour == NULL)
	{
			return;
	}	
	static int count = 0;
	if(switch_is_down(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左下右中
		gimbal_behavour->gimbal_mode =  GIMBAL_INIT;//初始化
	}	
	else if(switch_is_mid(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右下
		gimbal_behavour->gimbal_mode =  GIMBAL_MANUAL_MODE;//底盘分离 云台遥控器控制
	}
	else if(switch_is_mid(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右中
		gimbal_behavour->gimbal_mode =  GIMBAL_MANUAL_MODE;//底盘跟随 云台遥控器控制
	}
	else if(switch_is_mid(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右上
//		gimbal_behavour->gimbal_mode =  GIMBAL_RC_CTRL_ARMOR;//底盘自动 云台手动自瞄
		gimbal_behavour->gimbal_mode =  GIMBAL_TRACK_ARMOR;
	}
	else if(switch_is_up(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右下
		gimbal_behavour->gimbal_mode =  GIMBAL_RC_CTRL_ARMOR;//底盘无力 云台手动自瞄
	}	
	else if(switch_is_up(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右中
		gimbal_behavour->gimbal_mode =  GIMBAL_RC_CTRL_ARMOR;//底盘无力 云台自动自瞄
	}
	else if(switch_is_up(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右上
		gimbal_behavour->gimbal_mode =  GIMBAL_TRACK_ARMOR;//底盘自动 云台自动自瞄
	}
	else if(switch_is_down(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//双下
		gimbal_behavour->gimbal_mode =  GIMBAL_ZERO_FORCE;//底盘无力 云台无力
	}	
	else
	{
		gimbal_behavour->gimbal_mode =  GIMBAL_ZERO_FORCE;//底盘无力 云台无力	
	}
	if(switch_is_down(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(gimbal_behavour->gimbal_rc_ctrl->rc.s[ModeChannel_R]))
	{
		if(gimbal_behavour->gimbal_rc_ctrl->rc.ch[0] <= -655 && gimbal_behavour->gimbal_rc_ctrl->rc.ch[1] <= -655 &&
				gimbal_behavour->gimbal_rc_ctrl->rc.ch[2] >= 655 && gimbal_behavour->gimbal_rc_ctrl->rc.ch[3] <= -655)
		{	//内八复位
			count++;
			if(count >= 2000) //避免误触
			{
				CAN_Trigger(0, 0);
				CAN_Friction(0, 0, 0, 0);
				CAN_Gimbal(0, 0);
				CAN_CMD_RC(0, 1);
				vTaskDelay(500);
				__set_FAULTMASK(1);//关闭所有中断
				NVIC_SystemReset();//软件复位
			}
		}
		else
			count = 0;
	}
}


float Last_yaw = 0;
/**
  * @brief 云台数据更新
  */	
static void GIMBAL_Feedback_Update(gimbal_control_state *gimbal_feedback_update)
{
	if (gimbal_feedback_update == NULL)
	{
			return;
	}	
	static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR / ENCODER_ANGLE_RATIO;
	static float pit_ecd_ratio = PIT_MOTO_POSITIVE_DIR / ENCODER_ANGLE_RATIO;

	gimbal_feedback_update->gimbal_pitch_motor.relative_angle = pit_ecd_ratio * get_relative_pos(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,	
																																				gimbal_feedback_update->gimbal_pitch_motor.offset_ecd); //编码器角度
	gimbal_feedback_update->gimbal_yaw_motor.relative_angle = yaw_ecd_ratio * get_relative_pos(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
																																				gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);//编码器角度
	
	//动态输入角度更新
	gimbal_feedback_update->pitch_angle_dynamic_ref += PITCH_TURN * (gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[PitchChannel] * STICK_TO_PITCH_ANGLE_INC_FACT);
	gimbal_feedback_update->yaw_angle_dynamic_ref += gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[YawChannel] * STICK_TO_YAW_ANGLE_INC_FACT;
	
	//限幅
	VAL_LIMIT(gimbal_feedback_update->pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX);
}

/**
  * @brief 云台PID切换
  */	
static void GIMBAL_Handoff_Pid(gimbal_control_state *handoff_pid)
{
	if(handoff_pid == NULL)
	{
		return;
	}
	if(handoff_pid->gimbal_last_mode != GIMBAL_ZERO_FORCE && handoff_pid->gimbal_mode == GIMBAL_ZERO_FORCE)
	{
		handoff_pid->gimbal_yaw_motor.given_current = 0;
		handoff_pid->gimbal_pitch_motor.given_current = 0;
	}
	else if(handoff_pid->gimbal_last_mode != GIMBAL_INIT && handoff_pid->gimbal_mode == GIMBAL_INIT)
	{
		//初始化模式
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_angle_pid, 11.0f, 0.00f, 600.0f, 0.0f);
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_speed_pid, 210.0f, 0.0f, 0.0f, 0);
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_angle_pid, 10.0f, 0.0f, 400.0f, 0.0);//14 0 400 0
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_speed_pid, 210.0f, 0.0f, 0.0f, 0);//260 0 200							
		handoff_pid->pitch_angle_dynamic_ref = handoff_pid->Gimbal_Angular->ROLL;
		handoff_pid->yaw_angle_dynamic_ref = handoff_pid->Gimbal_Angular->YAW;
	}
	else if(handoff_pid->gimbal_last_mode != GIMBAL_MANUAL_MODE && handoff_pid->gimbal_mode == GIMBAL_MANUAL_MODE)
	{
		//手动模式
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_angle_pid, 13.0f, 0.07f, 1200.0f, 0.4f);//11 0.05 1200 0.7
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_speed_pid, 200.0f, 0.0f, 0.0f, 0);//155 0 0 0
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_angle_pid, 12.0f, 0.05f, 1000.0f, 0.3);//14 0 400 0
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_speed_pid, 240.0f, 0.0f, 0.0f, 0);//260 0 200			
		handoff_pid->pitch_angle_dynamic_ref = handoff_pid->Gimbal_Angular->ROLL;
		handoff_pid->yaw_angle_dynamic_ref = handoff_pid->Gimbal_Angular->YAW;		
	}
 	else if(handoff_pid->gimbal_last_mode != GIMBAL_TRACK_ARMOR && handoff_pid->gimbal_mode == GIMBAL_TRACK_ARMOR)
	{
		//装甲板自瞄
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_angle_pid, 13.0f, 0.07f, 1200.0f, 0.4f);//11 0.05 1200 0.7
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_speed_pid, 200.0f, 0.0f, 0.0f, 0);//155 0 0 0
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_angle_pid, 12.0f, 0.05f, 1000.0f, 0.3);//14 0 400 0
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_speed_pid, 240.0f, 0.0f, 0.0f, 0);//260 0 200						
		handoff_pid->pitch_angle_dynamic_ref = handoff_pid->Gimbal_Angular->ROLL;
		handoff_pid->yaw_angle_dynamic_ref = handoff_pid->Gimbal_Angular->YAW;		
	}
	else if(handoff_pid->gimbal_last_mode != GIMBAL_RC_CTRL_ARMOR && handoff_pid->gimbal_mode == GIMBAL_RC_CTRL_ARMOR)
	{
	  	//手动自瞄
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_angle_pid, 10.4f, 0.05f, 2000.0f, 0.4f);//11 0.05 1200 0.7
		Pid_reset(&handoff_pid->gimbal_pitch_motor.gimbal_speed_pid, 140.0f, 0.0f, 0.0f, 0);//155 0 0 0
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_angle_pid, 12.0f, 0.05f, 1000.0f, 0.3);//14 0 400 0
		Pid_reset(&handoff_pid->gimbal_yaw_motor.gimbal_speed_pid, 240.0f, 0.0f, 0.0f, 0);//260 0 200			
		handoff_pid->pitch_angle_dynamic_ref = handoff_pid->Gimbal_Angular->ROLL;
		handoff_pid->yaw_angle_dynamic_ref = handoff_pid->Gimbal_Angular->YAW;		
	}
	handoff_pid->gimbal_last_mode = handoff_pid->gimbal_mode;
}

/**
  * @brief 云台状态更新
  */	

static void GIMBAL_Behaviour_update(gimbal_control_state *gimbal_behaviour_update)
{
	switch(gimbal_behaviour_update->gimbal_mode)
	{
		case GIMBAL_ZERO_FORCE:
			gimbal_behaviour_update->gimbal_yaw_motor.given_current = 0;
			gimbal_behaviour_update->gimbal_pitch_motor.given_current = 0;
		break;
		case GIMBAL_INIT:
			gimbal_behaviour_update->gimbal_pitch_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_behaviour_update->Gimbal_Angular->ROLL, 8.0f);
			gimbal_behaviour_update->gimbal_yaw_motor.gimbal_angle_set = RAMP_float(0.0f, -gimbal_behaviour_update->gimbal_yaw_motor.relative_angle, 8.0f);
			gimbal_PID_init(gimbal_behaviour_update);
		break;
		case GIMBAL_MANUAL_MODE:
			gimbal_behaviour_update->gimbal_pitch_motor.gimbal_angle_set = gimbal_behaviour_update->pitch_angle_dynamic_ref;
			gimbal_behaviour_update->gimbal_yaw_motor.gimbal_angle_set = gimbal_behaviour_update->yaw_angle_dynamic_ref;
			gimbal_PID_compute(gimbal_behaviour_update);
		break;
		case GIMBAL_TRACK_ARMOR:
			gimbal_track_armor(gimbal_behaviour_update);		
			gimbal_PID_compute(gimbal_behaviour_update);
		break;
		case GIMBAL_RC_CTRL_ARMOR:
			Gimbal_Rc_Ctrl_Armor(gimbal_behaviour_update);			
			gimbal_PID_compute(gimbal_behaviour_update);
		break;
		default :
			gimbal_behaviour_update->gimbal_yaw_motor.given_current = 0;
			gimbal_behaviour_update->gimbal_pitch_motor.given_current = 0;
		break;		
	}
}	 
float PC_PITCH = 0.5f , PC_YAW = 0.8f;
float yaw_patrol_MAX = 0  /*右*/, yaw_patrol_MIN = 0; /*左*/
float pitch_patrol_MAX = 0, pitch_patrol_MIN = 0;//低头		抬头
float yaw_min = 0.04;
int timer = 0;
float yaw_angle = 0;
//自动自瞄
float TIME = 0;
float distance = 0;
static void gimbal_track_armor(gimbal_control_state *gimbal_motor_armor)
{
	if(judge_receive.GameSta == 4 || shoot_state == 1)
	{
		if(gimbal_motor_armor->gimbal_pc_data->shoot_flag == 1)
		{
			TIME = 1000;
			timer = 0;
		}
		else 
		{
			if(judge_receive.Patrol_Flag == 0)
			{
				yaw_patrol_MAX = 150;//右
				yaw_patrol_MIN = -150;
				pitch_patrol_MIN = -14;
				pitch_patrol_MAX = 22;				
			}
			else if(judge_receive.Patrol_Flag == 1)//S
			{
				yaw_patrol_MAX = 150;//右
				yaw_patrol_MIN = -150;
				pitch_patrol_MIN = -14;
				pitch_patrol_MAX = 22;
			}
			else if(judge_receive.Patrol_Flag == 2)//D
			{
				yaw_patrol_MAX = 40;//右
				yaw_patrol_MIN = -160;
				pitch_patrol_MIN = -14;
				pitch_patrol_MAX = 22;				
			}
			else if(judge_receive.Patrol_Flag == 3)//F
			{
				yaw_patrol_MAX = 160;//右
				yaw_patrol_MIN = -40;
				pitch_patrol_MIN = -14;
				pitch_patrol_MAX = 22;				
			}
			
			if(gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set >= pitch_patrol_MAX && gimbal_motor_armor->auto_pitch_del > 0)
				gimbal_motor_armor->auto_pitch_del *= -1;  
			else if(gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set <= pitch_patrol_MIN && gimbal_motor_armor->auto_pitch_del < 0)
				gimbal_motor_armor->auto_pitch_del *= -1;		//pitch巡逻范围
			VAL_LIMIT(gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);		 				
			if(gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set >= yaw_patrol_MAX && gimbal_motor_armor->auto_yaw_del > 0)
				gimbal_motor_armor->auto_yaw_del *= -1;			
			else if(gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set <= yaw_patrol_MIN && gimbal_motor_armor->auto_yaw_del < 0)
				gimbal_motor_armor->auto_yaw_del *= -1;
			
			if(gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set <= yaw_patrol_MAX && gimbal_motor_armor->auto_yaw_del < 0)
			{
				gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set	= 2;
				gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set += gimbal_motor_armor->auto_yaw_del * 1.5f;					
			}
			else
			{
				gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set += gimbal_motor_armor->auto_pitch_del;
				gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set += gimbal_motor_armor->auto_yaw_del;				
			}
			
//			if(gimbal_motor_armor->gimbal_pc_data->left_flag == 1)
//			{
//				gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set - 90;
//			}
//			else if(gimbal_motor_armor->gimbal_pc_data->right_flag == 1)
//			{
//				gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set + 90;
//			}
//			else ;
			
			if(TIME>0)
				TIME--;
			else;			
		}
		if(TIME)
		{ 
			if(gimbal_motor_armor->gimbal_pc_data->shoot_flag == 1)
			{  
				gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_pc_data->PcPitch * PC_PITCH + gimbal_motor_armor->Gimbal_Angular->ROLL;
				gimbal_motor_armor->gimbal_pitch_motor.now_angle= gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set;
				gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set = (gimbal_motor_armor->Gimbal_Angular->YAW - gimbal_motor_armor->gimbal_pc_data->PcYaw * PC_YAW);
				gimbal_motor_armor->gimbal_yaw_motor.now_angle= gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set;
				VAL_LIMIT(gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);				
			}
			else
			{
				if(timer > 400 && timer<1000)
				{	
					yaw_angle += yaw_min;
					if(yaw_angle >= 20 && yaw_min>0)
						yaw_min *= -1;
					else if(yaw_angle <= -20 && yaw_min<0)
						yaw_min *= -1;
					if(gimbal_motor_armor->gimbal_pc_data->distance < 4 && gimbal_motor_armor->gimbal_pc_data->distance >= 0)
					{
						gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_pitch_motor.last_angle;
						gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_yaw_motor.last_angle;
					}
					else
					{
						gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_yaw_motor.last_angle + yaw_angle;
						gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_pitch_motor.last_angle;
					}
				}
				else
				{
					gimbal_motor_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_pitch_motor.last_angle;
					gimbal_motor_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_motor_armor->gimbal_yaw_motor.last_angle;
					timer++;
				}
			}
		}
		gimbal_motor_armor->gimbal_pitch_motor.last_angle = gimbal_motor_armor->gimbal_pitch_motor.now_angle;
		gimbal_motor_armor->gimbal_yaw_motor.last_angle = gimbal_motor_armor->gimbal_yaw_motor.now_angle;		
	}		
}

//手动自瞄
static void Gimbal_Rc_Ctrl_Armor(gimbal_control_state *gimbal_rc_ctrl_armor)
{
	if(gimbal_rc_ctrl_armor->gimbal_pc_data->shoot_flag == 1)
	{
		TIME = 1000;
		timer = 0;
	}
	else 
	{
//		timer = 0;
		gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set += PITCH_TURN * (gimbal_rc_ctrl_armor->gimbal_rc_ctrl->rc.ch[PitchChannel] * STICK_TO_PITCH_ANGLE_INC_FACT);
		gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set += gimbal_rc_ctrl_armor->gimbal_rc_ctrl->rc.ch[YawChannel] *STICK_TO_YAW_ANGLE_INC_FACT;			
		VAL_LIMIT(gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);		
		
//			if(gimbal_rc_ctrl_armor->gimbal_pc_data->left_flag == 1)
//			{
//				gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set - 90;
//			}
//			else if(gimbal_motor_armor->gimbal_pc_data->right_flag == 1)
//			{
//				gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set + 90;
//			}
//			else ;
		
		if(TIME>0)
			TIME--;
		else;			
	}
	if(TIME)
	{ 
		if(gimbal_rc_ctrl_armor->gimbal_pc_data->shoot_flag == 1)
		{
			gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_pc_data->PcPitch * PC_PITCH + gimbal_rc_ctrl_armor->Gimbal_Angular->ROLL;
			gimbal_rc_ctrl_armor->gimbal_pitch_motor.now_angle= gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set;
			gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set = (gimbal_rc_ctrl_armor->Gimbal_Angular->YAW - gimbal_rc_ctrl_armor->gimbal_pc_data->PcYaw * PC_YAW);
			gimbal_rc_ctrl_armor->gimbal_yaw_motor.now_angle= gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set;
			VAL_LIMIT(gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
			distance = gimbal_rc_ctrl_armor->gimbal_pc_data->distance;
		}
		else
		{
//			if(timer > 400 && timer<1000)
//			{	
//				yaw_angle += yaw_min;
//				if(yaw_angle >= 20 && yaw_min>0)
//					yaw_min *= -1; 
//				else if(yaw_angle <= -20 && yaw_min<0)
//					yaw_min *= -1;
//				if(gimbal_rc_ctrl_armor->gimbal_pc_data->distance < 4 && gimbal_rc_ctrl_armor->gimbal_pc_data->distance >= 0)
//				{
//					gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_pitch_motor.last_angle;
//					gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_yaw_motor.last_angle;
//					VAL_LIMIT(gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
//				}
//				else
//				{
//					gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_yaw_motor.last_angle + yaw_angle;
//					gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_pitch_motor.last_angle;
//					VAL_LIMIT(gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
//				}
//			}
//			else
//			{
				gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_pitch_motor.last_angle;
				gimbal_rc_ctrl_armor->gimbal_yaw_motor.gimbal_angle_set = gimbal_rc_ctrl_armor->gimbal_yaw_motor.last_angle;
				VAL_LIMIT(gimbal_rc_ctrl_armor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
//				timer++;
//			}				
		}
	}
	gimbal_rc_ctrl_armor->gimbal_pitch_motor.last_angle = gimbal_rc_ctrl_armor->gimbal_pitch_motor.now_angle;
	gimbal_rc_ctrl_armor->gimbal_yaw_motor.last_angle = gimbal_rc_ctrl_armor->gimbal_yaw_motor.now_angle;		

}

//云台初始化PID计算
static void gimbal_PID_init(gimbal_control_state *gimbal_pid_init)
{
	PID_Calc(&gimbal_pid_init->gimbal_pitch_motor.gimbal_angle_pid, gimbal_pid_init->Gimbal_Angular->ROLL, gimbal_pid_init->gimbal_pitch_motor.gimbal_angle_set);
	PID_Calc(&gimbal_pid_init->gimbal_pitch_motor.gimbal_speed_pid, gimbal_pid_init->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm, gimbal_pid_init->gimbal_pitch_motor.gimbal_angle_pid.out);
	gimbal_pid_init->gimbal_pitch_motor.given_current = gimbal_pid_init->gimbal_pitch_motor.gimbal_speed_pid.out;
	
	PID_Calc(&gimbal_pid_init->gimbal_yaw_motor.gimbal_angle_pid, gimbal_pid_init->gimbal_yaw_motor.relative_angle, gimbal_pid_init->gimbal_pitch_motor.gimbal_angle_set);
	PID_Calc(&gimbal_pid_init->gimbal_yaw_motor.gimbal_speed_pid, gimbal_pid_init->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm, gimbal_pid_init->gimbal_yaw_motor.gimbal_angle_pid.out);
	gimbal_pid_init->gimbal_yaw_motor.given_current = gimbal_pid_init->gimbal_yaw_motor.gimbal_speed_pid.out;		
}

//云台YAW轴PID计算
static void gimbal_PID_compute(gimbal_control_state *gimbal_pid_compute)
{
	PID_Calc(&gimbal_pid_compute->gimbal_pitch_motor.gimbal_angle_pid, gimbal_pid_compute->Gimbal_Angular->ROLL, gimbal_pid_compute->gimbal_pitch_motor.gimbal_angle_set);
	PID_Calc(&gimbal_pid_compute->gimbal_pitch_motor.gimbal_speed_pid, gimbal_pid_compute->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm, gimbal_pid_compute->gimbal_pitch_motor.gimbal_angle_pid.out);
	gimbal_pid_compute->gimbal_pitch_motor.given_current = gimbal_pid_compute->gimbal_pitch_motor.gimbal_speed_pid.out;
		
	PID_Calc(&gimbal_pid_compute->gimbal_yaw_motor.gimbal_angle_pid, gimbal_pid_compute->Gimbal_Angular->YAW, gimbal_pid_compute->gimbal_yaw_motor.gimbal_angle_set);
	PID_Calc(&gimbal_pid_compute->gimbal_yaw_motor.gimbal_speed_pid, -gimbal_pid_compute->Gimbal_Angular->V_Z, gimbal_pid_compute->gimbal_yaw_motor.gimbal_angle_pid.out);
	gimbal_pid_compute->gimbal_yaw_motor.given_current = -gimbal_pid_compute->gimbal_yaw_motor.gimbal_speed_pid.out;	
}

//得到电机相对角度
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;

    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}


