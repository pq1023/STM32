#include "shoot_task.h"
#include "Smooth_Filter.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define SHOOT_TASK_PRIO 22
#define SHOOT_STK_SIZE 512
TaskHandle_t ShootTask_Handler;

#define shoot_free_time 50
Shoot_Motor_State shoot_motor;//static
SmoothFilter_t friction_smooth;
void Shoot_task(void);
static void Shoot_Init(Shoot_Motor_State *shoot_init);
static void shoot__behavour_set(Shoot_Motor_State* shoot_ctrl_init);
static void shoot_mode_control(Shoot_Motor_State* shoot_mo);
static void shoot_down_control(Shoot_Motor_State* shoot_down);
static void SHOOT_Rc(Shoot_Motor_State* shoot_rc);
static void SHOOT_Auto(Shoot_Motor_State* shoot_auto);
static void shoot_ctrl_limit(Shoot_Motor_State* shoot_limits);
static void friction_shoot_control(Shoot_Motor_State* shoot_control);
static void trigger_speed_control(Shoot_Motor_State* speed_control);
static void shoot_gight_stuck_bullet(Shoot_Motor_State *shoot_stuck);
static void shoot_left_stuck_bullet(Shoot_Motor_State *shoot_left_stuck);
void task_Shoot_Create(void)
{
	xTaskCreate((TaskFunction_t)Shoot_task,
                (const char *)"Shoot_task",
                (uint16_t)SHOOT_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SHOOT_TASK_PRIO,
                (TaskHandle_t *)&ShootTask_Handler);
}
SmoothFilter_t friction_smooth;
fp32 shoot_motor_left_up, shoot_motor_left_down;
fp32 shoot_motor_right_up, shoot_motor_right_down;
void Shoot_task(void)
{
	vTaskDelay(shoot_free_time);
	Shoot_Init(&shoot_motor);
	while(1)
	{
		shoot__behavour_set(&shoot_motor);
		shoot_mode_control(&shoot_motor);
//		shoot_motor_left_up = SmoothFilter_Calc(&friction_smooth, shoot_motor.friction_motor_left_up_pid.out);
//		shoot_motor_left_down = SmoothFilter_Calc(&friction_smooth, shoot_motor.friction_motor_left_down_pid.out);
//		shoot_motor_right_up = SmoothFilter_Calc(&friction_smooth, shoot_motor.friction_motor_right_up_pid.out);
//		shoot_motor_right_down = SmoothFilter_Calc(&friction_smooth, shoot_motor.friction_motor_right_down_pid.out);
//		if(shoot_motor.shoot_mode == SHOOT_STOP_MODE)
//		{
//			CAN_Friction(0,0,0,0);
//			CAN_Trigger(0,0);
//		}
//		else
//		{
			CAN_Friction(shoot_motor.friction_motor_left_up_pid.out, shoot_motor.friction_motor_left_down_pid.out,
									shoot_motor.friction_motor_right_up_pid.out, shoot_motor.friction_motor_right_down_pid.out);
			CAN_Trigger((int16_t)shoot_motor.trigger_right_current, (int16_t)shoot_motor.trigger_left_current);		
//			CAN_Trigger(0,0);
//			CAN_Friction(0,0,0,0);
			
//		}
		vTaskDelay(1);
	}
}
int stuck_right = 0, stuck_left = 0;
int stuck_right_time = 0, stuck_left_time = 0;
static void Shoot_Init(Shoot_Motor_State *shoot_init)
{
	if (shoot_init == NULL)
	{
			return;
	}	
	
	SmoothFilter_Init(&friction_smooth, 20);
	
	fp32 Trigger_left_pid[3] = {TRIGGER_LEFT_PID_KP, TRIGGER_LEFT_PID_KI, TRIGGER_LEFT_PID_KD};
	PID_Init(&shoot_init->trigger_motor_left_pid, PID_POSITION, Trigger_left_pid, TRIGGER_LEFT_PID_MAX_OUT, TRIGGER_LEFT_PID_MAX_IOUT);
	fp32 Trigger_right_pid[3] = {TRIGGER_RIGHE_PID_KP, TRIGGER_RIGHT_PID_KI, TRIGGER_RIGHT_PID_KD};
	PID_Init(&shoot_init->trigger_motor_right_pid, PID_POSITION, Trigger_right_pid, TRIGGER_RIGHT_PID_MAX_OUT, TRIGGER_RIGHT_PID_MAX_IOUT);
	
	fp32 friction_left_pid[3] = {FRICTION_LEFT_PID_KP, FRICTION_LEFT_PID_KI, FRICTION_LEFT_PID_KD};
	PID_Init(&shoot_init->friction_motor_left_up_pid, PID_POSITION, friction_left_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
	PID_Init(&shoot_init->friction_motor_left_down_pid, PID_POSITION, friction_left_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
	fp32 friction_right_pid[3] = {FRICTION_RIGHT_PID_KP, FRICTION_RIGHT_PID_KI, FRICTION_RIGHT_PID_KD};
	PID_Init(&shoot_init->friction_motor_right_up_pid, PID_POSITION, friction_right_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
	PID_Init(&shoot_init->friction_motor_right_down_pid, PID_POSITION, friction_right_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
	
	shoot_init->trigger_motor_right_measure = get_Trigger_Motor_Measure_Point(0);
	shoot_init->trigger_motor_left_measure = get_Trigger_Motor_Measure_Point(1);	
	for(uint8_t i = 0; i < 2; i++)
	{
		shoot_init->friction_motor_left_measure[i] = get_Friction_left_Motor_Measure_Point(i);
	}
	for(uint8_t m = 0; m < 2; m++)
	{
		shoot_init->friction_motor_right_measure[m] = get_Friction_right_Motor_Measure_Point(m);
	}
	shoot_init->shoot_rc_ctrl = get_remote_control_point();	//获取遥控器指针
	shoot_init->shoot_pc_data = get_PC_Data_Point();    //PC数据指针获取		
	shoot_init->shoot_motor_yaw = get_yaw_motor_point();
	shoot_init->shoot_mode = SHOOT_STOP_MODE;
	shoot_init->trigger_left_set_speed = 0;
	shoot_init->trigger_left_current = 0;
	shoot_init->trigger_right_set_speed = 0;
	shoot_init->trigger_right_current = 0;
	
	shoot_init->friction_left_speed = 0;
	shoot_init->friction_left_speed_set = 0;
	shoot_init->friction_right_speed_set = 0;
	shoot_init->friction_right_speed = 0;
}

static void shoot__behavour_set(Shoot_Motor_State* shoot_ctrl_init)
{
	if (shoot_ctrl_init == NULL)
	{
			return;
	}	
	
	if(switch_is_mid(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右下手动
		shoot_ctrl_init->shoot_mode = SHOOT_RC_MODE;
//		shoot_ctrl_init->shoot_mode = SHOOT_AUTO_CONTROL;
	}	
	else if(switch_is_mid(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//双中手动
		shoot_ctrl_init->shoot_mode = SHOOT_RC_MODE;
	}
	else if(switch_is_mid(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左中右上
		shoot_ctrl_init->shoot_mode = SHOOT_AUTO_CONTROL;
//		shoot_ctrl_init->shoot_mode = SHOOT_RC_MODE;
	}
	else if(switch_is_up(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右下
//		shoot_ctrl_init->shoot_mode = SHOOT_AUTO_CONTROL;
		shoot_ctrl_init->shoot_mode = SHOOT_RC_MODE;
	}		
	else if(switch_is_up(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_mid(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//左上右中自动		
//		shoot_ctrl_init->shoot_mode = SHOOT_AUTO_CONTROL;
		shoot_ctrl_init->shoot_mode = SHOOT_RC_MODE;
	}
	else if(switch_is_up(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_R]))
	{
		//双上自动		
//		shoot_ctrl_init->shoot_mode = SHOOT_AUTO_CONTROL;
		shoot_ctrl_init->shoot_mode = SHOOT_RC_MODE;
	}	
	else if(switch_is_down(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(shoot_ctrl_init->shoot_rc_ctrl->rc.s[ModeChannel_R]))//(双下)停止指令
	{
	 //双下
	 shoot_ctrl_init->shoot_mode = SHOOT_STOP_MODE;
	}
	else
	{
	 shoot_ctrl_init->shoot_mode = SHOOT_STOP_MODE;		
	}
}

static void shoot_mode_control(Shoot_Motor_State* shoot_mo)
{
	switch(shoot_mo->shoot_mode)
	{
		case SHOOT_STOP_MODE:
			shoot_down_control(shoot_mo);
		break;
		case SHOOT_RC_MODE:
			SHOOT_Rc(shoot_mo);
		break;
		case SHOOT_AUTO_CONTROL:
			SHOOT_Auto(shoot_mo);
		break;
		default :
			Shoot_Init(shoot_mo);
		break;
	}
}

/*============================枪口热量限制================================*/
static void shoot_ctrl_limit(Shoot_Motor_State* shoot_limits)
{
	u16 Heat_Rec_1, Heat_Rec_2;
	u8 Heat_Flag_1 = 1, Heat_Flag_2 = 1;
	uint16_t Real_heat_1 = 0, Real_heat_2 = 0;
	uint16_t Shoot_heat_limit = 240;
	uint16_t D_value_1 = 0, D_value_2 = 0;

	Real_heat_1 = judge_receive.Heat_1;//真实热量
	Real_heat_2 = judge_receive.Heat_2;
	D_value_1 = Shoot_heat_limit - Real_heat_1;
	D_value_2 = Shoot_heat_limit - Real_heat_2;
	if(D_value_1 < 40)
	{
		shoot_limits->trigger_left_set_speed = 0;
		trigger_speed_control(shoot_limits);
		Heat_Flag_1 = 0;
		Heat_Rec_1++;
	}
	else if(Heat_Flag_1 ==1 || Heat_Rec_1 > 300)
	{
		trigger_speed_control(shoot_limits);					
		Heat_Rec_1 = 0;		
	}
	
	if(D_value_2 < 40)
	{
		shoot_limits->trigger_right_set_speed = 0;
		trigger_speed_control(shoot_limits);
		Heat_Rec_2 = 0;
		Heat_Flag_2++;
	}
	else if(Heat_Flag_2 ==1 || Heat_Rec_2 > 300)
	{
		trigger_speed_control(shoot_limits);					
		Heat_Rec_2 = 0;				
	}
}

/*============================自动射击================================*/
int b;
int kin = 0;
int min = 0;
uint16_t tigger_right_speed = 3000, friction_speed = 7000, tigger_left_speed = 3000;
uint16_t friction_speed2 = 2000;
static void SHOOT_Auto(Shoot_Motor_State* shoot_auto)
{
//	if(judge_receive.GameSta == 4 || shoot_state == 1)
//	{
//		shoot_auto->friction_left_speed_set = friction_speed2;
//		shoot_auto->friction_right_speed_set = friction_speed2;
//		friction_shoot_control(shoot_auto);
	if(shoot_auto->shoot_pc_data->shoot_flag == 1)
	{
		shoot_auto->friction_left_speed_set = friction_speed;
		shoot_auto->friction_right_speed_set = friction_speed;
		friction_shoot_control(shoot_auto);
		if(shoot_auto->shoot_pc_data->frequency_adj == 1)
		{
			shoot_auto->trigger_left_set_speed = tigger_left_speed;
			if(b>100)
				shoot_auto->trigger_right_set_speed = tigger_right_speed;
			b++;
			shoot_gight_stuck_bullet(shoot_auto);
			shoot_left_stuck_bullet(shoot_auto);	
			shoot_ctrl_limit(shoot_auto);
		}
		else
		{
			shoot_auto->trigger_left_current = 0;
			shoot_auto->trigger_right_current = 0;		
			b=0;
			stuck_right_time = 0;
			stuck_right = 0;
			stuck_left_time = 0;
			stuck_left = 0;
			min = 1;
		}
		shoot_ctrl_limit(shoot_auto);
	}
	else 
	{
		shoot_down_control(shoot_auto);
		min = 2;
	}		
}

/*============================遥控器控制射击================================*/
int a=0;
static void SHOOT_Rc(Shoot_Motor_State* shoot_rc)
{
	//遥控射击控制->遥控拨轮向下滚动触发射击
	shoot_rc->friction_left_speed_set = friction_speed2;
	shoot_rc->friction_right_speed_set = friction_speed2;
	friction_shoot_control(shoot_rc);
	if(shoot_rc->shoot_rc_ctrl->rc.ch[4] > 0)
	{
		shoot_rc->friction_left_speed_set = friction_speed;
		shoot_rc->friction_right_speed_set = friction_speed;
		friction_shoot_control(shoot_rc);
		if(shoot_rc->shoot_rc_ctrl->rc.ch[4] >= 300)
		{
			shoot_rc->trigger_left_set_speed = tigger_left_speed;
			if(a>=120)
			{
				shoot_rc->trigger_right_set_speed = tigger_right_speed;	
			}
			a++;
			shoot_gight_stuck_bullet(shoot_rc);
			shoot_left_stuck_bullet(shoot_rc);	
		}
		else if(shoot_rc->shoot_rc_ctrl->rc.ch[4] < 300)
		{
			shoot_rc->trigger_left_set_speed = 0;
			shoot_rc->trigger_right_set_speed = 0;	
			stuck_right_time = 0;
			stuck_right = 0;
			stuck_left_time = 0;
			stuck_left = 0;	
			a=0;
		}		
		shoot_ctrl_limit(shoot_rc);		
	}
	else if(shoot_rc->shoot_rc_ctrl->rc.ch[4] <= 0 && shoot_rc->shoot_rc_ctrl->rc.ch[4] >= -660)
	{
		shoot_rc->trigger_left_current = 0;
		shoot_rc->trigger_right_current = 0;	
		stuck_right_time = 0;
		stuck_right = 0;
		stuck_left_time = 0;
		stuck_left = 0;
	}
}

/*==================================摩擦轮单环遥控器控制================================*/
static void friction_shoot_control(Shoot_Motor_State* shoot_control)
{
	if (shoot_control == NULL)
	{
			return;
	}
	shoot_control->friction_left_speed = shoot_control->friction_left_speed_set;
	shoot_control->friction_right_speed = shoot_control->friction_right_speed_set;
	
	PID_Calc(&shoot_control->friction_motor_left_up_pid, shoot_control->friction_motor_left_measure[FRICTION_UP]->speed_rpm, shoot_control->friction_left_speed);
	PID_Calc(&shoot_control->friction_motor_left_down_pid, shoot_control->friction_motor_left_measure[FRICTION_DOWN]->speed_rpm, -shoot_control->friction_left_speed);	
	PID_Calc(&shoot_control->friction_motor_right_up_pid, shoot_control->friction_motor_right_measure[FRICTION_UP]->speed_rpm, -shoot_control->friction_right_speed);
	PID_Calc(&shoot_control->friction_motor_right_down_pid, shoot_control->friction_motor_right_measure[FRICTION_DOWN]->speed_rpm, shoot_control->friction_right_speed);
	
}

/*==================================拨弹轮单环速度控制==================================*/
static void trigger_speed_control(Shoot_Motor_State* speed_control)
{
    speed_control->trigger_left_current = PID_Calc(&speed_control->trigger_motor_left_pid, speed_control->trigger_motor_left_measure->speed_rpm,
																					speed_control->trigger_left_set_speed);
    speed_control->trigger_right_current = PID_Calc(&speed_control->trigger_motor_right_pid, speed_control->trigger_motor_right_measure->speed_rpm,
																					-speed_control->trigger_right_set_speed);
}

/*************停止射击控制**************/
static void shoot_down_control(Shoot_Motor_State* shoot_down)
{
	shoot_down->friction_left_speed = 0;
	shoot_down->friction_right_speed = 0;
	shoot_down->trigger_right_set_speed = 0;
	shoot_down->trigger_left_set_speed = 0;
	
	PID_Calc(&shoot_down->friction_motor_left_up_pid, shoot_down->friction_motor_left_measure[FRICTION_UP]->speed_rpm, shoot_down->friction_left_speed);
	PID_Calc(&shoot_down->friction_motor_left_down_pid, shoot_down->friction_motor_left_measure[FRICTION_DOWN]->speed_rpm, shoot_down->friction_left_speed);	
	PID_Calc(&shoot_down->friction_motor_right_up_pid, shoot_down->friction_motor_right_measure[FRICTION_UP]->speed_rpm, shoot_down->friction_right_speed);
	PID_Calc(&shoot_down->friction_motor_right_down_pid, shoot_down->friction_motor_right_measure[FRICTION_DOWN]->speed_rpm, shoot_down->friction_right_speed);
	
	shoot_down->trigger_left_current = PID_Calc(&shoot_down->trigger_motor_left_pid, shoot_down->trigger_motor_left_measure->speed_rpm, shoot_down->trigger_left_set_speed);
	shoot_down->trigger_right_current = PID_Calc(&shoot_down->trigger_motor_right_pid, shoot_down->trigger_motor_right_measure->speed_rpm, shoot_down->trigger_right_set_speed);;
}

/*************卡弹处理**************/	
static void shoot_gight_stuck_bullet(Shoot_Motor_State *shoot_right_stuck)
{
	if(shoot_right_stuck->trigger_motor_right_measure->speed_rpm < 2000)
	{
		stuck_right_time++;
		if(stuck_right_time>2000 && stuck_right <= 0)
		{
			stuck_right = 500;
		}
		if(stuck_right > 0)
		{
			stuck_right--;
			shoot_right_stuck->trigger_right_set_speed = -3000;
			if(stuck_right == 0)
				stuck_right_time=0;
		}
	}
	else 
	{
		stuck_right_time = 0;
		stuck_right = 0;
	}
}

static void shoot_left_stuck_bullet(Shoot_Motor_State *shoot_left_stuck)
{
	if(shoot_left_stuck->trigger_motor_left_measure->speed_rpm < 2000)
	{
		stuck_left_time++;
		if(stuck_left_time>2000 && stuck_left <= 0)
		{
			stuck_left = 500;
		}
		if(stuck_left > 0)
		{
			stuck_left--;
			shoot_left_stuck->trigger_left_set_speed = -3000;
			if(stuck_left == 0)
				stuck_left_time=0;
		}
	}
	else 
	{
		stuck_left_time = 0;
		stuck_left = 0;
	}
}





