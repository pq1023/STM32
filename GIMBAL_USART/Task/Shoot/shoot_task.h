#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "sys.h"
//#include "chassis_task.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "Tx2_task.h"
#include "Detect_Task.h"
#include "gimbal_task.h"
//µ¥»·¿ØÖÆ×ó²¦µ¯ÂÖ
#define TRIGGER_LEFT_PID_KP 				10.0f
#define TRIGGER_LEFT_PID_KI 				0.0f
#define TRIGGER_LEFT_PID_KD 				0.0f
#define TRIGGER_LEFT_PID_MAX_OUT 		10000.0f
#define TRIGGER_LEFT_PID_MAX_IOUT 	2000.0f
//µ¥»·¿ØÖÆÓÒ²¦µ¯ÂÖ
#define TRIGGER_RIGHE_PID_KP 				10.0f
#define TRIGGER_RIGHT_PID_KI 				0.0f
#define TRIGGER_RIGHT_PID_KD 				0.0f
#define TRIGGER_RIGHT_PID_MAX_OUT 		10000.0f
#define TRIGGER_RIGHT_PID_MAX_IOUT 	2000.0f

//µ¥»·¿ØÖÆÄ¦²ÁÂÖ
#define FRICTION_LEFT_PID_KP 										30.0f
#define FRICTION_LEFT_PID_KI 											0.0f
#define FRICTION_LEFT_PID_KD 										0.0f
#define FRICTION_RIGHT_PID_KP 									30.0f
#define FRICTION_RIGHT_PID_KI 									0.0f
#define FRICTION_RIGHT_PID_KD 									0.0f
#define FRICTION_PID_MAX_OUT  								14000.0f
#define FRICTION_PID_MAX_IOUT 								3000.0f

//×óÓÒÄ¦²ÁÂÖ
#define FRICTION_UP	0
#define FRICTION_DOWN  1

typedef enum
{
	SHOOT_STOP_MODE,				//Í£Ö¹·¢Éä
	SHOOT_RC_MODE,		//·¢ÉäÒ£¿ØÆ÷¿ØÖÆ
	SHOOT_AUTO_CONTROL,		//·¢Éä×Ô¶¯¿ØÖÆ
} shoot_mode_e;

typedef struct
{
	PidTypeDef trigger_motor_left_pid;//²¦µ¯ÂÖPID	
	PidTypeDef trigger_motor_right_pid;//²¦µ¯ÂÖPID		
	PidTypeDef friction_motor_left_up_pid;
	PidTypeDef friction_motor_right_up_pid;
	PidTypeDef friction_motor_left_down_pid;
	PidTypeDef friction_motor_right_down_pid;	
	
	const motor_measure_t  *trigger_motor_left_measure;
	const motor_measure_t  *trigger_motor_right_measure;	
  const motor_measure_t  *friction_motor_left_measure[2];	
	const motor_measure_t  *friction_motor_right_measure[2];	
  const RC_ctrl_t 			  *shoot_rc_ctrl;	
	const PC_Data						*shoot_pc_data;//
	const monitor_t 			  *shoot_monitor_point;
	const Gimbal_Motor_t       *shoot_motor_yaw;
	const Gimbal_Motor_t       *shoot_motor_pitch;
	shoot_mode_e shoot_mode;
	
	fp32 trigger_left_set_speed;
	fp32 trigger_right_set_speed;
	fp32 trigger_left_current;
	fp32 trigger_right_current;	
	
	fp32 friction_left_speed_set;
	fp32 friction_right_speed_set;
	fp32 friction_left_speed;
	fp32 friction_right_speed;
	
}Shoot_Motor_State;

extern Judge_Receive_t judge_receive;
void task_Shoot_Create(void);

#endif

