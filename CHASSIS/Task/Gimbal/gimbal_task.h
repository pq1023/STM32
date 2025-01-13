#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"
#include "judgement_info.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "Detect_Task.h"

//归中设置
#define Glimbal_Yaw_Offset  							5381

/*yaw轴电机正方向*/
#define YAW_MOTO_POSITIVE_DIR  1.0f
/*电机ecd值转化成角度值8*/
#define ENCODER_ANGLE_RATIO    	(8192.0f/360.0f)

typedef struct
{
	const motor_measure_t *gimbal_motor_measure;//电机反馈
	uint16_t offset_ecd;
	fp32 relative_angle;
	int16_t given_current;  //这个值直接传给电机
}Gimbal_Motor_t;

typedef struct
{
	uint8_t Color;
	uint8_t GameSta;
	uint8_t Robot_type;
	uint16_t Heat_1;
	uint16_t Heat_2;	
	uint16_t ShootNum;
	uint32_t Shoot_Speed;
	const ext_robot_hurt_t *robot_hurt_type;
	uint8_t Start_Dodge;
	uint16_t outpost_HP;
	uint8_t patrol_flag;
}Gimbal_judge_state;

//extern Gimbal_judge_state gimbal_judge;
extern Gimbal_judge_state gumbal_judge;
typedef struct
{
	Gimbal_Motor_t gimbal_yaw_motor;
	const ext_robot_command_t *gimbal_command;
}gimbal_control_state;

void task_Gimbal_Create(void);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
#endif

