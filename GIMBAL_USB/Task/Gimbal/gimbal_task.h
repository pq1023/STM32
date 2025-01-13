#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"
#include "judgement_info.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "tx2_task.h"
#include "Detect_Task.h"


//归中设置
#define Glimbal_Yaw_Offset  							5381
#define Glimbal_Pitch_Offset 							2106
#define 	PITCH_MIN  			-12							//抬头
#define 	PITCH_MAX   		 22							//低头
#define PITCH_TURN   -1		//PITCH轴电机是否装反

/*pitch轴电机正向*/
#define PIT_MOTO_POSITIVE_DIR  		1.0f
/*yaw轴电机正方向*/
#define YAW_MOTO_POSITIVE_DIR  1.0f
/*电机ecd值转化成角度值8*/
#define ENCODER_ANGLE_RATIO    	(8192.0f/360.0f)
//yaw,pitch控制通道以及状态开关通道
#define 	YawChannel 			2
#define 	PitchChannel 		3
//遥控器控制参数改变，降低遥控器幅度
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.00015f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.00025f
/***************************************************************************/
//初始化pitch 角度环 PID参数
#define PITCH_INIT_PID_KP					60.0f
#define PITCH_INIT_PID_KI						0.0f
#define PITCH_INIT_PID_KD					0.0f
#define PITCH_PID_MAX_OUT 									29999.0f
#define PITCH_PID_MAX_IOUT 								8000.0f
//初始化pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 			20.0f
#define PITCH_SPEED_PID_KI 				0.0f
#define PITCH_SPEED_PID_KD 			0.0f
#define PITCH_SPEED_PID_MAX_OUT 		29999.0f
#define PITCH_SPEED_PID_MAX_IOUT 		5000.0f
//初始化yaw 角度环 PID参数
#define YAW_INIT_PID_KP						35.0f
#define YAW_INIT_PID_KI							0.0f
#define YAW_INIT_PID_KD						0.0f
#define YAW_PID_MAX_OUT 										29999.0f
#define YAW_PID_MAX_IOUT 									10000.0f
//初始化yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 				180.0f
#define YAW_SPEED_PID_KI 					0.0f
#define YAW_SPEED_PID_KD 				0.0f
#define YAW_SPEED_PID_MAX_OUT 			29999.0f
#define YAW_SPEED_PID_MAX_IOUT 			8000.0f

//自动巡航模式云台速度
#define GIMBAL_AUTO_PITCH_DEL             0.03
#define GIMBAL_AUTO_YAW_DEL               0.045

typedef struct
{
	const motor_measure_t *gimbal_motor_measure;//电机反馈
	
	PidTypeDef gimbal_speed_pid;
	PidTypeDef gimbal_angle_pid;
	uint16_t offset_ecd;
	fp32 relative_angle;
	fp32 gimbal_angle_set;
	fp32 gimbal_ramp_set;
	int16_t given_current;  //这个值直接传给电机
	fp32 last_angle;
	fp32 now_angle;
}Gimbal_Motor_t;

typedef enum
{
	GIMBAL_ZERO_FORCE = 0, 		//云台无力 0
	GIMBAL_INIT,           		//云台初始化(归中) 1
	GIMBAL_MANUAL_MODE,   	 	//云台手动控制 2
	GIMBAL_TRACK_ARMOR,				//自动自瞄 3
	GIMBAL_RC_CTRL_ARMOR,			//手控自瞄	4
}gimbal_mode_e;

typedef struct
{
	const monitor_t *gimbal_monitor_point;//监测系统的指针
	const Angular_Handle *Gimbal_Angular;//陀螺仪数据
	const RC_ctrl_t *gimbal_rc_ctrl;//遥控器数据
	
	const PC_Data        *gimbal_pc_data;//视觉数据获取
	gimbal_mode_e 	gimbal_mode;//云台模式
	gimbal_mode_e		gimbal_last_mode;
	Gimbal_Motor_t gimbal_yaw_motor;
	Gimbal_Motor_t gimbal_pitch_motor;
	
	FFC gimbal_FFC;
	
	float pitch_angle_dynamic_ref;			//角度动态输入
	float yaw_angle_dynamic_ref;
	fp32 auto_pitch_del;//巡逻
	fp32 auto_yaw_del;
}gimbal_control_state;
extern Judge_Receive_t judge_receive;
void task_Gimbal_Create(void);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
//float FeedforwardController(FFC *vFFC);
#endif


