#ifndef __CHASSIS_TASH_H
#define __CHASSIS_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"
#include "get_judge_measure.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "detect_task.h"
#include "gimbal_task.h"

#define CHASSIS_X_CHANNEL 0//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 1//前后的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 4//分离模式下拨杆向上拨动，底盘旋转
#define RC_RESOLUTION     660.0f//遥控器分辨率
#define CHASSIS_KB_RC_MAX_SPEED		10000.0f//底盘最大速度
#define CHASSIS_RC_MAX_SPEED_X  CHASSIS_KB_RC_MAX_SPEED
#define CHASSIS_RC_MAX_SPEED_Y  CHASSIS_KB_RC_MAX_SPEED
#define CHASSIS_VZ_RC_SEN 0.005f//遥控器波轮（max 660）转化成车体左右速度（m/s）的比例

#define WHEELSPACING            275//车轮与中心距离
#define PERIMETER              502//轮子周长
#define RADIAN_COEF        57.3f//弧度系数
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)//3508减速比
#define ANGLE_TO_RAD   0.01745329251994329576923690768489f/*角度转化比例 */

#define M3508_MOVE_KP	10.0f
#define M3508_MOVE_KI	0.0f
#define M3508_MOVE_KD	0.0f
#define M3505_MOVE_PID_MAX_OUT  			10000.0f
#define M3505_MOVE_PID_MAX_IOUT 			1000.0f
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 			15.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 				0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 			0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  		10000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 		100.0f

typedef enum
{
	CHASSIS_RELAX          	= 0,//地盘无力模式	
	CHASSIS_INIT           	= 1,
	CHASSIS_FOLLOW_GIMBAL  	= 2,//底盘跟随云台
	CHASSIS_SEPARATE_GIMBAL   = 3,//底盘分离
	CHASSIS_DODGE_MODE        = 4,//小陀螺
	
} chassis_mode_e; //底盘运行模式

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
} Chassis_Motor_t;

typedef struct
{
  const ext_power_heat_data_t *chassis_power_measure;//底盘裁判系统功率读取
  const ext_game_robot_state_t *chassis_status_measure;//底盘裁判系统功率读取
//  const ext_robot_hurt_t *chassis_hurt_type;//伤害状态数据
	const ext_game_state_t *chassis_game_state;//比赛开始标志	
	
	const monitor_t *chassis_monitor_point;//裁判系统监控	
	const Angular_Handle *Chassis_Angular;//陀螺仪数据获取
	const RC_ctrl_t *chassis_rc_ctrl;//遥控器数据获取

	const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
	const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角

	
	Chassis_Motor_t motor_chassis[4];          //底盘电机数据	
	chassis_mode_e chassis_mode;//底盘模式切换
	
	PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
	PidTypeDef chassis_angle_pid;              //底盘跟随角度pid
	
	float RC_X_ChassisSpeedRef;			//左右动态输入
	float RC_Y_ChassisSpeedRef;			//前后动态输入
	float RC_Z_ChassisSpeedRef;			//旋转动态输入
	float chassis_relative_angle_set;
	float vx, vy, vz;
	float            wheel_spd_ref[4];
	float						wheel_spd_fdb[4];	
	
}chassis_move_state;

void task_Chsssis_Create(void);

#endif

