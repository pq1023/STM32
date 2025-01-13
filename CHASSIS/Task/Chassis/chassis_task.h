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
#include "infrared.h"

#define CHASSIS_X_CHANNEL 0//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 1//前后的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 4//分离模式下拨杆向上拨动，底盘旋转
#define RC_RESOLUTION     660.0f//遥控器分辨率
#define CHASSIS_KB_RC_MAX_SPEED		10000.0f//底盘最大速度
#define CHASSIS_RC_MAX_SPEED_X  CHASSIS_KB_RC_MAX_SPEED
#define CHASSIS_RC_MAX_SPEED_Y  CHASSIS_KB_RC_MAX_SPEED
#define CHASSIS_VZ_RC_SEN 0.005f//遥控器波轮（max 660）转化成车体左右速度（m/s）的比例

#define WHEELSPACING            259//车轮与中心距离
#define PERIMETER              502//轮子周长
#define RADIAN_COEF        57.3f//弧度系数
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)//3508减速比
#define ANGLE_TO_RAD   0.01745329251994329576923690768489f/*角度转化比例 */

//底盘速度PID
#define M3508_MOVE_KP	15.0f
#define M3508_MOVE_KI	0.0f
#define M3508_MOVE_KD	0.0f
#define M3505_MOVE_PID_MAX_OUT  			13000.0f
#define M3505_MOVE_PID_MAX_IOUT 			3000.0f
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 15.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  		2000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 		100.0f
//底盘全场定位PID
#define CHASSIS_AOUT_KP 10.0f
#define CHASSIS_AOUT_KI 0.0f
#define CHASSIS_AOUT_KD 0.0f
#define CHASSIS_AOUT_PID_MAX_OUT  		1000.0f
#define CHASSIS_AOUT_PID_MAX_IOUT 		100.0f

#define CHASSIS_X_KP 15.0f
#define CHASSIS_X_KI 0.0f
#define CHASSIS_X_KD 0.0f
#define CHASSIS_X_PID_MAX_OUT  		1000.0f
#define CHASSIS_X_PID_MAX_IOUT 		100.0f

#define CHASSIS_Y_KP 15.0f
#define CHASSIS_Y_KI 0.0f
#define CHASSIS_Y_KD 0.0f
#define CHASSIS_Y_PID_MAX_OUT  		1000.0f
#define CHASSIS_Y_PID_MAX_IOUT 		100.0f

#define CHASSIS_Z_KP 30.0f
#define CHASSIS_Z_KI 0.0f
#define CHASSIS_Z_KD 0.0f
#define CHASSIS_Z_PID_MAX_OUT  		1000.0f
#define CHASSIS_Z_PID_MAX_IOUT 		100.0f
typedef enum
{
	CHASSIS_RELAX          	= 0,//地盘无力模式	
	CHASSIS_INIT           	= 1,
	CHASSIS_FOLLOW_GIMBAL  	= 2,//底盘跟随云台
	CHASSIS_SEPARATE_GIMBAL   = 3,//底盘分离
	CHASSIS_DODGE_MODE        = 4,//小陀螺
	CHASSIS_AUTOMATIC					= 5,//自动
	CHASSIS_CHARGED						= 6,//无人机操控
	CHASSIS_SUICIDE						= 7,//自杀模式
	CHASSIS_ARTIFICIAL					= 8,//人工模式
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
	const ext_robot_command_t *chassis_command;//云台手发送的坐标位置
	const ext_game_robot_pos_t *chassis_self_location;//机器人自身位置
	const monitor_t *chassis_monitor_point;//裁判系统监控	
	const Angular_Handle *Chassis_Angular;//陀螺仪数据获取
	const RC_ctrl_t *chassis_rc_ctrl;//遥控器数据获取

	const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角	
	const sensor_Distance_t *infrared_distance;
	
	Chassis_Motor_t motor_chassis[4];          //底盘电机数据	
	chassis_mode_e chassis_mode;//底盘模式切换
	
	PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
	PidTypeDef chassis_angle_pid;              //底盘跟随角度pid
	PidTypeDef chassis_auto_pid;
	
	PidTypeDef chassis_X_pid;
	PidTypeDef chassis_Y_pid;
	PidTypeDef chassis_Z_pid;
	
	float RC_X_ChassisSpeedRef;			//左右动态输入
	float RC_Y_ChassisSpeedRef;			//前后动态输入
	float RC_Z_ChassisSpeedRef;			//旋转动态输入
	float chassis_relative_angle_set;
	float vx, vy, vz;
	float wheel_spd_ref[4];
	float	wheel_spd_fdb[4];	
	float nowadays_X,nowadays_Y,nowadays_Z;
	float outpost_hp;//前哨战血量
	
}chassis_move_state;

void task_Chsssis_Create(void);

#endif

