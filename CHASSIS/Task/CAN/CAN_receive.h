/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
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

#ifndef CANTASK_H
#define CANTASK_H
#include "stm32f4xx.h"
#include "RemoteControl.h"

#define shoot_state 0


/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
		
		CAN_YAW_receive_ID = 0x511,
		CAN_RC_receive_ID = 0x512,
	
		CAN_JUDGE1_ID = 0x611,
		CAN_JUDGE2_ID = 0x612,

		CAN_Positioning = 0x310,
		CAN_MPU_receive_ID = 0x613,
} can_msg_id_e;
//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	  int32_t  count;
    int32_t all_ecd;
} motor_measure_t;

//全场定位
typedef struct
{
	int16_t Distance_Left;     //计算出来的距离
	int16_t Distance_Right;   //计算出来的距离
	float Angle;
	uint8_t RC_mes;
}Positioning;
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//发送裁判系统数据给上板
void CAN_JUDGE_1(void);
//接受遥控器数据
const RC_ctrl_t *get_can_remote_control_point(void);
void CAN_Send_Positioning(void);
#endif



