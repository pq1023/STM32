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
#include "detect_task.h"
#include "RemoteControl.h"
#define shoot_state 1


/* CAN send and receive ID */
typedef enum
{
	CAN_PIT_MOTOR_ID = 0x206,

	CAN_TRIGGER_right_ID    = 0x205,
	CAN_TRIGGER_left_ID    = 0x206,	
	
	CAN_FRICTION_left_up_ID   = 0x201,
	CAN_FRICTION_left_down_ID    = 0x202,
	CAN_FRICTION_right_up_ID = 0x203,
	CAN_FRICTION_right_down_ID = 0x204,

	CAN_YAW_send_ID = 0x205,
	CAN_RC_SEND_ID = 0x512,
	CAN_JUDGE1_receive_ID = 0x611,
	CAN_JUDGE2_receive_ID = 0x612,

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

typedef struct
{
	uint8_t Color;
	uint16_t Heat_1;
	uint16_t Heat_2;	
	uint16_t ShootNum;
	uint8_t GameSta;
	uint8_t armor_hurt_id;
	uint8_t hurt_type;	
}Judge_Receive_t;

//发送射击拨盘数据
extern void CAN_Trigger(int16_t trigger_right, int16_t trigger_left);
//发送射击摩擦轮数据
extern void CAN_Friction(int16_t left_up, int16_t left_down, int16_t right_up, int16_t right_down);
//发送pitch电机
extern void CAN_Gimbal(int16_t rev, int16_t pitch);
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger,friction电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(uint8_t i);
extern const motor_measure_t *get_Friction_right_Motor_Measure_Point(uint8_t i);
extern const motor_measure_t *get_Friction_left_Motor_Measure_Point(uint8_t i);
//发送YAW电流值给下板
extern void CAN_SEND_YAW(int16_t yaw_current);
//发送遥控数据
extern void CAN_CMD_RC(RC_ctrl_t *rc_ctrl, uint8_t restart);

#endif



