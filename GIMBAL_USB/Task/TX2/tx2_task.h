#ifndef __TX2_APP_H
#define __TX2_APP_H
#include "stm32f4xx.h"
#include <stdio.h>
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"

#include "get_judge_measure.h"
#include "CAN_Receive.h"
#include "RemoteControl.h"

//PC数据转换成期望结构，提取合成有用数据
typedef struct
{
	float pitch_angle_dynamic_refs;
	float yaw_angle_dynamic_refs;
	float PcPitch;      //上位机发过来的pitch轴数据
	float PcYaw;       //上位机发过来的yaw轴数据
	unsigned char shoot_flag;//装甲板识别指令
//	unsigned char left_flag;//左摄像头标志位
//	unsigned char right_flag;//右摄像头标志位
	unsigned char frequency_adj;//射频调整
	unsigned char head;
	float distance;
} PC_Data;

//原结构体
typedef __packed struct 
{
	unsigned char head;
	float angle_pitch;
	float angle_yaw;
	float distance;
	unsigned char shoot_flag;
	unsigned char frequency_adj;
	unsigned char end;
}PC_Ctrl_t;

typedef union
{
	fp32	p;
	uint8_t pitch_t[4];
}pitch_un;

typedef union
{
	fp32	y;
	uint8_t yaw_t[4];
}yaw_un;

typedef union
{
	fp32	D;
	uint8_t dist_t[4];
}dist_un;

typedef struct 
{
	uint8_t start;//开始
	uint8_t color;//颜色
	yaw_un 		yaw;
	pitch_un 	pitch;
	u8 			mode;
	u8 			speed;
	uint8_t end;//结束
	unsigned char CRC8;
}send_Tx2_t;

extern send_Tx2_t Send2PC_Sta;


//上位机数据转换共用体
typedef union 
{
	PC_Ctrl_t PcDate;
	unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
}PC_Ctrl_Union_t;

typedef	union PcSend 
{
	uint8_t bit8[4];
	float raw;
} SendIMUData_t;

typedef struct
{
	const RC_ctrl_t *RC_pc;
	float now_pc_yaw_ref;
	float now_pc_pitch_ref;
	float last_pc_yaw_ref;
	float last_pc_pitch_ref;
} Inter_Data_t;

//通过指针方式获取原始数据
extern const PC_Data *get_PC_Data_Point(void);
send_Tx2_t *get_Send2PC_Point(void);
void MinipcDatePrcess(PC_Ctrl_t *pData);
void minipc_Get_Message(void);
void Interactive_task(void  *pvParameters);
static void minipc_Send_Message(USART_TypeDef* USARTx, send_Tx2_t *TXmessage);
void task_Tx2_Create(void);
void task_PC_Create(void);
void tx2_error(void);
#endif


