#ifndef _TX2_TASK_H_
#define	_TX2_TASK_H_
#include "stm32f4xx.h"
#include <stdio.h>
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"

typedef struct
{
	float pitch_angle_dynamic_refs;
	float yaw_angle_dynamic_refs;
	float PcPitch;      //上位机发过来的pitch轴数据
	float PcYaw;       //上位机发过来的yaw轴数据
	unsigned char shoot_flag;//射击指令
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
	unsigned char shoot_flag;
	unsigned char frequency_adj;
	unsigned char end;
	float distance;
}PC_Ctrl_t;


typedef struct 
{
	uint8_t start;//开始
	uint8_t speed_high;
	uint8_t speed_low;
	uint8_t color;//颜色
	uint8_t heat;
	uint8_t pitch[4];
	uint8_t yaw[4];
	uint8_t V_Z[2];
	uint8_t Outpost_State;
	uint8_t shoot_speed;//射速
	uint8_t end;//结束
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


//通过指针方式获取原始数据
extern const PC_Data *get_PC_Data_Point(void);
send_Tx2_t *get_Send2PC_Point(void);
void minipc_Get_Message(void);
void minipc_Get_Message_ISR(DataRevice *Buffer);
void minipc_Error_Out(void);
void minipc_Send_Message(void);
extern void Tx2_task(void  *pvParameters);

#endif


