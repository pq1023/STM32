/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收_____底盘云台超级电容->CAN1_____发射机构->CAN2______
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020		     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */

#include "CAN_Receive.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1
#define SHOOT_CAN CAN2

//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
			  if(((ptr)->ecd-(ptr)->last_ecd )>4096)                                                 \
			  { (ptr)->count--;}                                                                     \
	       else if(((ptr)->ecd-(ptr)->last_ecd )<-4096)                                          \
			   {(ptr)->count++;}                                                                     \
		     (ptr)->last_ecd = (ptr)->ecd;                                 	                       \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
				 (ptr)->all_ecd =(ptr)->count*8192+(ptr)->ecd;                                         \
    }

//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }


//统一处理can接收函数
static void CAN1_hook(CanRxMsg *rx_message);
static void CAN2_hook(CanRxMsg *rx_message);
//声明电机变量
motor_measure_t motor_yaw, motor_pit, motor_trigger[2], motor_left_friction[2],motor_right_friction[2];
Judge_Receive_t judge_receive;

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_hook(&rx1_message);
    }
}

//can2中断
void CAN2_RX1_IRQHandler(void)
{
    static CanRxMsg rx2_message;

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}

//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN2_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
			case CAN_PIT_MOTOR_ID:
			{
				//处理电机数据宏函数
				get_gimbal_motor_measuer(&motor_pit, rx_message);
				//记录时间
				DetectHook(PitchGimbalMotorTOE);
				break;
			}
			case CAN_YAW_send_ID:
			{
				get_gimbal_motor_measuer(&motor_yaw, rx_message);
				break;			
			}
			case CAN_JUDGE1_receive_ID:
			{
				judge_receive.Color = rx_message->Data[0];
				judge_receive.GameSta = rx_message->Data[1];
				judge_receive.Heat_1 = rx_message->Data[2] << 8 | rx_message->Data[3];
				judge_receive.Heat_2 = rx_message->Data[4] << 8 | rx_message->Data[5];
				judge_receive.Start_Dodge = rx_message->Data[6];
				judge_receive.Patrol_Flag = rx_message->Data[7];
				break;
			}
			default:
			{
					break;
			}
    }
}

static void CAN1_hook(CanRxMsg *rx_message)
{
    switch(rx_message->StdId)
    {
			case CAN_TRIGGER_right_ID:
			case CAN_TRIGGER_left_ID:
			{
					static uint8_t i = 0;	
					i = rx_message->StdId - CAN_TRIGGER_right_ID;
					//处理电机数据宏函数
					get_motor_measure(&motor_trigger[i], rx_message);
					//记录时间
					DetectHook(TriggerMotorRTOE + i);
					break;
			}

			case CAN_FRICTION_left_up_ID:
			case CAN_FRICTION_left_down_ID:
			{
					static uint8_t i = 0;
					//处理电机ID号
					i = rx_message->StdId - CAN_FRICTION_left_up_ID;
					//处理电机数据宏函数
					get_motor_measure(&motor_left_friction[i], rx_message);
					//记录时间
					DetectHook(frictionmotorLUTOE + i);
					break;
			}
			case CAN_FRICTION_right_up_ID:
			case CAN_FRICTION_right_down_ID:
			{
					static uint8_t i = 0;
					//处理电机ID号
					i = rx_message->StdId - CAN_FRICTION_right_up_ID;
					//处理电机数据宏函数
					get_motor_measure(&motor_right_friction[i], rx_message);
					//记录时间
					DetectHook(frictionmotorRUTOE + i);
					break;
			}
			default:
			{
					break;
			}
    }
}


//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回trigger电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Trigger_Motor_Measure_Point(uint8_t i)
{
    return &motor_trigger[(i & 0X03)];
}
//返回Friction电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Friction_left_Motor_Measure_Point(uint8_t i)
{
    return &motor_left_friction[(i & 0x03)];
}

//返回Friction电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Friction_right_Motor_Measure_Point(uint8_t i)
{
    return &motor_right_friction[(i & 0x03)];
}
/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
***********************************************************************************************
*/
/***********************************************************************************************/

//发送射击拨盘数据
void CAN_Trigger(int16_t trigger_right, int16_t trigger_left)
{
    CanTxMsg SendCanTxMsg;
    SendCanTxMsg.StdId = 0x1FF;
    SendCanTxMsg.IDE = CAN_ID_STD;
    SendCanTxMsg.RTR = CAN_RTR_DATA;
    SendCanTxMsg.DLC = 0x08;
    SendCanTxMsg.Data[0] = trigger_right >> 8;
    SendCanTxMsg.Data[1] = trigger_right;
    SendCanTxMsg.Data[2] = trigger_left >> 8;
    SendCanTxMsg.Data[3] = trigger_left;
    SendCanTxMsg.Data[4] = 0;
    SendCanTxMsg.Data[5] = 0;
    SendCanTxMsg.Data[6] = 0;
    SendCanTxMsg.Data[7] = 0;

    CAN_Transmit(CAN1, &SendCanTxMsg);
}

//发送射击摩擦轮数据
void CAN_Friction(int16_t left_up, int16_t left_down, int16_t right_up, int16_t right_down)
{
	CanTxMsg Shoot_Friction;
	Shoot_Friction.StdId = 0x200;
	Shoot_Friction.IDE = CAN_ID_STD;
	Shoot_Friction.RTR = CAN_RTR_DATA;
	Shoot_Friction.DLC = 0x08;
	Shoot_Friction.Data[0] = left_up >> 8;
	Shoot_Friction.Data[1] = left_up;
	Shoot_Friction.Data[2] = left_down >> 8;
	Shoot_Friction.Data[3] = left_down;
	Shoot_Friction.Data[4] = right_up >> 8;
	Shoot_Friction.Data[5] = right_up;
	Shoot_Friction.Data[6] = right_down >> 8;
	Shoot_Friction.Data[7] = right_down;
	
	CAN_Transmit(CAN1, &Shoot_Friction);
}
void CAN_Gimbal(int16_t rev, int16_t pitch)			//pitch
{
	CanTxMsg GIMBAL_P;
	GIMBAL_P.StdId = 0x1FF;
	GIMBAL_P.IDE = CAN_ID_STD;
	GIMBAL_P.RTR = CAN_RTR_DATA;
	GIMBAL_P.DLC = 0x08;
	GIMBAL_P.Data[0] = rev >> 8;
	GIMBAL_P.Data[1] = rev;
	GIMBAL_P.Data[2] = pitch >> 8;
	GIMBAL_P.Data[3] = pitch;
	GIMBAL_P.Data[4] = 0;	
	GIMBAL_P.Data[5] = 0;
	GIMBAL_P.Data[6] = 0;
	GIMBAL_P.Data[7] = 0;	
	
	CAN_Transmit(CAN2, &GIMBAL_P);
}

////发送YAW电流值给下板
//void CAN_SEND_YAW(int16_t yaw_current)
//{
//	CanTxMsg YawMessage;
//	union
//	{
//		float f;
//		char c[4];
//	} Data1;
//	Data1.f = yaw_current;
//	YawMessage.StdId = CAN_YAW_send_ID;
//	YawMessage.IDE = CAN_ID_STD;
//	YawMessage.RTR = CAN_RTR_DATA;
//	YawMessage.DLC = 0x08;
//	
//	YawMessage.Data[0] = (unsigned char)(Data1.c[0]);
//	YawMessage.Data[1] = (unsigned char)(Data1.c[1]);
//	YawMessage.Data[2] = (unsigned char)(Data1.c[2]);
//	YawMessage.Data[3] = (unsigned char)(Data1.c[3]); 
//	YawMessage.Data[4] = 0;
//	YawMessage.Data[5] = 0;
//	YawMessage.Data[6] = 0;
//	YawMessage.Data[7] = 0;
//	
//	CAN_Transmit(CAN2, &YawMessage);
//}

//发送遥控数据
void CAN_CMD_RC(RC_ctrl_t *rc_ctrl, uint8_t restart)
{
	static uint8_t restart_sta = 0;
	
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_RC_SEND_ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	
	if(restart)
		restart_sta = 1;
	if(restart_sta)
	{
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = restart_sta;
	}
	else
	{
		TxMessage.Data[0] = rc_ctrl->rc.ch[0] >> 8;
		TxMessage.Data[1] = rc_ctrl->rc.ch[0];
		TxMessage.Data[2] = rc_ctrl->rc.ch[1] >> 8;
		TxMessage.Data[3] = rc_ctrl->rc.ch[1];
		TxMessage.Data[4] = rc_ctrl->rc.ch[4] >> 8;
		TxMessage.Data[5] = rc_ctrl->rc.ch[4];
		TxMessage.Data[6] = rc_ctrl->rc.s[0]<<4 | rc_ctrl->rc.s[1];
		TxMessage.Data[7] = 0;
	}

		CAN_Transmit(CAN2, &TxMessage);
}
union SendData
{
	float f;
	uint8_t c[4];
} send_f;
void CAN_MPU(void)
{
	CanTxMsg MPU_message;
	MPU_message.StdId = CAN_MPU_message_ID;
	MPU_message.IDE = CAN_ID_STD;
	MPU_message.RTR = CAN_RTR_DATA;
	MPU_message.DLC = 0x04;
	send_f.f = Angular_Handler.YAW;
	MPU_message.Data[0] = send_f.c[0];
	MPU_message.Data[1] = send_f.c[1];
	MPU_message.Data[2] = send_f.c[2];
	MPU_message.Data[3] = send_f.c[3];	
	CAN_Transmit(CAN2,&MPU_message);
}
/**************************************End of file************************************************/


