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
#include "detect_task.h"
#include "gimbal_task.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1
//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                   \
    {                                                                                        \
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
#define get_gimbal_motor_measuer(ptr, rx_message)                                            \
    {                                                                                        \
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
motor_measure_t motor_yaw, motor_chassis[4];
RC_ctrl_t Can_Rc_ctrl;
Positioning Positioning_t;
		
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
float YAW_receive_current;
float MPU_R_YAW;
static void CAN2_hook(CanRxMsg *rx_message)
{
//	if(Positioning_t.RC_mes == 1)
//		CAN_Send_Positioning();
//	else;
    switch (rx_message->StdId)
    {
			case CAN_YAW_MOTOR_ID:
			{
				//处理电机数据宏函数
				get_gimbal_motor_measuer(&motor_yaw, rx_message);
				//记录时间
				DetectHook(YawGimbalMotorTOE);
				break;
			}			
			case CAN_RC_receive_ID:
			{
				if(rx_message->Data[7])
				{
					CAN_Send_Positioning();
					delay_ms(100);
					__set_FAULTMASK(1);//关闭所有中断
					NVIC_SystemReset();//软件复位
				}
				Can_Rc_ctrl.rc.ch[0] = rx_message->Data[0] << 8 | rx_message->Data[1];
				Can_Rc_ctrl.rc.ch[1] = rx_message->Data[2] << 8 | rx_message->Data[3];
				Can_Rc_ctrl.rc.ch[2] = 0;//云台占用
				Can_Rc_ctrl.rc.ch[3] = 0;//云台占用
				Can_Rc_ctrl.rc.ch[4] = rx_message->Data[4] << 8 | rx_message->Data[5];
				Can_Rc_ctrl.rc.s[0] = rx_message->Data[6] >> 4;
				Can_Rc_ctrl.rc.s[1] = rx_message->Data[6] & 0x0F;
				break;				
			}	
			case CAN_MPU_receive_ID:
			{
				union ReceiveData
				{
					uint8_t c[4];
					float f;
				}mpu_f;			
				mpu_f.c[0] = rx_message->Data[0];
				mpu_f.c[1] = rx_message->Data[1];
				mpu_f.c[2] = rx_message->Data[2];
				mpu_f.c[3] = rx_message->Data[3];
				MPU_R_YAW = mpu_f.f;
			}
			default:
				break;
    }
}
static void CAN1_hook(CanRxMsg *rx_message)
{
    switch(rx_message->StdId)
    {
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
				//处理电机ID号
				i = rx_message->StdId - CAN_3508_M1_ID;
				//处理电机数据宏函数
				get_motor_measure(&motor_chassis[i], rx_message);
				//记录时间
				DetectHook(ChassisMotor1TOE + i);
				break;
			}
			case CAN_Positioning:
			{
				Positioning_t.Distance_Right =((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);   
				Positioning_t.Distance_Left = -((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
				union SendData
				{
					uint8_t c[4];
					float f;
				}data_f;
				data_f.c[0] = rx_message->Data[4] ;
				data_f.c[1] = rx_message->Data[5] ;
				data_f.c[2] = rx_message->Data[6] ;
				data_f.c[3] = rx_message->Data[7] ;
				Positioning_t.Angle = data_f.f;					
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
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
//返回遥控器控制变量，通过指针传递方式传递信息
const RC_ctrl_t *get_can_remote_control_point(void)
{
    return &Can_Rc_ctrl;
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

//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//发送底盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}


void CAN_JUDGE_1(void)
{
	CanTxMsg JUDGE1_TxMessage;
	JUDGE1_TxMessage.StdId = CAN_JUDGE1_ID;
	JUDGE1_TxMessage.IDE = CAN_ID_STD;
	JUDGE1_TxMessage.RTR = CAN_RTR_DATA;
	JUDGE1_TxMessage.DLC = 0x08;
	JUDGE1_TxMessage.Data[0] = gumbal_judge.Color;
	JUDGE1_TxMessage.Data[1] = gumbal_judge.GameSta;
	JUDGE1_TxMessage.Data[2] = gumbal_judge.Heat_1 >> 8;
	JUDGE1_TxMessage.Data[3] = gumbal_judge.Heat_1;
	JUDGE1_TxMessage.Data[4] = gumbal_judge.Heat_2 >> 8;
	JUDGE1_TxMessage.Data[5] = gumbal_judge.Heat_2;
	JUDGE1_TxMessage.Data[6] = gumbal_judge.Start_Dodge;
	JUDGE1_TxMessage.Data[7] = gumbal_judge.patrol_flag;

	CAN_Transmit(CAN2, &JUDGE1_TxMessage);
}


void CAN_Send_Positioning(void)
{
	CanTxMsg POSitioning;
	POSitioning.StdId = 0x401;
	POSitioning.IDE = CAN_ID_STD;
	POSitioning.RTR = CAN_RTR_DATA;
	POSitioning.DLC = 0x01;
	POSitioning.Data[0] = 1;
	CAN_Transmit(CAN1, &POSitioning);
}
/**************************************End of file************************************************/



