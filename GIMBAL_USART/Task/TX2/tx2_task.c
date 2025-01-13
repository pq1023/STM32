#include "Tx2_task.h"
#include "detect_task.h"
#include "math.h"
#include "led.h"

#include "usart.h"
#include "string.h"

#include "IMUTask.h"
#include "MPU_TIME_Init.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "protocol.h"
#include "user_lib.h"

#include "gimbal_task.h"

PC_Ctrl_Union_t PcData;
PC_Data GimbalRaw;
send_Tx2_t Send2PC_Sta;
extern gimbal_control_state gimbal_control;
extern QueueHandle_t TxCOM6;
extern QueueHandle_t RxCOM6;

//通过指针方式获取原始数据
const PC_Data *get_PC_Data_Point(void)
{
    return &GimbalRaw;
}
send_Tx2_t *get_Send2PC_Point(void)
{
	return &Send2PC_Sta;
}
fp32 pc_pitch_limit = 3, pc_yaw_limit = 7;
fp32 pc_pitch_deadline = 1, pc_yaw_deadline = 1;
fp32 pitch_motor_speed, yaw_motor_speed;
fp32 pc_pitch_raw = 0, pc_yaw_raw = 0;
//上位机数据解码
void MinipcDatePrcess(PC_Ctrl_t *pData)
{
	if(pData==NULL) 
	{
		return ;
	}
	pitch_motor_speed = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;	//fabs(fp32_limit(fp32_deadline((fp32)gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm, -3, 3), 50, -50));
	yaw_motor_speed   = fabs(fp32_limit(fp32_deadline((fp32)gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->speed_rpm, -10, 10), 100, -100));

	pc_pitch_deadline = fp32_map(pitch_motor_speed, 0, 50, 0, 3);
	pc_yaw_deadline   = fp32_map(yaw_motor_speed, 0, 100, 0, 10);
	
	pc_pitch_raw                      = pData->angle_pitch;
	pc_yaw_raw                        = pData->angle_yaw;
	GimbalRaw.PcPitch= fp32_deadline(fp32_limit(pData->angle_pitch, -pc_pitch_limit, pc_pitch_limit), -pc_pitch_deadline, pc_pitch_deadline);
	GimbalRaw.PcYaw= fp32_deadline(fp32_limit(pData->angle_yaw, -pc_yaw_limit, pc_yaw_limit), -pc_yaw_deadline, pc_yaw_deadline);

//	if(fabs(pData->angle_pitch) > 3)
//		GimbalRaw.PcPitch                 = fp32_deadline(fp32_limit(pData->angle_pitch, pc_pitch_limit, -pc_pitch_limit), -pc_pitch_deadline, pc_pitch_deadline);
//	else
//		GimbalRaw.PcPitch									= pData->angle_pitch;
//	
//	if(fabs(pData->angle_yaw) > 5)
//		GimbalRaw.PcYaw                   = -fp32_deadline(fp32_limit(pData->angle_yaw, pc_yaw_limit, -pc_yaw_limit), -pc_yaw_deadline, pc_yaw_deadline);
//	else
//		GimbalRaw.PcYaw									= -pData->angle_yaw;
	
	GimbalRaw.shoot_flag              = pData->shoot_flag;
	GimbalRaw.frequency_adj           = pData->frequency_adj;

}

void minipc_Get_Message(void)
{
	DataRevice Buffer;
	xQueueReceive(TxCOM6,&Buffer,10);
	memcpy(PcData.PcDataArray,Buffer.buffer,MINIPC_FRAME_LENGTH);
	MinipcDatePrcess(&(PcData.PcDate));
}

void minipc_Get_Message_ISR(DataRevice *Buffer)
{
	memcpy(PcData.PcDataArray,Buffer->buffer,MINIPC_FRAME_LENGTH);
	MinipcDatePrcess(&(PcData.PcDate));
}

void minipc_Error_Out(void)
{
	PcData.PcDate.angle_pitch   = 0;
	PcData.PcDate.angle_yaw     = 0;
	PcData.PcDate.distance			= 0;
	PcData.PcDate.shoot_flag    = 0;
	
	GimbalRaw.PcPitch                 = 0;
	GimbalRaw.PcYaw                   = 0;
	GimbalRaw.shoot_flag              = 0;
	GimbalRaw.distance           = 0;
}

static void UART_PutChar(USART_TypeDef* USARTx,u8 ch)
{
	USART_SendData(USARTx, (u8)ch);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
}

void minipc_Send_Message(void)
{
	SendIMUData_t Gimbal_Yaw, Gimbal_Pitch;
	
	Send2PC_Sta.start = 0xAA;
	Send2PC_Sta.end   = 0xBB;
	
	Gimbal_Pitch.raw = Angular_Handler.ROLL;
	Gimbal_Yaw.raw = Angular_Handler.YAW;
//	Gimbal_Pitch.raw = 15.644124;
//	Gimbal_Yaw.raw = 14.264546;
	
	Send2PC_Sta.pitch[0] = Gimbal_Pitch.bit8[0];
	Send2PC_Sta.pitch[1] = Gimbal_Pitch.bit8[1];
	Send2PC_Sta.pitch[2] = Gimbal_Pitch.bit8[2];
	Send2PC_Sta.pitch[3] = Gimbal_Pitch.bit8[3];
	
	Send2PC_Sta.yaw[0] = Gimbal_Yaw.bit8[0];
	Send2PC_Sta.yaw[1] = Gimbal_Yaw.bit8[1];
	Send2PC_Sta.yaw[2] = Gimbal_Yaw.bit8[2];
	Send2PC_Sta.yaw[3] = Gimbal_Yaw.bit8[3];
	

	UART_PutChar(USART6, Send2PC_Sta.start);

	UART_PutChar(USART6, Send2PC_Sta.color);

	UART_PutChar(USART6, Send2PC_Sta.pitch[0]);
	UART_PutChar(USART6, Send2PC_Sta.pitch[1]);
	UART_PutChar(USART6, Send2PC_Sta.pitch[2]);
	UART_PutChar(USART6, Send2PC_Sta.pitch[3]);

	UART_PutChar(USART6, Send2PC_Sta.yaw[0]);
	UART_PutChar(USART6, Send2PC_Sta.yaw[1]);
	UART_PutChar(USART6, Send2PC_Sta.yaw[2]);
	UART_PutChar(USART6, Send2PC_Sta.yaw[3]);
	
//	UART_PutChar(USART1, Send2PC_Sta.heat);

	UART_PutChar(USART6, Send2PC_Sta.end);	
}


int minipc_error_timer = 0;
/*****************************************/
#define TX2_TASK_PRIO 21
#define TX2_TASK_SIZE 256
static TaskHandle_t TX2_TASK_Handler;
void Tx2_task(void  *pvParameters);
/********************************************/
//TX2任务
void task_Tx2_Create(void)
{
	xTaskCreate((TaskFunction_t )Tx2_task,
							(const char*    )"Tx2_task",
							(uint16_t       )TX2_TASK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )TX2_TASK_PRIO,
							(TaskHandle_t*  )&TX2_TASK_Handler);
}
int BA = 38;
void Tx2_task(void  *pvParameters)
{
    while(1)
    {
			LED1(1);
			if(++minipc_error_timer >= 10) {
				minipc_error_timer = 0;
				minipc_Error_Out();
			}
			minipc_Send_Message();
			vTaskDelay(52);			
    }
}
