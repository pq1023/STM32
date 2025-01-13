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
fp32 pc_pitch_limit = 5, pc_yaw_limit = 5;
fp32 pc_pitch_deadline = 1, pc_yaw_deadline = 1;
fp32 pitch_motor_speed, yaw_motor_speed;
fp32 pc_pitch_raw = 0, pc_yaw_raw = 0;
//上位机数据解码
void MinipcDatePrcess(PC_Ctrl_t *pData)
{
	pc_pitch_raw                      = pData->angle_pitch;
	pc_yaw_raw                        = pData->angle_yaw;
	GimbalRaw.PcPitch                 = fp32_deadline(fp32_limit(pData->angle_pitch, pc_pitch_limit, -pc_pitch_limit), -pc_pitch_deadline, pc_pitch_deadline);
	GimbalRaw.PcYaw                   = fp32_deadline(fp32_limit(pData->angle_yaw, pc_yaw_limit, -pc_yaw_limit), -pc_yaw_deadline, pc_yaw_deadline);
	if(fabs(pData->angle_pitch) > 3)
		
		GimbalRaw.pitch_angle_dynamic_refs = -fp32_deadline(fp32_limit(pData->angle_pitch, pc_pitch_limit, -pc_pitch_limit), -pc_pitch_deadline, pc_pitch_deadline) + Angular_Handler.Pitch;
	else
		GimbalRaw.pitch_angle_dynamic_refs = -pData->angle_pitch + Angular_Handler.Pitch;
	if(fabs(pData->angle_yaw) > 4)
		GimbalRaw.yaw_angle_dynamic_refs   = fp32_deadline(fp32_limit(pData->angle_yaw, pc_yaw_limit, -pc_yaw_limit), -pc_yaw_deadline, pc_yaw_deadline) + Angular_Handler.YAW; //右负左正
	else
		GimbalRaw.yaw_angle_dynamic_refs   = pData->angle_yaw + Angular_Handler.YAW;
	GimbalRaw.shoot_flag              = pData->shoot_flag;
	GimbalRaw.frequency_adj						= pData->frequency_adj;
//	GimbalRaw.distance           = pData->distance;

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
	PcData.PcDate.frequency_adj = 0;
	PcData.PcDate.shoot_flag    = 0;
	
	GimbalRaw.PcPitch                 = 0;
	GimbalRaw.PcYaw                   = 0;
	GimbalRaw.shoot_flag              = 0;
	GimbalRaw.frequency_adj           = 0;
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
			vTaskDelay(20);			

    }
}

