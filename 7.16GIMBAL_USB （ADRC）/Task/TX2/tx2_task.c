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
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"

PC_Data GimbalRaw;
extern gimbal_control_state gimbal_control;

extern const unsigned char CRC8_TAB_UI[256];
void PcDataClean(unsigned  char * pData, int num);
uint8_t PcDataCheck( uint8_t *pData );
//static void UART_PutChar(USART_TypeDef* USARTx, u8 ch);
static void minipc_Send_Message(USART_TypeDef* USARTx, send_Tx2_t *SEND_PC);
static PC_Ctrl_Union_t PcData;
extern QueueHandle_t TxCOM6;
extern QueueHandle_t RxCOM6;
extern u8 GFlag_state;
int QFlag_state, last_QFlag_state, last_GFlag_state;
send_Tx2_t TX_vision_Mes;
Inter_Data_t InterUpdate;
uint32_t TxTaskStack;
uint32_t sendTaskStack;
int tim=0;
extern vu8 bDeviceState;		//USB???? ???

//通过指针方式获取原始数据
const PC_Data *get_PC_Data_Point(void)
{
    return &GimbalRaw;
}
fp32 pc_pitch_limit = 3, pc_yaw_limit = 6;
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
	pitch_motor_speed = fabs(fp32_limit(fp32_deadline((fp32)gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm, -3, 3), 50, -50));//gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;	//
	yaw_motor_speed   = fabs(fp32_limit(fp32_deadline((fp32)gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->speed_rpm, -10, 10), 100, -100));

	pc_pitch_deadline = fp32_map(pitch_motor_speed, 0, 50, 0, 3);
	pc_yaw_deadline   = fp32_map(yaw_motor_speed, 0, 100, 0, 10);
	
	pc_pitch_raw                      = pData->angle_pitch;
	pc_yaw_raw                        = pData->angle_yaw;
//	GimbalRaw.PcPitch= fp32_deadline(fp32_limit(pData->angle_pitch, -pc_pitch_limit, pc_pitch_limit), -pc_pitch_deadline, pc_pitch_deadline);
//	GimbalRaw.PcYaw= fp32_deadline(fp32_limit(pData->angle_yaw, -pc_yaw_limit, pc_yaw_limit), -pc_yaw_deadline, pc_yaw_deadline);

//	if(fabs(pData->angle_pitch) > 4)
//		GimbalRaw.PcPitch                 = fp32_deadline(fp32_limit(pData->angle_pitch, -pc_pitch_limit, pc_pitch_limit), -pc_pitch_deadline, pc_pitch_deadline);
//	else
		GimbalRaw.PcPitch									= pData->angle_pitch;
	
//	if(fabs(pData->angle_yaw) > 2)
//		GimbalRaw.PcYaw                   = fp32_deadline(fp32_limit(pData->angle_yaw, -pc_yaw_limit, pc_yaw_limit), -pc_yaw_deadline, pc_yaw_deadline);
//	else
		GimbalRaw.PcYaw									= pData->angle_yaw;

	GimbalRaw.shoot_flag              = pData->shoot_flag;
	GimbalRaw.distance								= pData->distance;
	GimbalRaw.frequency_adj           = pData->frequency_adj;

}

float last_relative_angle=0,err_angle=0;
int count=0;
#define Interactive_TASK_PRIO 21
#define Interactive_TASK_SIZE 256
static TaskHandle_t Interactive_TASK_Handler;
void Interactive_task(void  *pvParameters);
//PC发送任务
void task_PC_Create(void)
{
	xTaskCreate((TaskFunction_t )Interactive_task,
							(const char*    )"Interactive_task",
							(uint16_t       )Interactive_TASK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )Interactive_TASK_PRIO,
							(TaskHandle_t*  )&Interactive_TASK_Handler);
}
void Interactive_task(void  *pvParameters)
{
	TX_vision_Mes.yaw.y=0;
	TX_vision_Mes.pitch.p = 0;
	while(1)
	{
//		TX_vision_Mes.yaw.y 	= -15.641284f; 
//		TX_vision_Mes.pitch.p 	= -20.4549713f;
		TX_vision_Mes.yaw.y 	= Angular_Handler.YAW;
		TX_vision_Mes.pitch.p 	= Angular_Handler.ROLL;
		TX_vision_Mes.color 	= judge_receive.Color;
//		TX_vision_Mes.selfid 	= GameRobotStat.robot_id;
		TX_vision_Mes.mode 		= 0;
		TX_vision_Mes.speed 	= 0;
		minipc_Send_Message(USART6, &TX_vision_Mes);
		LED2(ON);
		vTaskDelay(1);
	}
}

static void minipc_Send_Message(USART_TypeDef* USARTx, send_Tx2_t *TXmessage)
{
	unsigned char crc = 0;
	unsigned char *TX_data;
	TX_data = (unsigned char*)TXmessage;
	crc = get_crc8_check_sum(TX_data, 11, 0xff);
	TXmessage->CRC8 = crc; 
	VCP_DataTx(0xAA);
	
	VCP_DataTx(TXmessage->pitch.pitch_t[0]);
	VCP_DataTx(TXmessage->pitch.pitch_t[1]);
	VCP_DataTx(TXmessage->pitch.pitch_t[2]);
	VCP_DataTx(TXmessage->pitch.pitch_t[3]);
	
	VCP_DataTx(TXmessage->yaw.yaw_t[0]);
	VCP_DataTx(TXmessage->yaw.yaw_t[1]);
	VCP_DataTx(TXmessage->yaw.yaw_t[2]);
	VCP_DataTx(TXmessage->yaw.yaw_t[3]);
	
	VCP_DataTx(TXmessage->mode);
	VCP_DataTx(TXmessage->color);
	VCP_DataTx(TXmessage->speed);
	
	VCP_DataTx(0xBB);	
//	VCP_DataTx(TXmessage->CRC8);
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
//PC数据校验函数
uint8_t PcDataCheck( uint8_t *pData )
{
	if(pData[0]==0x5A && pData[15]==0xA5 )
	{
		return 1;
	}
	else 
	{
		return 0;
	}
}
void Tx2_task(void  *pvParameters)
{
//	u8 len;	
	u8 usbstatus=0;	
	while(1)
	{
	if(usbstatus!=bDeviceState)
	{
		usbstatus=bDeviceState;
		if(usbstatus==1)
		{
			LED4(ON);
		}
		else
		{
			LED4(OFF);
		}
	} 
	if(USB_USART_RX_STA&0x8000)
	{				
//		if(verify_crc8_check_sum(&USB_USART_RX_BUF[1], 13) != NULL)
		if(PcDataCheck(USB_USART_RX_BUF) == 1)
		{
			memcpy(PcData.PcDataArray,USB_USART_RX_BUF, MINIPC_FRAME_LENGTH);
			InterUpdate.now_pc_pitch_ref=PcData.PcDate .angle_pitch ;
			InterUpdate.now_pc_yaw_ref =PcData.PcDate .angle_yaw ;
			if(InterUpdate.last_pc_yaw_ref !=InterUpdate.now_pc_yaw_ref ||InterUpdate.last_pc_pitch_ref !=InterUpdate.now_pc_pitch_ref)
			{
				InterUpdate.last_pc_yaw_ref=InterUpdate.now_pc_yaw_ref;
				InterUpdate.last_pc_pitch_ref=InterUpdate.now_pc_pitch_ref;
			}
			MinipcDatePrcess(&(PcData.PcDate));
		}
//			len=USB_USART_RX_STA&0x3FFF;
		USB_USART_RX_STA=0;
	}

	TxTaskStack = uxTaskGetStackHighWaterMark(NULL);
	vTaskDelay(1);
	}
}
