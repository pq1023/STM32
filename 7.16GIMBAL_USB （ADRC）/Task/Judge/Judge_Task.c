#include "judge_Task.h"
#include "detect_task.h"
#define JUDGE_BUFFER_LEN 		100
#define JUDGE_INTER_TIME 		2

#define	JUDGE_TASK_PRIO 11
#define JUDGE_TASK_SIZE 256
static TaskHandle_t JUDGE_TASK_Handler;

u8 Judge_Buffer[JUDGE_BUFFER_LEN];		

Judge_FLAG Judge_Flag;
uint32_t judgeisTaskStack;
//读取裁判系统
void task_Judge_Create(void)
{
	xTaskCreate((TaskFunction_t)Judge_task, 
				(const char*    )"Judge_task", 
				(uint16_t       )JUDGE_TASK_SIZE, 
				(void*          )NULL, 
				(UBaseType_t    )JUDGE_TASK_PRIO,  
				(TaskHandle_t*  )&JUDGE_TASK_Handler);
}

void Judge_task(void *pvParameters)
{
	u8 	RES;
	while(1)
	{
		
		xQueueReceive(RxCOM1,&RES,portMAX_DELAY);		
		if(Judge_Flag.Flag==0 && RES==FRAME_HEADER)	//帧头的第一个字节
		{
			Judge_Flag.Flag=1;
			Judge_Buffer[0]=RES;
		}
		else if(Judge_Flag.Flag==1)				//长度
		{
			Judge_Flag.Flag=2;
			Judge_Buffer[1]=RES;
		}
		else if(Judge_Flag.Flag==2)				//长度
		{
			Judge_Flag.Flag=3;
			Judge_Buffer[2]=RES;
			Judge_Flag.data_len=(u16)(RES<<8)|Judge_Buffer[1]+ LEN_CMDID + LEN_TAIL;
			if(Judge_Flag.data_len>100)
			{
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==3)				//包序号
		{
			Judge_Flag.Flag=4;
			Judge_Buffer[3]=RES;
		}
		else if(Judge_Flag.Flag==4)				//CRC8
		{
			Judge_Flag.Flag=5;
			Judge_Buffer[4]=RES;
			if (verify_crc8_check_sum( Judge_Buffer, HEADER_LEN ) != NULL)	//CRC8校验
			{
				Judge_Flag.data_cnt=0;
			}
			else				//校验没有通过，重新开始
			{
				Judge_Flag.data_cnt=0;
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==5 && Judge_Flag.data_len>0)	//开始接受数据
		{
			Judge_Flag.data_len--;
//			Judge_Flag.data_cnt=Judge_Flag.data_cnt+1;
			Judge_Buffer[LEN_HEADER+Judge_Flag.data_cnt++]=RES;
			if(Judge_Flag.data_len==0)
			{
				Judge_Flag.Flag=6;
			}	
		}
		else 
		{
			Judge_Flag.Flag=0;
			Judge_Flag.data_len=0;
			Judge_Flag.data_cnt=0;
		}
		if(Judge_Flag.Flag==6)
		{
			Judge_DataVerify(Judge_Buffer);		//帧头部分检测完毕，读取裁判系统数据
			Judge_Flag.Flag=0;								//其他数据清零
			Judge_Flag.data_len=0;
			Judge_Flag.data_cnt=0;
		}judgeisTaskStack = uxTaskGetStackHighWaterMark(NULL);
	}
 
}





//裁判系统数据解析
void Judge_DataVerify(u8 *Buff)
{
	Dateframe_t	*frame;
  if(Buff!=NULL)
  {
	frame=(Dateframe_t *)Buff;
	//将进行帧头与整帧数据CRC校验  帧头CRC8  整帧CRC16
	if(verify_crc16_check_sum((uint8_t *)frame, HEADER_LEN + CMD_LEN + frame->FrameHeader.DataLength + CRC_LEN))
	{
		judgement_data_handler(Buff);  //通过校验进行数据解析
	}
 }
}




