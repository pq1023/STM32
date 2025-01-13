#include "infrared.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define INFRARED_TASK_PRIO 19
#define INFRARED_STK_SIZE 512
TaskHandle_t InfraredTask_Handler;

void Infrared_task(void);
/**
  * @brief	红外任务创建
  */
void task_Infrared_Create(void)
{
	xTaskCreate((TaskFunction_t)Infrared_task,
	            (const char *)"Infrared_task",
	            (uint16_t)INFRARED_STK_SIZE,
	            (void *)NULL,
	            (UBaseType_t)INFRARED_TASK_PRIO,
	            (TaskHandle_t *)&InfraredTask_Handler);
}

										//	0		1		2			3		4			5		6			7 
u8 USART3_TX_BUF[9]={0x62,0x33,0x09,0x00,0x01,0x00,0x00,0x00};
int a=0;
int time = 32;
void Infrared_task(void)
{
	u8 t;
	while(1)
	{
		a++;
		USART3_TX_BUF[8]=USART3_TX_BUF[0]^USART3_TX_BUF[1]^USART3_TX_BUF[2]^USART3_TX_BUF[3]^USART3_TX_BUF[4]^USART3_TX_BUF[5]^USART3_TX_BUF[6]^USART3_TX_BUF[7];
		for(t=0;t<9;t++)
		{
			USART_SendData(USART3, USART3_TX_BUF[t]);      
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//等待发送结束
		}
		USART3_TX_BUF[4]++;
		if(USART3_TX_BUF[4]==4)
		{
			USART3_TX_BUF[4]=01;
		}
		vTaskDelay(time);
	}	
}

sensor_Distance_t sensor_Distance;
//接收原始数据，为10个字节，给了20个字节长度，防止DMA传输越界
static uint8_t rx_buf[2][RX_BUF_NUM];
void infrared_Usart_init(void)
{
	Usart3_Init(rx_buf[0], rx_buf[1], RX_BUF_NUM);
}

/*******************************************
返回数据变量，通过指针传递方式传递信息
********************************************/
const sensor_Distance_t *get_sensor_Distance_point(void)
{
    return &sensor_Distance;
}
u8 rx_sum=0;
//u16 sensor_Distance1[3];
u8 Q=0;
//串口中断
void USART3_IRQHandler(void)
{
	u8 buffer[20];
		static uint16_t this_time_rx_len = 0;
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//空闲中断
    {
        (void)USART3->SR;
        (void)USART3->DR;
        if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream1);
            DMA1_Stream1->NDTR = (uint16_t)RX_BUF_NUM;
            DMA1_Stream1->CR |= (uint32_t)DMA_SxCR_CT;
            DMA_Cmd(DMA1_Stream1, ENABLE);

            if(this_time_rx_len == FRAME_LENGTH)
            {
							memcpy(buffer, rx_buf[0],RX_BUF_NUM );
							if(buffer[0]==0x62)
							{
									rx_sum=buffer[0]^buffer[1]^buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7];
								if(buffer[8]==rx_sum)//                //处理数据

                {
									Q=buffer[4]-1;
									sensor_Distance.sensor[Q]=(buffer[5] << 8) + buffer[6];
								}
							}

            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream1);
            DMA1_Stream1->NDTR = (uint16_t)RX_BUF_NUM;
            DMA1_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);
            DMA_Cmd(DMA1_Stream1, ENABLE);

            if(this_time_rx_len == FRAME_LENGTH)
            {
							
							 memcpy(buffer, rx_buf[1],RX_BUF_NUM );
							if(buffer[0]==0x62)
							{
									rx_sum=buffer[0]^buffer[1]^buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7];
								if(buffer[8]==rx_sum)                //处理数据
                {
									Q=buffer[4]-1;
									sensor_Distance.sensor[Q]=(buffer[5] << 8) + buffer[6];
								}
							}
						 else
							{
									return;
							}
            }
        }
				USART_ClearITPendingBit(USART3, USART_IT_IDLE);
    }
		 else if (USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)//检查指定的USART中断是否发生  接受中断
    {
			  USART_ClearFlag(USART3,USART_FLAG_ORE);
        USART_ReceiveData(USART3);//			
    }
}

