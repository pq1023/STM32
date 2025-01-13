#ifndef _USART3_H_
#define _USART3_H_

#include "stm32f4xx.h"

#define  RCC_AHB1Periph_GPIOx     RCC_AHB1Periph_GPIOB     //gpio时钟 引脚
#define  GPIO_Pin_Rx              GPIO_Pin_11
#define  GPIO_Pin_Tx              GPIO_Pin_10
#define  GPIO_PinSourceRx         GPIO_PinSource11
#define  GPIO_PinSourceTx         GPIO_PinSource10

#define  RCC_APB1Periph_USARTx    RCC_APB1Periph_USART3    //串口
#define  GPIO_AF_USARTx           GPIO_AF_USART3
#define  usartx										USART3
#define  USARTx_IRQn							USART3_IRQn

#define  usart_bound              9600                     //波特率
#define  usart_stopbits						USART_StopBits_1				 //停止位
#define	 usart_parity							USART_Parity_No					 //奇偶校验
#define	 usart_wordlength         USART_WordLength_8b			 //数据位

#define  RCC_AHB1Periph_DMAx      RCC_AHB1Periph_DMA1     //DMA时钟
#define  DMAx_Streamx_rx          DMA1_Stream1             //dma数据流
#define  DMAx_Streamx_tx          DMA1_Stream3 
#define  DMA_Channel_x            DMA_Channel_4            //dma通道

extern void Usart3_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
