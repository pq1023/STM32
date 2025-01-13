//裁判系统通讯使用USART1
#include "usart1.h"
#include "detect_task.h"
#include "stdio.h"
QueueHandle_t TxCOM1;
QueueHandle_t RxCOM1;
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART1->SR & 0X40) == 0); //循环发送,直到发送完毕

    USART1->DR = (u8) ch;
    return ch;
}
#endif
/* TX */
#define    GPIO_TX                   GPIOA
#define    GPIO_PIN_TX               GPIO_Pin_9
#define    GPIO_PINSOURCE_TX         GPIO_PinSource9
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOA

/* RX */
#define    GPIO_RX                   GPIOA
#define    GPIO_PIN_RX               GPIO_Pin_10
#define    GPIO_PINSOURCE_RX         GPIO_PinSource10
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOA
/*
*/

void Usart1_Init( u32 bound )
{
    USART_InitTypeDef  xUsartInit;
    GPIO_InitTypeDef   xGpioInit;

    NVIC_InitTypeDef   xNvicInit;

    RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );

    GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_USART1 );
    GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_USART1 );

    xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
    xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
    xGpioInit.GPIO_OType = GPIO_OType_PP;
    xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init( GPIO_TX, &xGpioInit );

    xGpioInit.GPIO_Pin = GPIO_PIN_RX;
    GPIO_Init( GPIO_RX, &xGpioInit );

    xUsartInit.USART_BaudRate            = bound;
    xUsartInit.USART_WordLength          = USART_WordLength_8b;
    xUsartInit.USART_StopBits            = USART_StopBits_1;
    xUsartInit.USART_Parity              = USART_Parity_No;
    xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;


    USART_Init( USART1, &xUsartInit );
    USART_Cmd( USART1, ENABLE );

    USART_ITConfig( USART1, USART_IT_RXNE, ENABLE  );

    {
        /* 创建SEND消息队列 */
        TxCOM1 = xQueueCreate(45, sizeof(u8));

        if( TxCOM1 == 0 ) /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
        {
        }

        /* 创建REVICE消息队列 */
        RxCOM1 = xQueueCreate(45, sizeof(u8));

        if( RxCOM1 == 0 ) /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
        {
        }
    }

    USART_Cmd(USART1, ENABLE);

    xNvicInit.NVIC_IRQChannel                    = USART1_IRQn;
    xNvicInit.NVIC_IRQChannelPreemptionPriority  = JUDGE_NVIC;
    xNvicInit.NVIC_IRQChannelSubPriority         = 0;
    xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init( &xNvicInit );

}

//串口总线空闲中断：在检测到接收数据之后，数据总线上一个字节的时间内没有接收到数据则发生中断
//USART1 + DMA + IDLE方式，可以接收一帧一帧的数据
void USART1_IRQHandler(void)
{
    u8 Res;
    BaseType_t  TaskWoken;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        //clear the idle pending flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        (void)USART1->SR;
        (void)USART1->DR;

        Res = USART_ReceiveData(USART1);	//读取接收到的数据
        DetectHook(JudgementTOE);

        xQueueSendFromISR(RxCOM1, &Res, &TaskWoken);
    }
		   else if(USART_GetITStatus(USART1, USART_IT_ORE_RX) != RESET)    //读入USART_SR 
    {
			    USART_ClearFlag(USART1,USART_FLAG_ORE); //读SR 
          USART_ReceiveData(USART1);       //读入USART_DR 
    }
}






