 /**
   ****************************(C) COPYRIGHT 2020 NCIST****************************
   * @file       timer.c
   * @brief      
   *             		TIM5 PWM部分初始化
   *             		PWM输出初始化
   *             
   * @note       
   * @history
   *  Version    Date            Author          Modification
   *  V1.0.0     2021-07-17     	   RM              1. 完成
   *
   @verbatim
   ==============================================================================

   ==============================================================================
   @endverbatim
   ****************************(C) COPYRIGHT 2020 NCIST****************************
   */
#include "timer.h"
#include "led.h"

fp32 TIME_COUNT0=0;
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x05; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x05; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,ENABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


void TIM5_PWM_Init(u32 arr, u32 psc)//arr：自动重装值；psc：时钟预分频数
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  	//TIM5时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); //GPIOA0复用为定时器5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           //GPIOA0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //初始化PA0

    TIM_TimeBaseStructure.TIM_Prescaler = psc; //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_Period = arr; //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //初始化定时器5

    //初始化TIM2 Channel PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式：TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较使能输出
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性：TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse = 0;//比较初始值
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC1

    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR1是的预装载寄存器

    TIM_ARRPreloadConfig(TIM5, ENABLE); //ARPE使能

    TIM_Cmd(TIM5, ENABLE);  //使能TIM5
}


//定时器3中断服务程序
int count_vx;
int count_vy=0;
float chassis_vy, chassis_vz, chassis_vx;
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIM3更新中断标志 
			count_vy++;
//			if(count_vy > 0 && count_vy <= 12)
//			{
//				chassis_vx = 700;//600
//				chassis_vy = 0;
//			}
//			else if(count_vy > 12 && count_vy <= 24)
//			{
//				chassis_vx = -700;
//				chassis_vy = 0;
			
			
//			}
//			else
//				count_vy = 0;
			
			if(count_vy > 0 && count_vy <= 4)
			{
				chassis_vx = 0;//600
				chassis_vy = 400;
			}
//			else if(count_vy > 12 && count_vy <= 24)
//			{
//				chassis_vx = 0;
//				chassis_vy = 0;
//			}
			else
				count_vy = 0;
		}
}


void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
}

