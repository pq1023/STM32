#include "sys.h"
#include "delay.h"

#include "LED.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "timer.h"
#include "can.h"
#include "CAN_Receive.h"
#include "RemoteControl.h"

#include "IMUTask.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "infrared.h"
#include "usart1.h"

#include "Detect_Task.h"
#include "Judge_Task.h"


#define START_TASK_PRIO		1				//任务优先级
#define START_STK_SIZE 		128  			//任务堆栈大小
TaskHandle_t StartTask_Handler;				//任务句柄

void start_task(void *pvParameters);		//任务函数
void TaskStart(void);
void Init(void);

void TaskStart(void)
{
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}

int main(void)
{	
	Init();
	TaskStart();
}

void Init(void)
{
	delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	remote_control_init();
	Usart1_Init(115200);
	TIM3_Int_Init(4999,7199);//10Khz的计数频率 Tout = ((arr+1)*(psc+1))/Tclk
	infrared_Usart_init();//串口3配置
	LED_Init();
	IMU_Init();
	
	LED1(1);
	LED2(0);
	
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);//1Mbps
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
	
	delay_ms(1000);
}

//开始任务任务函数
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区

	IMU_task_Create();//陀螺仪任务创建
	task_Gimbal_Create();//云台任务创建
	task_Chsssis_Create();//底盘任务创建
	task_Detect_Create();//监测任务创建
	task_Judge_Create();//裁判系统任务创建
//	task_Infrared_Create();//激光测距任务创建
	
	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

