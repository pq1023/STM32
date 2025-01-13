#include "shoot_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define SHOOT_TASK_PRIO 22
#define SHOOT_STK_SIZE 512
TaskHandle_t ShootTask_Handler;
void Shoot_task(void);

void task_Shoot_Create(void)
{
	xTaskCreate((TaskFunction_t)Shoot_task,
                (const char *)"Shoot_task",
                (uint16_t)SHOOT_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SHOOT_TASK_PRIO,
                (TaskHandle_t *)&ShootTask_Handler);
}

static void Shoot_Init(void)
{
	
}

void Shoot_task(void)
{
	Shoot_Init();
	while(1)
	{

		vTaskDelay(1);
	}
}

