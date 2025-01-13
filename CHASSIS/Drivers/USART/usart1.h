#ifndef _BSP_USART_INIT_H_
#define _BSP_USART_INIT_H_

#include "stm32f4xx.h"                  // Device header
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"
#include "main.h"

extern QueueHandle_t TxCOM1;
extern QueueHandle_t RxCOM1;

void Usart1_Init( u32 bound );



#endif

