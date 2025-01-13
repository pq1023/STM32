#ifndef _INFRARED_H_
#define _INFRARED_H_
#include "stm32f4xx.h"
#include "usart3.h"
#include <string.h>


#define RX_BUF_NUM 20u

#define FRAME_LENGTH 9u

typedef __packed struct
{
	u16 sensor[3];//Êý¾Ý
}sensor_Distance_t;

extern sensor_Distance_t sensor_Distance;

extern void infrared_Usart_init(void);
extern const sensor_Distance_t *get_sensor_Distance_point(void);
extern void task_Infrared_Create(void);
extern void Infrared_task(void);


#endif
