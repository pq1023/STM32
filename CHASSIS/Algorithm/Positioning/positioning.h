#ifndef __POSITIONING_H
#define __POSITIONING_H
#include "CAN_Receive.h"
#include "math.h"
#include "IMUTask.h"
typedef struct
{
	float Real_Distance_Y;
	float Real_Distance_X;
}Distances_T;

extern Positioning Positioning_t;
extern Angular_Handle  Angular_Handler; 
extern void Distance_State(void);

#endif

