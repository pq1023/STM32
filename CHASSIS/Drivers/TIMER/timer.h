#ifndef TIMER_H
#define TIMER_H

#include "sys.h"
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u32 arr, u32 psc);
void TIM5_PWM_Init(u32 arr, u32 psc);
//static void beep_detect(int time);
#endif
