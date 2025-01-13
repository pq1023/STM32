#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "sys.h"

#define ON  0
#define OFF 1

#define LED1_GPIO_PIN	GPIO_Pin_10
#define LED1_GPIO_PORT	GPIOC
#define LED1_GPIO_CLK	RCC_AHB1Periph_GPIOC

#define LED2_GPIO_PIN	GPIO_Pin_11
#define LED2_GPIO_PORT	GPIOC
#define LED2_GPIO_CLK	RCC_AHB1Periph_GPIOC

#define LED3_GPIO_PIN	GPIO_Pin_12
#define LED3_GPIO_PORT	GPIOC
#define LED3_GPIO_CLK	RCC_AHB1Periph_GPIOC

#define LED4_GPIO_PIN	GPIO_Pin_2
#define LED4_GPIO_PORT	GPIOD
#define LED4_GPIO_CLK	RCC_AHB1Periph_GPIOD

#define LED1(a)	if (a)	\
        GPIO_ResetBits(LED1_GPIO_PORT,LED1_GPIO_PIN);\
    else		\
        GPIO_SetBits(LED1_GPIO_PORT,LED1_GPIO_PIN)

#define LED2(a)	if (a)	\
        GPIO_ResetBits(LED2_GPIO_PORT,LED2_GPIO_PIN);\
    else		\
        GPIO_SetBits(LED2_GPIO_PORT,LED2_GPIO_PIN)

#define LED3(a)	if (a)	\
        GPIO_ResetBits(LED3_GPIO_PORT,LED3_GPIO_PIN);\
    else		\
        GPIO_SetBits(LED3_GPIO_PORT,LED3_GPIO_PIN)


#define LED4(a)	if (a)	\
        GPIO_ResetBits(LED4_GPIO_PORT,LED4_GPIO_PIN);\
    else		\
        GPIO_SetBits(LED4_GPIO_PORT,LED4_GPIO_PIN)
void LED_Init(void);

#endif

