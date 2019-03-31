#ifndef TIMER__H
#define TIMER__H

#endif

#include "sys.h"
#include "delay.h"
#include "usart.h"


void TIM5_IC_Init(void);
void TIM9_PWM_Init(uint16_t arr_period,uint16_t psc_TIM9);
void TIM14_PWM_Init(uint16_t arr_period,uint16_t psc_TIM14);

