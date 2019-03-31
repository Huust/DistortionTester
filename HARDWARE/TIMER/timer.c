#include "timer.h"
#include "adc.h"

#define arr (0XFFFFFFFF)
#define psc (84 - 1)
uint8_t flag_1 = 0;
uint8_t flag_2 = 0;

uint32_t value_1,value_2,value_3,CNT_TIM;
float fre_ic;

void TIM5_IC_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM5_ICInitStructure;
	
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); 
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=arr;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	


	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
  TIM5_ICInitStructure.TIM_ICFilter = 0x00;
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
		
	TIM_ITConfig(TIM5,TIM_IT_CC1,ENABLE);
	
  TIM_Cmd(TIM5,ENABLE ); 	

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);

}

void TIM5_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM5,TIM_IT_CC1);
	value_3 = TIM5->CCR1;
	
	
	if(flag_1 == 0xff)
	{
		TIM_ITConfig(TIM5,TIM_IT_CC1,DISABLE);
		value_2 = value_3;
		CNT_TIM = value_2 - value_1;	
		TIM5->CNT = 0;
		fre_ic = 1.0/(0.000001*CNT_TIM);
		flag_2 = 1;
		//printf("%f.3 hz\n",fre_ic);
	MYADC_Init();
	START();
	}
	else
	{
		value_1 = value_3;	
		flag_2 = 0;
	}
	flag_1 = ~flag_1;
}

void TIM9_PWM_Init(uint16_t arr_period,uint16_t psc_TIM9)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler=psc_TIM9; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=arr_period;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC1Init(TIM9, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM9,ENABLE);
	TIM_Cmd(TIM9, ENABLE); 
	TIM_SetCompare1(TIM9,arr_period/2);
}

void TIM14_PWM_Init(uint16_t arr_period,uint16_t psc_TIM14)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_Init(GPIOF,&GPIO_InitStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler=psc_TIM14; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=arr_period;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC1Init(TIM14, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM14,ENABLE);
	TIM_Cmd(TIM14, ENABLE); 
	TIM_SetCompare1(TIM14,arr_period/2);
}

