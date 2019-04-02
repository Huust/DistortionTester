
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "arm_math.h"
#include "adc.h"
#include "timer.h"

#define poolsize (1024)    /* 每次DMA传输的ADC采样点数 */
#define fft_size (1024)    /* fft点数 */
float fre_sampling = 42682;    /* ADC的采样频率： 1/(ADCCLK * (Tsampling+12))   其中（Tsampling+12）指的是采两个点之间的周期  */
extern uint16_t datapool_adc[poolsize];
extern uint8_t flag_2;

//失真度相关的变量
float value1;
float value2;
float ave = 0;
float *p1 = &value1;
float *p2 = &value2;
int index_position;  /*某一数组下标*/
int cnt = 0; /*统计在1024个点内，出现奇次谐波的次数*/




q15_t datapool_adc1[fft_size];


q15_t adc1_output[fft_size*2];
q15_t adc1_outcome_q15[fft_size];
float32_t adc1_outcome_f32[fft_size];
float32_t adc1_voltage_f32[fft_size]; /* ADC采样值-->实际电压值 */
float32_t adc_output_f32[fft_size*2];
float32_t adc1_voltage_f32_over[1500]; //分子数组
float32_t adc1_voltage_f32_below[1500];  //分母数组

uint8_t signal = 0;
int index_arr;
int index_arr_static;
float MAX;

/* 串口发送函数 */
void HMISendb(u8 k);
void HMISends(char *buf1);
u8 idfind(u8 *buf,u8 *val,u8 len);

int main()
{
	//fre_ic/(42682/1024)	   
	delay_init(168);
	uart_init(9600);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	MYDMA_ADC_Init();
	TIM5_IC_Init();
	MYGPIO_Init();
	
	// PF9->TIM14   PF9->TIM9
	// 自出方波检测失真度
	TIM9_PWM_Init(100 - 1,168 - 1);
	TIM14_PWM_Init(100 - 1,84 - 1);


	
	
	uint16_t i = 0;//循环控制变量
	extern float fre_ic;
	
	arm_rfft_instance_q15 S;
	DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
	HMISends("page 0");
				HMISendb(0xff);	
	while(1)
	{
		
		/*输入捕获获得频率，因此轮询+标志位，每得到一次频率有效值，并且DMA传输结束就进行处理*/
		while((flag_2 != 1)&&(DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0)) != SET);
				flag_2 = 0;
				DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
				
		for(i=0;i<poolsize;i++)
				{
					datapool_adc1[i] = datapool_adc[i];
				}
				
				
		
				/* FFT */
				arm_rfft_init_q15(&S,fft_size,0,1);
				arm_rfft_q15(&S,datapool_adc1,adc1_output);
		for(i=0;i<(poolsize*2);i++)
				{
					adc_output_f32[i] = (float32_t)adc1_output[i];  //强制转换成f32，提高精度
				}
				arm_cmplx_mag_f32(adc_output_f32,adc1_outcome_f32,fft_size);		
				
				
				
				
				/* 获得adc经过fft后的真实电压值 */
				for(i = 0;i<fft_size;i++)
				{
					adc1_voltage_f32[i] = adc1_outcome_f32[i]/4096*3.3;
				}
				
				
				
				/*找到一次谐波所在元素下标index的大致位置*/
				if((fre_ic/(fre_sampling/fft_size)) - ((int)(fre_ic/(fre_sampling/fft_size))) >= 0.5)
				{
					index_arr = (int)(fre_ic/(fre_sampling/fft_size))+1;
				}
				else
				{
					index_arr = (int)(fre_ic/(fre_sampling/fft_size));
				}
				
				index_arr_static = index_arr;
				
				/* 位置寻找优化 */
				/* 考虑到可能会在10以内出现一次谐波，所以将fre_ic<=400单独考虑 */
				/* 优化方向：没有考虑到频谱泄露的问题 */
				if(fre_ic<=400)
				{
					for(i = 0;(2*i+1)*index_arr_static<fft_size;i++)
					{
						adc1_voltage_f32_below[i] = adc1_voltage_f32[(2*i+1)*index_arr_static];
						adc1_voltage_f32_over[i] = adc1_voltage_f32[(2*i+3)*index_arr_static];
						
						cnt++;
					}
					adc1_voltage_f32_over[cnt-1] = 0;
				}
				else
				{
					for(i = 0;i<(fft_size-10);i++)
					{
						if(i == index_arr-10)
						{
							MAX = adc1_voltage_f32[index_arr-10];
						}
						if(MAX<=adc1_voltage_f32[i])
						{
							MAX = adc1_voltage_f32[i];
						}				
						if(i == index_arr+10)
						{
							adc1_voltage_f32_below[cnt] = MAX;
							if(cnt>=1)
							adc1_voltage_f32_over[cnt-1] = MAX;
							cnt++;
							index_arr = (2*cnt+1)*index_arr_static;
							MAX = 0xfffff;   //将max取大，避免下一次循环误入判断
						}
						
						
					}
				}
				i = 0;
				
				

				
				
				
				//RMS计算   失真度的定义：分子为去除噪声，直流分量和一次谐波后剩下频率分量幅值均方根
				//			分母为去除噪声，直流分量后剩下频率分量幅值均方根
				arm_rms_f32(adc1_voltage_f32_over,cnt-1,p1);//分子
				arm_rms_f32(adc1_voltage_f32_below,cnt,p2);//分母
				ave = value1/value2;
				
				
				/* 显示到串口上 */
				printf("t0.txt=\"%.3f%%\" ",ave*100);
				HMISendb(0xff);
				printf("t3.txt=\"%.3fhz\" ",fre_ic);
				HMISendb(0xff);
				cnt = 0;
				
				TIM_ITConfig(TIM5,TIM_IT_CC1,ENABLE);
	}
	

	
	}	

	
	
	
	void HMISendb(u8 k)		         //0xff发送函数
{		 
	u8 i;
	 for(i=0;i<3;i++)
	 {
	 if(k!=0)
	 	{
			USART_SendData(USART1,k);  //发送一个字节
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){};//等待发送结束
		}
	 else 
	 return ;

	 } 
} 

	void HMISends(char *buf1)		  //字符串发送函数
{
	u8 i=0;
	while(1)
	{
	 if(buf1[i]!=0)
	 	{
			USART_SendData(USART1,buf1[i]);  //发送一个字节
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){};//等待发送结束
		 	i++;
		}
	 else 
	 return ;

	}
}


	
	
	

	


	
	
	
		
		




