
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "arm_math.h"
#include "adc.h"
#include "timer.h"

#define poolsize (1024)
#define fft_size (1024)
float fre_sampling = 42682;
extern uint16_t datapool_adc[poolsize];
extern uint8_t flag_2;

//ʧ������
float value1;
float value2;
float ave = 0;
float *p1 = &value1;
float *p2 = &value2;
int index_position;
int cnt = 0;//���ڽ��ߴ�г����������rms�ļ���




q15_t datapool_adc1[fft_size];


q15_t adc1_output[fft_size*2];
q15_t adc1_outcome_q15[fft_size];
float32_t adc1_outcome_f32[fft_size];
float32_t adc1_voltage_f32[fft_size];
float32_t adc_output_f32[fft_size*2];
float32_t adc1_voltage_f32_over[1500]; //��������
float32_t adc1_voltage_f32_below[1500];  //��ĸ����

uint8_t signal = 0;
int index_arr;
int index_arr_static;
float MAX;

void HMISendb(u8 k);
void HMISends(char *buf1);
u8 idfind(u8 *buf,u8 *val,u8 len);

int main()
{
//�����㣬����ֱ���Ļ���outcome_f32��ֱ������ a*32768/597 �Ľ��ԼΪʵ�ʽ��������ֱ�������Ե�
//��������У������ת����f32����ģ��������ģֵ��ת���𰸲�ͬ��ʲô�����������һ����Ҫ��32��Ϊʲô
//fre_ic/(42682/1024)	   134615
	delay_init(168);
	uart_init(9600);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	MYGPIO_Init();
	// PF9->TIM14   PF9->TIM9
	TIM9_PWM_Init(100 - 1,168 - 1);
	TIM14_PWM_Init(100 - 1,84 - 1);
	//MYADC_Init();
	//START();
	MYDMA_ADC_Init();
	TIM5_IC_Init();
	
	
	uint16_t i = 0;//ѭ�����Ʊ���
	extern float fre_ic;
	
	arm_rfft_instance_q15 S;
	DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
	HMISends("page 0");
				HMISendb(0xff);	
	while(1)
	{
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
					adc_output_f32[i] = (float32_t)adc1_output[i];
				}
				arm_cmplx_mag_f32(adc_output_f32,adc1_outcome_f32,fft_size);		
				
				
				
				
				/* ���adc����fft�����ʵ��ѹֵ */
				for(i = 0;i<fft_size;i++)
				{
					adc1_voltage_f32[i] = adc1_outcome_f32[i]/4096*3.3;
				}
				
				
				
				/*�ҵ�һ��г������Ԫ���±�index*/
				if((fre_ic/(fre_sampling/fft_size)) - ((int)(fre_ic/(fre_sampling/fft_size))) >= 0.5)
				{
					index_arr = (int)(fre_ic/(fre_sampling/fft_size))+1;
				}
				else
				{
					index_arr = (int)(fre_ic/(fre_sampling/fft_size));
				}
				
				index_arr_static = index_arr;
				
				/* λ��Ѱ���Ż� */
				/* ���Ż������ǵ����ܻ���10���ڳ���һ��г�� */
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
							MAX = 0xfffff;   //��maxȡ�󣬱�����һ��ѭ�������ж�
						}
						
						
					}
				}
				i = 0;
				
				

				
				
				
				//RMS����
				arm_rms_f32(adc1_voltage_f32_over,cnt-1,p1);//����
				arm_rms_f32(adc1_voltage_f32_below,cnt,p2);//��ĸ
				ave = value1/value2;
				
				
				
				printf("t0.txt=\"%.3f%%\" ",ave*100);
				HMISendb(0xff);
				printf("t3.txt=\"%.3fhz\" ",fre_ic);
				HMISendb(0xff);
				cnt = 0;
				
				TIM_ITConfig(TIM5,TIM_IT_CC1,ENABLE);
	}
	

	
	}	

	
	
	
	void HMISendb(u8 k)		         //0xff���ͺ���
{		 
	u8 i;
	 for(i=0;i<3;i++)
	 {
	 if(k!=0)
	 	{
			USART_SendData(USART1,k);  //����һ���ֽ�
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){};//�ȴ����ͽ���
		}
	 else 
	 return ;

	 } 
} 
void HMISends(char *buf1)		  //�ַ������ͺ���
{
	u8 i=0;
	while(1)
	{
	 if(buf1[i]!=0)
	 	{
			USART_SendData(USART1,buf1[i]);  //����һ���ֽ�
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){};//�ȴ����ͽ���
		 	i++;
		}
	 else 
	 return ;

		}
	}


	
	
	

	


	
	
	
		
		




