#include "adc.h"
#include "delay.h"

#define poolsize 1024  //此处涉及采样问题，待定

uint16_t datapool_adc[poolsize];
uint8_t t = 0;
int timeout_max = 1000;



//GPIO初始化
void MYGPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//PA7初始化
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
}




//ADC1->PA7 通道7  
 void MYADC_Init(void)
{
	ADC_DeInit();
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_RegularChannelConfig(ADC1,ADC_Channel_7,1,ADC_SampleTime_480Cycles);
	ADC_Init(ADC1,&ADC_InitStructure);
	
	

	


	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);	
	
	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1,ENABLE);

}





//DMA2 通道0 数据流0  注意 这里选用多重adc，那么外设设为cdr，应该是不需要考虑dma-adc映射表的
void MYDMA_ADC_Init(void)
{
	DMA_Cmd(DMA2_Stream0,DISABLE);//上次数据传输未完成时，失能指令无效
	while(DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);
	//DMA_DeInit(DMA2_Stream0);
	
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	DMA_InitStructure.DMA_BufferSize = poolsize;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)datapool_adc;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	
	DMA_Cmd(DMA2_Stream0,ENABLE);
	/*debug使用
	while((DMA_GetCmdStatus(DMA2_Stream0)!=ENABLE)&&(timeout_max-->0))
	{
		flag = 1;
	}
*/

}

void START(void)
{
	ADC_SoftwareStartConv(ADC1);
}


