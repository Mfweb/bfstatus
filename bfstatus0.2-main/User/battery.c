/**
  ******************************************************************************
  * @file    battery.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 电池电量检测
  ******************************************************************************
  */
#include "battery.h"

void battery_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource0;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct); 
	ADC_Cmd(ADC1, ENABLE);   
	ADC_ResetCalibration(ADC1); 
	while(ADC_GetResetCalibrationStatus(ADC1)); 
	ADC_StartCalibration(ADC1); 
	while(ADC_GetCalibrationStatus(ADC1));  
}

void battery_read(void)
{
	uint16_t ADC_data = 0;
	uint8_t	i,id;
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_55Cycles5);
	
	for(i=16;i>0;i--)
	{
		ADC_SoftwareStartConvCmd(ADC1,ENABLE);
		do
		{
			id = ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC);
		}
		while(!id);
		ADC_data += ADC_GetConversionValue(ADC1);
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	}
	battery.adc_data = ADC_data >> 4;//取 16 次采样平均值
	battery.now_value = (3.67f/2.77f)*((float)battery.adc_data/4096.0f*3.3f);//计算当前电量
	if(battery.now_value < 3.7)flag.battery_alarm = 1;//低于3.7V则启动低电量报警
	else flag.battery_alarm = 0;
}
