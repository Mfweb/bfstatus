/**
  ******************************************************************************
  * @file    yg.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini ҡ�����ݲɼ�
  ******************************************************************************
  */
#include "yg.h"
#include "oled.h"
#include "stm32fx_delay.h"
#include "key_led.h"
#include "eeprom.h"

yg_dt volatile YG_DATA;
void YG_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	//ADC input init PA0-3 AD0-3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource0|GPIO_PinSource1|GPIO_PinSource2|GPIO_PinSource3;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource0;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = 5;
	ADC_Init(ADC1, &ADC_InitStruct); 
	ADC_Cmd(ADC1, ENABLE);   
	ADC_ResetCalibration(ADC1); 
	while(ADC_GetResetCalibrationStatus(ADC1)); 
	ADC_StartCalibration(ADC1); 
	while(ADC_GetCalibrationStatus(ADC1));  
}

uint16_t Read_ADC1_MultiChannel(uint8_t channNo)
{  
	uint16_t ADC_data = 0;
	uint8_t	i,id;
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	ADC_RegularChannelConfig(ADC1,channNo,1,ADC_SampleTime_7Cycles5);
	
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
	ADC_data = ADC_data >> 4;//ȡ 16 �β���ƽ��ֵ
	return ADC_data;
}

void Get_YG(void)
{
	YG_DATA.org.pitch			= Read_ADC1_MultiChannel(3);
	YG_DATA.org.roll			= Read_ADC1_MultiChannel(2);
	YG_DATA.org.yaw				= Read_ADC1_MultiChannel(0);
	YG_DATA.org.throttle 	= Read_ADC1_MultiChannel(1);
	
	//��һ��֮�� 0-1000
	YG_DATA.nor.pitch			= (((float)YG_DATA.org.pitch		- YG_DATA.min.pitch)		/ (YG_DATA.max.pitch-YG_DATA.min.pitch))				* 1000;
	YG_DATA.nor.roll			= (((float)YG_DATA.org.roll			- YG_DATA.min.roll)			/ (YG_DATA.max.roll-YG_DATA.min.roll))					* 1000;
	YG_DATA.nor.yaw				= (((float)YG_DATA.org.yaw			- YG_DATA.min.yaw)			/ (YG_DATA.max.yaw-YG_DATA.min.yaw))						* 1000;
	YG_DATA.nor.throttle	= (((float)YG_DATA.org.throttle - YG_DATA.min.throttle) / (YG_DATA.max.throttle-YG_DATA.min.throttle))	* 1000;
	//����1000 ��ȥ��㾲ֵ̬ 1000-2000 �е�1500����
	YG_DATA.dat.pitch			= YG_DATA.nor.pitch 					+ 1000 - YG_DATA.sta.pitch;
	YG_DATA.dat.roll			= YG_DATA.nor.roll 						+ 1000 - YG_DATA.sta.roll;
	YG_DATA.dat.yaw				= YG_DATA.nor.yaw							+ 1000 - YG_DATA.sta.yaw;
	YG_DATA.dat.throttle	= 1000 - YG_DATA.nor.throttle + 1000 - YG_DATA.sta.throttle;
	if(YG_DATA.dat.throttle < 1000)YG_DATA.dat.throttle=1000;//�����޷�
	//��ҡ��������У��  ��֤���� ���¶Գ�
	if(YG_DATA.dat.pitch>1500)YG_DATA.dat.pitch = (YG_DATA.dat.pitch-1500) * YG_DATA.ratio_n.pitch + 1500;
	else YG_DATA.dat.pitch = (YG_DATA.dat.pitch - 1500) * YG_DATA.ratio_p.pitch + 1500;
	
	if(YG_DATA.dat.roll>1500)YG_DATA.dat.roll = (YG_DATA.dat.roll - 1500) * YG_DATA.ratio_n.roll + 1500;
	else YG_DATA.dat.roll = (YG_DATA.dat.roll - 1500) * YG_DATA.ratio_p.roll + 1500;
	
	if(YG_DATA.dat.yaw>1500)YG_DATA.dat.yaw = (YG_DATA.dat.yaw - 1500) * YG_DATA.ratio_n.yaw + 1500;
	else YG_DATA.dat.yaw = (YG_DATA.dat.yaw - 1500) * YG_DATA.ratio_p.yaw + 1500;
}
//���ݹ�һ��
float ScaleLinear(float x, float x_end, float deadband)
{
	if (x>deadband)
		return (x-deadband)/(x_end - deadband);
	else if(x<-deadband)
		return (x+deadband)/(x_end - deadband);
	else
		return 0.0f;
}
//���ҡ��ϵ������������ ������ƽ�ȣ�
void get_ratio(void)
{
	float temp_max[3],temp_min[3];
	temp_max[0] = 40.0f* ScaleLinear((float)(YG_DATA.max.pitch-1500),500.0f,50.0f);
	temp_max[1] = 40.0f* ScaleLinear((float)(YG_DATA.max.roll -1500),500.0f,50.0f);
	temp_max[2] = 180.0f/3.1415926f * ScaleLinear((float)(YG_DATA.max.yaw-1500),500.0f,70.0f);
	
	temp_min[0] = -40.0f* ScaleLinear((float)(YG_DATA.min.pitch-1500),500.0f,50.0f);
	temp_min[1] = -40.0f* ScaleLinear((float)(YG_DATA.min.roll -1500),500.0f,50.0f);
	temp_min[2] = -180.0f/3.1415926f * ScaleLinear((float)(YG_DATA.min.yaw-1500),500.0f,70.0f);
	
	if(temp_min[0] < temp_max[0])
	{
		YG_DATA.ratio_n.pitch = temp_min[0]/temp_max[0];
		YG_DATA.ratio_p.pitch = 1.0f;
	}
	else
	{
		YG_DATA.ratio_p.pitch = temp_max[0]/temp_min[0];
		YG_DATA.ratio_n.pitch = 1.0f;
	}
	
	if(temp_min[1] < temp_max[1])
	{
		YG_DATA.ratio_n.roll = temp_min[1]/temp_max[1];
		YG_DATA.ratio_p.roll = 1.0f;
	}
	else
	{
		YG_DATA.ratio_p.roll = temp_max[1]/temp_min[1];
		YG_DATA.ratio_n.roll = 1.0f;
	}
	
	if(temp_min[2] < temp_max[2])
	{
		YG_DATA.ratio_n.yaw = temp_min[2]/temp_max[2];
		YG_DATA.ratio_p.yaw = 1.0f;
	}
	else
	{
		YG_DATA.ratio_p.yaw = temp_max[2]/temp_min[2];
		YG_DATA.ratio_n.yaw = 1.0f;
	}
	
	
//	oled_printf(0,0,"%.3f  %.3f  ",YG_DATA.ratio_n.pitch,YG_DATA.ratio_p.pitch);
//	oled_printf(0,1,"%.3f  %.3f  ",YG_DATA.ratio_n.roll,YG_DATA.ratio_p.roll);
//	oled_printf(0,2,"%.3f  %.3f  ",YG_DATA.ratio_n.yaw,YG_DATA.ratio_p.yaw);
//	while(1);
}
//ҡ�˱궨
void wait_cab(uint8_t check)
{
	if(check)//�����Ҫ���EEPROM
	{
		if(check_null())//���EEPROM����������
		{
			read_ee();//��ȡ���� 
			get_ratio();//�������
			return;//����
		}
	}
	YG_DATA.max.pitch			= 0;
	YG_DATA.max.roll			= 0;
	YG_DATA.max.yaw				= 0;
	YG_DATA.max.throttle	= 0;
	
	YG_DATA.min.pitch			= 4096;
	YG_DATA.min.roll			= 4096;
	YG_DATA.min.yaw				= 4096;
	YG_DATA.min.throttle	= 4096;

	oled_clear();
	oled_printf(0,0,"     Start Cal");
	oled_printf(0,1,"     max   min");
	while(KEY3)//�궨ҡ�����ֵ����Сֵ
	{
		YG_DATA.org.pitch			= Read_ADC1_MultiChannel(3);
		YG_DATA.org.roll			= Read_ADC1_MultiChannel(2);
		YG_DATA.org.yaw				= Read_ADC1_MultiChannel(0);
		YG_DATA.org.throttle 	= Read_ADC1_MultiChannel(1);
		
		if(YG_DATA.min.pitch		> YG_DATA.org.pitch)		YG_DATA.min.pitch			= YG_DATA.org.pitch;
		if(YG_DATA.min.roll			> YG_DATA.org.roll)			YG_DATA.min.roll			= YG_DATA.org.roll;
		if(YG_DATA.min.yaw			> YG_DATA.org.yaw)			YG_DATA.min.yaw				= YG_DATA.org.yaw;
		if(YG_DATA.min.throttle > YG_DATA.org.throttle)	YG_DATA.min.throttle	= YG_DATA.org.throttle;
		
		if(YG_DATA.max.pitch		< YG_DATA.org.pitch)		YG_DATA.max.pitch			= YG_DATA.org.pitch;
		if(YG_DATA.max.roll			< YG_DATA.org.roll)			YG_DATA.max.roll			= YG_DATA.org.roll;
		if(YG_DATA.max.yaw			< YG_DATA.org.yaw)			YG_DATA.max.yaw				= YG_DATA.org.yaw;
		if(YG_DATA.max.throttle < YG_DATA.org.throttle)	YG_DATA.max.throttle	= YG_DATA.org.throttle;
		
		oled_printf(0,2,"%d: %5d %5d",1,YG_DATA.max.pitch,YG_DATA.min.pitch);
		oled_printf(0,3,"%d: %5d %5d",2,YG_DATA.max.roll,YG_DATA.min.roll);
		oled_printf(0,4,"%d: %5d %5d",3,YG_DATA.max.yaw,YG_DATA.min.yaw);
		oled_printf(0,5,"%d: %5d %5d",4,YG_DATA.max.throttle,YG_DATA.min.throttle);
		DelayMs(20);
	}
	oled_clear();
	while(KEY3==KEY_DOWN);//����KEY3��ʱ��Ҫ��֤ҡ��ȫ������ �����������
	YG_DATA.max.throttle  -= 100;//������Ԥ��100����  ��ֹ��΢һ��ҡ�˾ʹ���
	YG_DATA.sta.pitch 		= 0;
	YG_DATA.sta.roll 			= 0;
	YG_DATA.sta.yaw 			= 0;
	YG_DATA.sta.throttle 	= 0;
	YG_DATA.ratio_n.pitch = YG_DATA.ratio_n.roll = YG_DATA.ratio_n.yaw = 1.0f;//������1
	YG_DATA.ratio_p.pitch = YG_DATA.ratio_p.roll = YG_DATA.ratio_p.yaw = 1.0f;
	Get_YG();//���һ������  ��ʱ��Ϊҡ�����м�λ��  ���������λ��
	YG_DATA.sta.pitch 		= YG_DATA.dat.pitch 		- 1500;//���������ĵ��ƫ��
	YG_DATA.sta.roll 			= YG_DATA.dat.roll 			- 1500;
	YG_DATA.sta.yaw 			= YG_DATA.dat.yaw 			- 1500;
	YG_DATA.sta.throttle 	= YG_DATA.dat.throttle 	- 1000;
	save_ee();//����ҡ�˱궨��
	get_ratio();//����ҡ�˱���
}
