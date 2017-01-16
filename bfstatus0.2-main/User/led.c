/**
  ******************************************************************************
  * @file    led.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini LED
  ******************************************************************************
  */
#include "led.h"
#include "BF_Status_Mini_Global.h"
#include "stm32fx_delay.h"

void led_init(void)
{
	GPIO_QuickInit(GPIOB,GPIO_Pin_14,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_15,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOC,GPIO_Pin_13,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOC,GPIO_Pin_14,GPIO_Mode_Out_PP);
	PBout(14)=PBout(15)=PCout(14)=PCout(13)=1;
}
void start_led(void)
{
	PBout(14)=PBout(15)=PCout(14)=PCout(13)=ON;
	DelayMs(100);
	PBout(14)=PBout(15)=PCout(14)=PCout(13)=OFF;
	DelayMs(100);
	PBout(14)=PBout(15)=PCout(14)=PCout(13)=ON;
	DelayMs(100);
	PBout(14)=PBout(15)=PCout(14)=PCout(13)=OFF;
}
void led_ref(void)
{
	if(flag.CalibratingACC)//���ڱ궨���ٶȼ�   LED1��˸
	{
		LED1 = !LED1;
		LED2 = 1;
		LED3 = 1;
		LED4 = 1;
	}
	else if(flag.CalibratingGYR)//���ڱ궨������   LED2��˸
	{
		LED1 = 1;
		LED2 = !LED2;
		LED3 = 1;
		LED4 = 1;
	}
	else if(!flag.Lock)	//�Ѿ�����  ѭ����˸
	{
		LED1 = !LED2;
		LED2 = !LED3;
		LED3 = !LED4;
		LED4 = !LED1;
	}
	else if(flag.CalibratingMAG==1)//���ڱ궨Z������  LED3����
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 0;
		LED4 = 1;
	}
	else if(flag.CalibratingMAG==2)//���ڱ궨X������  LED4��3����
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 0;
		LED4 = 0;
	}
	else if(flag.CalibratingMAG==3)//���ڱ궨Y������  LED4��3��1����
	{
		LED1 = 0;
		LED2 = 1;
		LED3 = 0;
		LED4 = 0;
	}
	else//����״̬  4��LED����
	{
		LED1 = 0;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
	}
}
