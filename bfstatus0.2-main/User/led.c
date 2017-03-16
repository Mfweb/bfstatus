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
	if(flag.CalibratingACC)//正在标定加速度计   LED1闪烁
	{
		LED1 = !LED1;
		LED2 = 1;
		LED3 = 1;
		LED4 = 1;
	}
	else if(flag.CalibratingGYR)//正在标定陀螺仪   LED2闪烁
	{
		LED1 = 1;
		LED2 = !LED2;
		LED3 = 1;
		LED4 = 1;
	}
	else if(!flag.Lock)	//已经解锁  循环闪烁
	{
		LED1 = !LED2;
		LED2 = !LED3;
		LED3 = !LED4;
		LED4 = !LED1;
	}
	else if(flag.CalibratingMAG==1)//正在标定Z轴罗盘  LED3常亮
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 0;
		LED4 = 1;
	}
	else if(flag.CalibratingMAG==2)//正在标定X轴罗盘  LED4、3常亮
	{
		LED1 = 1;
		LED2 = 1;
		LED3 = 0;
		LED4 = 0;
	}
	else if(flag.CalibratingMAG==3)//正在标定Y轴罗盘  LED4、3、1常亮
	{
		LED1 = 0;
		LED2 = 1;
		LED3 = 0;
		LED4 = 0;
	}
	else//其他状态  4个LED常亮
	{
		LED1 = 0;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
	}
}
