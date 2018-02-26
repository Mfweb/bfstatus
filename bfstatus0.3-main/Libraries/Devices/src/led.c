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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_QuickInit(GPIOA,GPIO_Pin_4,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOA,GPIO_Pin_8,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_4,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_5,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOA,GPIO_Pin_10,GPIO_Mode_IPD);
	LED1=LED2=LED3=LED4=OFF;
}
void start_led(void)
{
	LED1=LED2=LED3=LED4=ON;
	DelayMs(100);
	LED1=LED2=LED3=LED4=OFF;
	DelayMs(100);
	LED1=LED2=LED3=LED4=ON;
	DelayMs(100);
	LED1=LED2=LED3=LED4=OFF;
}
void led_ref(void)
{
	if(flag.CalibratingACC || flag.CalibratingVel)//正在标定加速度计   LED1闪烁
	{
		LED1 = !LED1;
		LED2 = OFF;
		LED3 = OFF;
		LED4 = OFF;
	}
	else if(flag.CalibratingGYR)//正在标定陀螺仪   LED2闪烁
	{
		LED1 = OFF;
		LED2 = !LED2;
		LED3 = OFF;
		LED4 = OFF;
	}
	else if(!flag.Lock)	//已经解锁  循环闪烁
	{
		static uint8_t xh_ct = 0;
		if(xh_ct==0)
		{
			LED1 = ON;
			LED2 = OFF;
			LED3 = OFF;
			LED4 = OFF;
			xh_ct=1;
		}
		else if(xh_ct==1)
		{
			LED1 = OFF;
			LED2 = ON;
			LED3 = OFF;
			LED4 = OFF;
			xh_ct=2;
		}
		else if(xh_ct==2)
		{
			LED1 = OFF;
			LED2 = OFF;
			LED3 = ON;
			LED4 = OFF;
			xh_ct=3;
		}
		else if(xh_ct==3)
		{
			LED1 = OFF;
			LED2 = OFF;
			LED3 = OFF;
			LED4 = ON;
			xh_ct=0;
		}
	}
	else if(flag.CalibratingMAG==1)//正在标定Z轴罗盘  LED3常亮
	{
		LED1 = OFF;
		LED2 = OFF;
		LED3 = ON;
		LED4 = OFF;
	}
	else if(flag.CalibratingMAG==2)//正在标定X轴罗盘  LED4、3常亮
	{
		LED1 = OFF;
		LED2 = OFF;
		LED3 = ON;
		LED4 = ON;
	}
	else if(flag.CalibratingMAG==3)//正在标定Y轴罗盘  LED4、3、1常亮
	{
		LED1 = ON;
		LED2 = OFF;
		LED3 = ON;
		LED4 = ON;
	}
	else//其他状态  4个LED常亮
	{
		LED1 = ON;
		LED2 = ON;
		LED3 = ON;
		LED4 = ON;
	}
}
