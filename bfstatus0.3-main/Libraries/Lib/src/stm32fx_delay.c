#include "stm32fx_delay.h"

void DelayMs(__IO uint32_t nTime)
{
	DelayUs(nTime*1000);
}

void DelayUs(__IO uint32_t nTime)
{
	int i,j;
	for(i=0;i<nTime;i++)
		for(j=0;j<2;j++);
}

void delay(u32 x)
{
    u32 i,j;
	for(i=0;i<x;i++)
	   for(j=0;j<500;j++);
}

volatile uint32_t fac_us;
volatile uint32_t fac_ms;

void Init_SysTick(void)
{
	SysTick->CTRL &= 0xfffffffb;//控制寄存器，选择外部时钟即系统时钟的八分之一（HCLK/8；72M/8=9M）
	fac_us = 72/8;    //定义全局变量，即延时一微秒所需的的时钟周期数(72/8=9,单位为微妙)
	fac_ms = (u16)fac_us*1000; //一毫秒所需的时钟周期数（9000）
}

void DelayMs_Tick(__IO uint32_t nTime)
{
	DelayUs_Tick(nTime*1000);
}

void DelayUs_Tick(__IO uint32_t nTime)
{
	SysTick->LOAD = nTime * fac_us;          //时间加载    72M主频     
	SysTick->CTRL |= 0x01;             //开始倒数      
	while(!(SysTick->CTRL&(1<<16))); //等待时间到达   
	SysTick->CTRL=0X00000000;        //关闭计数器   
	SysTick->VAL=0X00000000;         //清空计数器     
}
