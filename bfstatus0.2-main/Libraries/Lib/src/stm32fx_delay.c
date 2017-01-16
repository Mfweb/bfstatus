#include "stm32fx_delay.h"

__IO uint32_t TimingDelay;


void Init_SysTick(void)
{
	if(SysTick_Config(SystemCoreClock / 10000))
	while(1);
}

void DelayMs(__IO uint32_t nTime)
{
	DelayUs(nTime*1000);
//	TimingDelay = nTime*1000;
//	while(TimingDelay != 0);
}

void DelayUs(__IO uint32_t nTime)
{
	int i,j;
	for(i=0;i<nTime;i++)
		for(j=0;j<2;j++);
//	TimingDelay = nTime;
//	while(TimingDelay != 0);
}

void delay(u32 x)
{
    u32 i,j;
	for(i=0;i<x;i++)
	   for(j=0;j<500;j++);
}

