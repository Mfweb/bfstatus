#include "stm32fx_delay.h"

__IO uint32_t TimingDelay;


void Init_SysTick(void)
{
	if(SysTick_Config(SystemCoreClock / 10000))
	while(1);
}

void DelayMs(__IO uint16_t nTime)
{
	uint16_t i;
	while(nTime--)
	{
		i=10000;
		while(i--);
	}
//	TimingDelay = nTime*1000;
//	while(TimingDelay != 0);
}

void DelayUs(__IO uint16_t nTime)
{
	uint16_t i;
	while(nTime--)
	{
		i=10;
		while(i--);
	}
//	TimingDelay = nTime;
//	while(TimingDelay != 0);
}

