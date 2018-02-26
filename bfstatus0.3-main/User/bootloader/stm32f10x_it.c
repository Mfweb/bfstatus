#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
void TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_IT_Update)
	{    
    TIM3->SR = ~TIM_FLAG_Update;//清除中断标志
	}
}

void NMI_Handler(void)
{}
void HardFault_Handler(void)
{while(1){}}
void MemManage_Handler(void)
{while(1){}}
void BusFault_Handler(void)
{while(1){}}
void UsageFault_Handler(void)
{while(1){}}
void DebugMon_Handler(void)
{}
void PendSV_Handler(void)
{}
void SysTick_Handler()
{}
