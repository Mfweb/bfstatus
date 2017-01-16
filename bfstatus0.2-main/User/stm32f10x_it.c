#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "BF_Status_Mini_System.h"
#include "BF_Status_Mini_RC.h"
#include "BF_Status_Mini_Ctrl.h"
#include "IMU.h"
#include "ms5611.h"

void TIM3_IRQHandler(void)
{	
	if(TIM3->SR & TIM_IT_Update)
	{    
    TIM3->SR = ~TIM_FLAG_Update;//清除中断标志
		Time_slice();		//时间片
		RC_GetData();		//遥控数据获取
 		Angle_Get();		//姿态解算		
 		Control();			//控制输出
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
void SVC_Handler(void)
{}
void DebugMon_Handler(void)
{}
void PendSV_Handler(void)
{}
void SysTick_Handler(void)
{}
