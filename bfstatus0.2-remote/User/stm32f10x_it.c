/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "nrf24l01.h"
#include "yg.h"
#include "oled.h"
#include "key_led.h"

uint8_t TX_BUF[16];
uint8_t RX_BUF[16];
extern uint8_t Cal_Sel;
extern uint8_t send_hold;
extern uint8_t send_flip;
void TIM3_IRQHandler(void)		    //10ms中断一次
{
	uint8_t i;
	if(TIM3->SR & TIM_IT_Update)
	{
    
		for(i=0;i<32;i++)RX_BUF[i] = 0x00;
		NRF_Rx_Dat(RX_BUF);
		Get_YG();
		TX_BUF[0] = (YG_DATA.dat.pitch >> 8);
		TX_BUF[1] = (uint8_t)YG_DATA.dat.pitch;
		
		TX_BUF[2] = (YG_DATA.dat.roll >> 8);
		TX_BUF[3] = (uint8_t)YG_DATA.dat.roll;
		
		TX_BUF[4] = (YG_DATA.dat.yaw >> 8);
		TX_BUF[5] = (uint8_t)YG_DATA.dat.yaw;
		
		TX_BUF[6] = (YG_DATA.dat.throttle >> 8);
		TX_BUF[7] = (uint8_t)YG_DATA.dat.throttle;
		TX_BUF[8] = 0x00;
		if(Cal_Sel==1)
		{
			Cal_Sel = 0;
			TX_BUF[8]|=0x01;//校准加速度计
		}
		
		if(Cal_Sel==2)
		{
			Cal_Sel = 0;
			TX_BUF[8]|=0x02;//校准陀螺仪
		}
		
		if(Cal_Sel==3)
		{
			Cal_Sel = 0;
			TX_BUF[8]|=0x10;//校准地磁
		}
		
		if(KEY6==KEY_DOWN)
			TX_BUF[8]|=0x04;//解锁
		
		
		
		TX_BUF[8]|=0x08;	//请求返回数据
		
		if(send_hold)
		{
			send_hold = 0;
			TX_BUF[8]|=0x20;//定高
		}
		
		if(send_flip)
		{
			send_flip = 0;
			TX_BUF[8]|=0x40;
		}
//		
//		USART_SendData(USART1,0x55);while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
//		USART_SendData(USART1,0x55);while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
//		for(i=0;i<8;i++)
//		{
//			USART_SendData(USART1,TX_BUF[i]);while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
//		}
//		USART_SendData(USART1,0xAA);while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
//		USART_SendData(USART1,0xAA);while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
//		if(changed)
		NRF_Tx_Dat(TX_BUF);
		LED2 = !LED2;
		TIM3->SR = ~TIM_FLAG_Update;//清除中断标志
	}
}
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  这里用来实现叫高精度延迟  stm32fx_delay.c  By Mfweb 2015.03.05
  * @retval None
  */
void SysTick_Handler(void)
{

}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET) //确保是否产生了EXTI Line中断
	{
		EXTI_ClearITPendingBit(EXTI_Line0);     //清除中断标志位
	}  
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
