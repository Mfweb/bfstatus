/**
  ******************************************************************************
  * @file    motor.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini ���
  ******************************************************************************
  */
#include "motor.h"
#include "stm32f10x_gpio.h"
#include "BF_Status_Mini_Conf.h"

void Motor_Stop(void)
{
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
}
//24Khz PWM*4
void motor_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;    //���Ƶ��PWMƵ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //������A��ʱ�Ӻ͸���ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);   //�򿪶�ʱ��2ʱ��  
	// ����GPIO���ܡ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// ��λ��ʱ����
	TIM_DeInit(TIM2);
	// ���ü�ʱ����
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	TIM_TimeBaseStructure.TIM_Period = 999;		            //��������	
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	//pwmʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	// ����TIM2ΪPWM���ģʽ
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);
	TIM_OC4Init(TIM2,&TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	// ������ʱ����
	TIM_Cmd(TIM2,ENABLE);
}

void motor_output(int16_t M1,int16_t M2,int16_t M3,int16_t M4)
{
	M1+=MOTOT_DIE;
	M2+=MOTOT_DIE;
	M3+=MOTOT_DIE;
	M4+=MOTOT_DIE;
	if(M1>999)M1=999;
	if(M2>999)M2=999;
	if(M3>999)M3=999;
	if(M4>999)M4=999;
	
	if(M1<0)M1=0;
	if(M2<0)M2=0;
	if(M3<0)M3=0;
	if(M4<0)M4=0;
	
	TIM2->CCR3 = M1;
	TIM2->CCR4 = M2;
	TIM2->CCR2 = M3;
	TIM2->CCR1 = M4;
}
