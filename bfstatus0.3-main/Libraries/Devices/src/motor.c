/**
  ******************************************************************************
  * @file    motor.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 电机
  ******************************************************************************
  */
#include "motor.h"
#include "stm32f10x_gpio.h"
#include "BF_Status_Mini_Conf.h"
#include "BF_Status_Mini_Math.h"
#include "battery.h"


void Motor_Stop(void);
void motor_init(void);
void motor_output(void);

motor_t gMotor={
	.Init = &motor_init,
	.Set = &motor_output,
	.Stop = &Motor_Stop,
	.M1 = 0,
	.M2 = 0,
	.M3 = 0,
	.M4 = 0
};

void Motor_Stop(void)
{
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
}
//72Khz PWM*4
void motor_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);   //打开定时器2时钟  
	// 设置GPIO功能
	GPIO_QuickInit(GPIOA,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3,GPIO_Mode_AF_PP);
	// 复位定时器
	TIM_DeInit(TIM2);
	// 配置计时器
	TIM_TimeBaseStructure.TIM_Period = 999;//计数上限
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 72000000) - 1;//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	// 配置TIM2为PWM输出模式
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
	// 启动计时器。
	TIM_Cmd(TIM2,ENABLE);
	Motor_Stop();
}

int thrust_bat_compensated(int th)
{
	int ratio = th;
//		//二次函数油门电压补偿
	float thrust = ((float)th / 1000) * 60;
	float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
	float supply_voltage = gBattery.now_value;
	float percentage = volts / supply_voltage;
	percentage = percentage > 1.0f ? 1.0f : percentage;
	ratio = percentage * 1000;
	return ratio;		
}

void motor_output(void)
{
	gMotor.M1+=MOTOT_DIE1;
	gMotor.M2+=MOTOT_DIE2;
	gMotor.M3+=MOTOT_DIE3;
	gMotor.M4+=MOTOT_DIE4;
	
	gMotor.M1 = thrust_bat_compensated(gMotor.M1);
	gMotor.M2 = thrust_bat_compensated(gMotor.M2);
	gMotor.M3 = thrust_bat_compensated(gMotor.M3);
	gMotor.M4 = thrust_bat_compensated(gMotor.M4);
	
	gMotor.M1 = math_limit(gMotor.M1,999,0);
	gMotor.M2 = math_limit(gMotor.M2,999,0);
	gMotor.M3 = math_limit(gMotor.M3,999,0);
	gMotor.M4 = math_limit(gMotor.M4,999,0);
	
	TIM2->CCR3 = gMotor.M1;
	TIM2->CCR4 = gMotor.M2;
	TIM2->CCR2 = gMotor.M3;
	TIM2->CCR1 = gMotor.M4;
}
