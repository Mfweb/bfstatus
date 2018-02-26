/**
  ******************************************************************************
  * @file    battery.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 电池电量检测
  ******************************************************************************
  */
#include "battery.h"
#include "stm32f10x_i2c.h"
#include <stdio.h>
#include "stm32fx_delay.h"
#include "BF_Status_Mini_System.h"
#include "misc.h"
#include "stm32f10x_flash.h"
void battery_read(void);
void battery_init(void);
void Set_SYS_Mode(uint8_t mode);
//电池数据
data_bettery_t gBattery={
	.Init = &battery_init,
	.Read = &battery_read,
	.Reboot = &Set_SYS_Mode,
	.now_value = 0,
	.adc_data = 0
};
/*  3.00, // 00%
  3.78, // 10%
  3.83, // 20%
  3.87, // 30%
  3.89, // 40%
  3.92, // 50%
  3.96, // 60%
  4.00, // 70%
  4.04, // 80%
  4.10  // 90%*/
void battery_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource0;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct); 
	ADC_Cmd(ADC1, ENABLE);   
	ADC_ResetCalibration(ADC1); 
	while(ADC_GetResetCalibrationStatus(ADC1)); 
	ADC_StartCalibration(ADC1); 
	while(ADC_GetCalibrationStatus(ADC1));
	
	GPIO_QuickInit(GPIOA,GPIO_Pin_9,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_3,GPIO_Mode_IPU);
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) == Bit_RESET){}
	POWER_ON();
	DelayMs(50);
}

__asm void SystemReset(void)
{
 MOV R0, #1           //; 
 MSR FAULTMASK, R0    //; 清除FAULTMASK 禁止一切中断产生
 LDR R0, =0xE000ED0C  //;
 LDR R1, =0x05FA0004  //; 
 STR R1, [R0]         //; 系统软件复位   
 
deadloop
		B deadloop        //; 死循环使程序运行不到下面的代码
}

/**
 * @brief  配置系统复位之后的模式
 * @code
 *      Set_SYS_Mode(ENTER_STOP);
 *      系统复位后进入STOP模式
 * @endcode
 * @param  mode:  模式配置ENTER_STOP:进入STOP模式   ENTER_BOOTLOADER:进入bootLoader模式
 * @retval 此函数执行后不再返回
 */
void Set_SYS_Mode(uint8_t mode)
{
	//FLASH_Unlock();
	//FLASH_ErasePage(FLASH_BASE_ADDR + 0x800);//擦除最后1个page
	//FLASH_ProgramHalfWord(FLASH_BASE_ADDR + 0x800,mode);//配置下次启动模式
	//FLASH_Lock();
	uint8_t * addr = NULL;
	addr = (uint8_t*)(0x20000000+0x5000-4);
	*addr = mode;
	SystemReset();
	while(1);
}

/**
 * @brief  获取一次电池数据
 * @code
 *      battery_read();
 * @endcode
 * @param  void
 * @retval void
 */
void battery_read(void)
{
	uint16_t ADC_data = 0;
	uint8_t	i,id;
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_55Cycles5);
	
	for(i=16;i>0;i--)
	{
		ADC_SoftwareStartConvCmd(ADC1,ENABLE);
		do
		{
			id = ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC);
		}
		while(!id);
		ADC_data += ADC_GetConversionValue(ADC1);
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	}
	gBattery.adc_data = ADC_data >> 4;//取 16 次采样平均值
	gBattery.now_value = (3.67f/2.77f)*((float)gBattery.adc_data/4096.0f*3.3f);//计算当前电量
	if(gBattery.now_value <= 3.7)flag.BatteryAlarm = true;//低于3.7V则启动低电量报警(此警报仅用于禁止解锁)
	else flag.BatteryAlarm = false;
}
