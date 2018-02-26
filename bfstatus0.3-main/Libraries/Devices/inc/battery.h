#ifndef __BATTERY_H__
#define __BATTERY_H__
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "BF_Status_Mini_Global.h"

typedef struct
{
	void (*Init)(void);
	void (*Read)(void);
	void (*Reboot)(uint8_t mode);
	float now_value;		//电池电量
	uint16_t adc_data;	//电池电量原始AD
}data_bettery_t;

#define POWER_KEY GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)
#define KEY_DOWN Bit_RESET
#define KEY_UP Bit_SET

#define POWER_ON() GPIO_SetBits(GPIOA, GPIO_Pin_9)
#define POWER_OFF() GPIO_ResetBits(GPIOA, GPIO_Pin_9)

#define ENTER_STOP 1
#define ENTER_BOOTLOADER 2

extern data_bettery_t gBattery;								//电池状态
#endif
