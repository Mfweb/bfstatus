#include "key_led.h"


void key_led_init(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_QuickInit(GPIOC,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15,GPIO_Mode_IN_FLOATING);//上3个按键
	GPIO_QuickInit(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5,GPIO_Mode_IN_FLOATING);//下三个按键
	
	GPIO_QuickInit(GPIOB,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_8|GPIO_Pin_9,GPIO_Mode_Out_PP);//4个LED
	//PBout(12) = PBout(13) = PBout(14) = PBout(15) = 1;
}

