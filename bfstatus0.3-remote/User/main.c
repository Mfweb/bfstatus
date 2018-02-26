/**
  ******************************************************************************
  * @file    main.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 遥控器程序(STM32F103CBT6) + OLED12864 + nRF24L01
	*						V0.1版本的AT24C02没有挂接在OLED的IIC总线上，建议修改
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "stm32fx_delay.h"
#include "stm32f10x_adc.h"
#include "oled.h"
#include "yg.h"
#include "nrf24l01.h"
#include "debug_usart.h"
#include <stdio.h>
#include "key_led.h"
#include "systime.h"
#include "menu.h"
#include "battery.h"
#include "app.h"

// key1:功能选择  key2:执行功能  key3:翻滚  key4:关机  key5:定高 key6:解锁

int main(void)
{
	key_led_init();
	//DelayMs(500);
	//GPIO_QuickInit(GPIOA,GPIO_Pin_10,GPIO_Mode_Out_PP);
	I2C_INIT();
	oled_init();
	YG_INIT();
	usart_init();
	SPI_NRF_Init();
	wait_cab(1);
	menu_init();
	xTaskCreate(Task_Start, "Start!", 100,NULL, 1, &startTaskHandle);
	vTaskStartScheduler();//启动调度
	while(1);
}
