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
#include "stm32f10x_dma.h"
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

extern uint8_t Cal_Sel;
extern uint8_t TX_BUF[16];
extern uint8_t RX_BUF[16];
uint8_t send_hold = 0;
uint8_t send_flip = 0;
int main(void)
{
	DelayMs(500);
	key_led_init();
	init_i2c();
	oled_init();
	YG_INIT();
	usart_init();
	SPI_NRF_Init();
	if(NRF_Check()==ERROR)
	{
		oled_printf(0,1,"nRF error!");
		while(1)
		{
			LED2 = !LED2;
			DelayMs(1000);
		}
	}
	NRF_RX_Mode();
	
	wait_cab(1);
	menu_init();
	time_init();
	while(1)
	{
		power_get();
		menu_loop();
		if(Cal_Sel==4)
		{
			Cal_Sel = 0;
			wait_cab(0);
		}

		if(KEY5==KEY_DOWN)
		{
			DelayMs(10);
			while(KEY5==KEY_DOWN);
			send_hold = 1;
		}
		
		if(KEY3==KEY_DOWN)
		{
			DelayMs(10);
			while(KEY3==KEY_DOWN);
			send_flip = 1;
		}
		LED3 = !LED3;
	}
}
