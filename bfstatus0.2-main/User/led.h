#ifndef __LED_H__
#define __LED_H__
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#define LED1 	PCout(14)
#define LED2 	PBout(14)
#define LED3 	PBout(15)
#define LED4 	PCout(13)
#define ON		0
#define OFF		!ON

void start_led(void);
void led_init(void);
void led_ref(void);
#endif
