#ifndef __LED_H__
#define __LED_H__
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#define LED1 	PAout(4)
#define LED2 	PAout(8)
#define LED3 	PBout(4)
#define LED4 	PBout(5)

#define ON		0
#define OFF		!ON

void start_led(void);
void led_init(void);
void led_ref(void);
#endif
