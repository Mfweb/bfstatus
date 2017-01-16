#ifndef __KEY_LED_H__
#define __KEY_LED_H__
#include "stm32f10x.h"
#define LED1 PBout(12)
#define LED2 PBout(13)
#define LED3 PBout(8)
#define LED4 PBout(9)

#define KEY1 PCin(13)
#define KEY2 PCin(14)
#define KEY3 PCin(15)

#define KEY4 PBin(3)
#define KEY5 PBin(4)
#define KEY6 PBin(5)

#define ON 0
#define OFF !ON
#define KEY_DOWN Bit_RESET
#define KEY_UP   Bit_SET

void key_led_init(void);
#endif
