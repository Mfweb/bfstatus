#ifndef __BATTERY_H__
#define __BATTERY_H__
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "BF_Status_Mini_Global.h"

void battery_read(void);
void battery_init(void);
#endif
