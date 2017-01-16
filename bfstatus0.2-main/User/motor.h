#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f10x.h"
#include "stm32fx_delay.h"
#include "stm32f10x_tim.h"

void Motor_Stop(void);
void motor_init(void);
void motor_output(int16_t M1,int16_t M2,int16_t M3,int16_t M4);
#endif
