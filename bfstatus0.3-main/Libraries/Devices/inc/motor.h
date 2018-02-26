#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f10x.h"
#include "stm32fx_delay.h"
#include "stm32f10x_tim.h"
typedef struct {
	void (*Init)(void);
	void (*Set)(void);
	void (*Stop)(void);
	int base;
	int M1;
	int M2;
	int M3;
	int M4;
}motor_t;

extern motor_t gMotor;
#endif
