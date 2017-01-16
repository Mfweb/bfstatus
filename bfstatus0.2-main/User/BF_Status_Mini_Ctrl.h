#ifndef __BF_STATUS_MINI_CTRL_H__
#define __BF_STATUS_MINI_CTRL_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_Global.h"
#include "BF_Status_Mini_System.h"
#include "BF_Status_Mini_Math.h"
#include "IMU.h"

void Control(void);
void CorePID(void);
void Motor_OutPut(void);
void Integral_Clear(void);
#endif
