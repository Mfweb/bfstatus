#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H
#include "stm32f10x.h"
float kalmanUpdate(const float gyro_m,const float incAngle,float dt);
#endif
