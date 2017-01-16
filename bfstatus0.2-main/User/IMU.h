#ifndef __IMU_H__
#define __IMU_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_System.h"
#include "BF_Status_Mini_Global.h"
#include "BF_Status_Mini_Math.h"
#include "BF_Status_Mini_Filter.h"
#include <math.h>
#include "mpu6050.h"
#include "hmc5883l.h"


Gravity Quaternion_vectorGravity(Quaternion *pNumQ);
void Quaternion_ToAngE(Quaternion *pNumQ,EulerAngle *pAngE);
void Quaternion_Normalize(Quaternion *pNumQ);
void Quaternion_RungeKutta(Quaternion *pNumQ,float GyrX,float GyrY,float GyrZ,float helfTimes);
void Angle_Get(void);
#endif
