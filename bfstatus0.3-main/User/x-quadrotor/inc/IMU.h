#ifndef __IMU_H__
#define __IMU_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_System.h"
#include "BF_Status_Mini_Global.h"
#include "BF_Status_Mini_Math.h"
#include "BF_Status_Mini_Filter.h"
#include <math.h>
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
struct __Ang
{
  float Pitch;
  float Roll;
  float Yaw;
};


/*  重力  */
typedef __IO struct
{
  float x;
  float y;
  float z;
}gravity_t;
/* 四元数 */
typedef __IO struct
{
  float q0;
  float q1;
  float q2;
  float q3;
}quaternion_t;
typedef struct imu_t imu_t;
extern imu_t gIMU;						//姿态角
extern quaternion_t gNumQ;							//四元数
//IMU
struct imu_t
{
	void (*Get)(void);
	struct __Ang radian;//弧度值
	struct __Ang angle;//角度值
};
#endif
