#ifndef __BF_STATUS_MINI_GLOBAL_H__
#define __BF_STATUS_MINI_GLOBAL_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_Conf.h"
#include "FreeRTOS.h"
#include "battery.h"
#include "task.h"
#include "semphr.h"
#include "nrf24l01.h"
/* Flag */
typedef struct
{
	_Bool							BatteryAlarm;				//低电量警告
	_Bool							LockYaw;						//航向锁定
	_Bool							IsBaro;							//气压计存在
	_Bool							IsMAG;							//地磁存在并且检测通过
	_Bool							Lock;								//是否锁定
	_Bool							HoldAltitude;				//定高
	_Bool							ZeroGravity;				//0G状态
	_Bool							PowerOff;						//关机
	uint8_t						CalibratingACC; 		//正在标定加速度计
	uint8_t						CalibratingVel;			//正在校准静止时Z轴速度
	uint8_t						CalibratingGYR;			//正在标定陀螺仪
	uint8_t						CalibratingMAG; 		//正在标定地磁
	uint8_t						Flipping;						//是否正在翻转
	uint8_t						Throwing;						//是否正在手抛起飞
	xSemaphoreHandle	iic_sem;						//IIC信号量
	xSemaphoreHandle	spi_sem;						//SPI信号量
}flag_t;

struct d_int16
{
	int16_t x;
	int16_t y;
	int16_t z;
};
struct d_float
{
	float x;
	float y;
	float z;
};

//飞行器的一些状态
struct __vel{
	float acc_z;//无重力分量的Z加速度
	float base_acc_z;//校准值
	float last_VelocityZ;//上次速度
	float VelocityZ;//Z轴速度
	float EstimatedZ;//Z轴估计高度
};
typedef struct
{
	struct __vel velocityZ;//Z轴速度计算用
}self_t;

extern volatile flag_t flag;			//全局标志
extern self_t gSelfState;					//飞行器状态
#endif
