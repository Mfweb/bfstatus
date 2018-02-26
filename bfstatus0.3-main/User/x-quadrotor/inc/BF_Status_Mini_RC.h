#ifndef __BF_STATUS_MINI_RC_H__
#define __BF_STATUS_MINI_RC_H__
#include "stm32f10x.h"
#include "nrf24l01.h"
#include "BF_Status_Mini_Global.h"
#define RC_CAL_ACC		(1<<0)
#define RC_CAL_GYR		(1<<1)
#define RC_UNLOCK			(1<<2)
#define RC_SEND_BACK	(1<<3)
#define RC_CLA_MAG		(1<<4)
#define RC_HOLD_ALT		(1<<5)
#define RC_FLIP				(1<<6)
#define RC_POWER_OFF	(1<<7)


#define RC_SET_HODE_ALT			(1<<0)
#define RC_SET_HODE_HEADER	(1<<1)
#define RC_SET_COLLISION		(1<<2)
#define RC_SET_HAND_TH			(1<<3)


typedef struct
{
	_Bool Hode_ALT_Mode;		//定高模式
	_Bool Hode_Header_Mode;//无头模式
	_Bool Collision;//碰撞检测
	_Bool Hand_Throwing;//手抛起飞模式
}_rc_setting;
/* 遥控 */
typedef struct 
{
	int16_t Roll;
	int16_t Pitch;
	int16_t Throttle;//油门
	int16_t Yaw;
	
	uint16_t Lost_count;//丢控计数
	_rc_setting Setting;
}data_rc_t;

extern data_rc_t gRCData;					//遥控数据

uint8_t RC_GetData(void);
void RC_SendData(void);
#endif
