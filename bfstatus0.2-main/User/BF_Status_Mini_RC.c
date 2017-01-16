/**
  ******************************************************************************
  * @file    BF_Status_Mini_RC.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 无线接收
  ******************************************************************************
  */
#include "BF_Status_Mini_RC.h"
#include "BF_Status_Mini_Ctrl.h"
#include "led.h"
#include <stdio.h>
uint8_t GetData[32];
uint16_t Lost_count;
void RC_GetData(void)
{
	if(flag.need_back)return;
	if(NRF_Rx_Dat(GetData)==RX_DR)
	{
		/*遥控数据*/
		RC_Data.Pitch =	(((uint16_t)GetData[0])<<8)|GetData[1];
		RC_Data.Roll =	(((uint16_t)GetData[2])<<8)|GetData[3];
		RC_Data.Yaw =	(((uint16_t)GetData[4])<<8)|GetData[5];
		if(!flag.hold_altitude)//定高的时候不获取遥控油门值
		{
			if(flag.zero_pull == 2)//如果正在手抛模式中
			{
				if(((int16_t)((((uint16_t)GetData[6])<<8)|GetData[7]) - 1000) > RC_Data.Throttle)
					flag.zero_pull = 0;//退出手抛模式
			}
			else
			{
				RC_Data.Throttle = (int16_t)((((uint16_t)GetData[6])<<8)|GetData[7]) - 1000;
			}
		}
		//油门限幅
		if(RC_Data.Throttle<0)RC_Data.Throttle=0;
		if(RC_Data.Throttle>TH_MAX_CHECK)RC_Data.Throttle=TH_MAX_CHECK;
		/*需要向遥控返回数据*/
		if(GetData[8] & 0x08)flag.need_back = 1;												//需要返回数据
		/*校准传感器时必须处于锁定状态*/
		if(GetData[8] & 0x01 && flag.Lock)flag.CalibratingACC = 200;	//校准加速度计
		if(GetData[8] & 0x02 && flag.Lock)flag.CalibratingGYR = 200;	//校准陀螺仪
		if(GetData[8] & 0x10 && flag.Lock)flag.CalibratingMAG = 1;		//校准罗盘
		//if(GetData[8] & 0x40 && !flag.Lock)flag.flip = 1;						//翻滚
		/*解锁判定*/
		if(GetData[8] & 0x04 && \
			Angle.angle.Pitch < 10.0f && Angle.angle.Pitch > -10.0f && \
			Angle.angle.Roll < 10.0f && Angle.angle.Roll > -10.0f && \
			RC_Data.Throttle < TH_MIN_CHECK && \
			(sensor_ms.get_count == 0 || !flag.IsBaro) && \
			flag.battery_alarm == 0)flag.Lock=0; //倾斜过大、油门超过检查油门、气压计没有标记完成、电压过低则不允许解锁
		
		/* 启动定高  必须在解锁状态下  并且已经标定过起飞气压值*/
		if(GetData[8] & 0x20 && !flag.Lock && sensor_ms.data_baro_start!=0)
		{
			flag.hold_altitude = !flag.hold_altitude;
			if(flag.hold_altitude) //如果是启动定高  则获取当前的高度
			{
				sensor_ms.hold_altitude = sensor_ms.data_altitude_abs;
				//sensor_ms.hold_altitude = sensor_ms.data_baro_now;
			}
		}
		//printf("geted \r\n");
		//printf("%d %d %d %d\r\n",RC_Data.PITCH,RC_Data.ROLL,RC_Data.YAW,RC_Data.THROTTLE);
		Lost_count = 0;
	}
	else
	{
		Lost_count++;
		if(Lost_count>=LOST_CTRL_TIME)//丢控  锁定
		{
			flag.Lock = 1;
		}
		else
			Lost_count++;
	}
}

/* nRF 发送当前状态 */
void RC_SendData(void)
{
	uint8_t TX_BUF[16];
	int16_t temp[3];
	
	temp[0] = Angle.angle.Pitch*100.0f;
	temp[1] = Angle.angle.Roll*100.0f;
	temp[2] = Angle.angle.Yaw*100;
	
	
	TX_BUF[0] = (uint8_t)temp[0];
	TX_BUF[1] = (uint8_t)(temp[0]>>8);
	TX_BUF[2] = (uint8_t)temp[1];
	TX_BUF[3] = (uint8_t)(temp[1]>>8);
	TX_BUF[4] = (uint8_t)temp[2];
	TX_BUF[5] = (uint8_t)(temp[2]>>8);
	TX_BUF[6] = 0x00;
	if(!flag.Lock)TX_BUF[6]|=0x01;
	if(flag.IsMAG)TX_BUF[6]|=0x02;
	if(flag.zerog)TX_BUF[6]|=0x04;
	TX_BUF[7] = (uint8_t)((uint16_t)(battery.now_value*100.0f));
	TX_BUF[8] = (uint8_t)(((uint16_t)(battery.now_value*100.0f))>>8);
	
	NRF_Tx_Dat(TX_BUF);
}

