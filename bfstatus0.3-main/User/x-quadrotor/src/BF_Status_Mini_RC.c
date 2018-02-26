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
#include "bmp280.h"
#include <stdio.h>

//遥控数据
data_rc_t gRCData={0,0,0,0,0,{false,true,false,false}};

/**
 * @brief  从nRF24L01读取一次数据
 * @code
 *      RC_GetData();
 * @endcode
 * @param  void
 * @retval void
 */
uint8_t RC_GetData(void)
{
	if(gSensorNRF.RXData(gSensorNRF.RX_BUF)==RX_DR)
	{
		/*遥控数据*/
		gRCData.Pitch =	(((uint16_t)gSensorNRF.RX_BUF[0])<<8)|gSensorNRF.RX_BUF[1];
		gRCData.Roll =	(((uint16_t)gSensorNRF.RX_BUF[2])<<8)|gSensorNRF.RX_BUF[3];
		gRCData.Yaw =	(((uint16_t)gSensorNRF.RX_BUF[4])<<8)|gSensorNRF.RX_BUF[5];

		if(flag.Throwing == 2)//如果正在手抛模式中
		{
			if(((int16_t)((((uint16_t)gSensorNRF.RX_BUF[6])<<8)|gSensorNRF.RX_BUF[7]) - 1000) > gRCData.Throttle)
				flag.Throwing = 0;//退出手抛模式
		}
		else
		{
			gRCData.Throttle = (int16_t)((((uint16_t)gSensorNRF.RX_BUF[6])<<8)|gSensorNRF.RX_BUF[7]) - 1000;
		}

		//油门限幅
		gRCData.Throttle = math_limit(gRCData.Throttle,1000,0);
		/*校准传感器时必须处于锁定状态*/
		if(gSensorNRF.RX_BUF[8] & RC_CAL_ACC && flag.Lock)flag.CalibratingACC = 200;	//校准加速度计
		if(gSensorNRF.RX_BUF[8] & RC_CAL_GYR && flag.Lock)flag.CalibratingGYR = 200;	//校准陀螺仪
		if(gSensorNRF.RX_BUF[8] & RC_CLA_MAG && flag.Lock)flag.CalibratingMAG = 1;		//校准罗盘
		if(gSensorNRF.RX_BUF[8] & RC_FLIP		 && flag.Lock)flag.Flipping = 1;							//翻滚
//		if(gnRFData.RX_BUF[8] & RC_SEND_BACK)						flag.SendBack = true;
		if(gSensorNRF.RX_BUF[8] & RC_POWER_OFF)						{flag.PowerOff = true;flag.Lock = true;}				//关机
		/*解锁判定*/
//		if(GetData[8] & RC_UNLOCK && \
//			math_abs(Angle.angle.Pitch) < 10.0f && math_abs(Angle.angle.Roll) < 10.0f &&  RC_Data.Throttle < TH_MIN_CHECK && \
//			(sensor_ms.get_count == 0 || !flag.IsBaro) && flag.battery_alarm == false)
		if(gSensorNRF.RX_BUF[8] & RC_UNLOCK && \
			math_abs(gIMU.angle.Pitch) < 10.0f && math_abs(gIMU.angle.Roll) < 10.0f &&  gRCData.Throttle < TH_MIN_CHECK)
				flag.Lock=false; //倾斜过大、油门超过检查油门、气压计没有标记完成、电压过低则不允许解锁
		
		/* 启动定高  必须在解锁状态下*/
		
		if(gSensorNRF.RX_BUF[8] & RC_HOLD_ALT && flag.Lock == false)
		{
			if(flag.HoldAltitude == true)
			{
				flag.HoldAltitude = false;
				gTarget.Altitude = 0;
			}
			else//启动定高
			{
				gTarget.Altitude = gSelfState.velocityZ.EstimatedZ;
				gTarget.base_throttle_set = gCtrl.throttle_out;
				gTarget.base_throttle = gTarget.base_throttle_set;
				gCtrl.alt.increment = 0;
				gCtrl.alt.error = 0;
				gCtrl.alt.last_error = 0;
				flag.HoldAltitude = true;
			}
		}
		if(flag.Lock)//只有在锁定的时候才更新配置
		{
			if(gSensorNRF.RX_BUF[9] & RC_SET_HODE_ALT)
				gRCData.Setting.Hode_ALT_Mode = true;
			else
				gRCData.Setting.Hode_ALT_Mode = false;
			
			if(gSensorNRF.RX_BUF[9] & RC_SET_HODE_HEADER)
				gRCData.Setting.Hode_Header_Mode = true;
			else
				gRCData.Setting.Hode_Header_Mode = false;
			
			if(gSensorNRF.RX_BUF[9] & RC_SET_COLLISION)
				gRCData.Setting.Collision = true;
			else
				gRCData.Setting.Collision = false;
			
			if(gSensorNRF.RX_BUF[9] & RC_SET_HAND_TH)
				gRCData.Setting.Hand_Throwing = true;
			else
				gRCData.Setting.Hand_Throwing = false;
		}

		//printf("geted \r\n");
		//printf("%d %d %d %d\r\n",RC_Data.PITCH,RC_Data.ROLL,RC_Data.YAW,RC_Data.THROTTLE);
		gRCData.Lost_count = 0;
		return TRUE;
	}
	else
	{
		gRCData.Lost_count++;
		if(gRCData.Lost_count>=LOST_CTRL_TIME)//丢控  锁定
		{
			if(flag.Lock == true)//锁定状态下丢控  则关机
			{
				#ifdef LOST_POWER_OFF
				flag.PowerOff = true;
				#endif
			}
			else
			{
				flag.Lock = true;
			}
			gRCData.Lost_count = 0;
		}
		else
			gRCData.Lost_count++;
		return FALSE;
	}
}

/**
 * @brief  nRF24L01发送数据
 * @code
 *      RC_SendData();
 * @endcode
 * @param  void
 * @retval void
 */
void RC_SendData(void)
{
	gSensorNRF.TX_BUF[6] = 0x00;
	if(!flag.Lock)gSensorNRF.TX_BUF[6]|=0x01;
	if(flag.IsMAG)gSensorNRF.TX_BUF[6]|=0x02;
	if(flag.ZeroGravity)gSensorNRF.TX_BUF[6]|=0x04;
	gSensorNRF.TX_BUF[7] = (uint8_t)((uint16_t)(gBattery.now_value*100.0f));
	gSensorNRF.TX_BUF[8] = (uint8_t)(((uint16_t)(gBattery.now_value*100.0f))>>8);
	
	gSensorNRF.TX_BUF[9] = 0;
	if(gRCData.Setting.Hode_ALT_Mode == true)
		gSensorNRF.TX_BUF[9] |= RC_SET_HODE_ALT;
	if(gRCData.Setting.Hode_Header_Mode == true)
		gSensorNRF.TX_BUF[9] |= RC_SET_HODE_HEADER;
	if(gRCData.Setting.Collision == true)	
		gSensorNRF.TX_BUF[9] |= RC_SET_COLLISION;
	if(gRCData.Setting.Hand_Throwing == true)
		gSensorNRF.TX_BUF[9] |= RC_SET_HAND_TH;
	/*
	ch1 :0 1
	ch2 :2 3
	ch3 :4 5
	ch4 :10 11 12 13
	ch5 :14 15 16 17
	ch6 :18 19 20 21
	ch7 :22 23
	ch8 :24 25
	ch9 :26 27
	ch10:28 29
	*/
	int16_t temp[3];
	temp[0] = gIMU.angle.Pitch*100;//ch1
	temp[1] = gIMU.angle.Roll*100;//ch2
	temp[2] = gIMU.angle.Yaw*100;//ch3
	gSensorNRF.TX_BUF[0] = (uint8_t)temp[0];
	gSensorNRF.TX_BUF[1] = (uint8_t)(temp[0]>>8);
	gSensorNRF.TX_BUF[2] = (uint8_t)temp[1];
	gSensorNRF.TX_BUF[3] = (uint8_t)(temp[1]>>8);
	gSensorNRF.TX_BUF[4] = (uint8_t)temp[2];
	gSensorNRF.TX_BUF[5] = (uint8_t)(temp[2]>>8);
	int32_t temp32[3];
	temp32[0] = gSelfState.velocityZ.EstimatedZ *100.f;		//ch4
	temp32[1] = gTarget.base_throttle;	//ch5
	temp32[2] = gMotor.base;			//ch6
	
	gSensorNRF.TX_BUF[10] = (uint8_t)temp32[0];
	gSensorNRF.TX_BUF[11] = (uint8_t)(temp32[0]>>8);
	gSensorNRF.TX_BUF[12] = (uint8_t)(temp32[0]>>16);
	gSensorNRF.TX_BUF[13] = (uint8_t)(temp32[0]>>24);
	
	gSensorNRF.TX_BUF[14] = (uint8_t)temp32[1];
	gSensorNRF.TX_BUF[15] = (uint8_t)(temp32[1]>>8);
	gSensorNRF.TX_BUF[16] = (uint8_t)(temp32[1]>>16);
	gSensorNRF.TX_BUF[17] = (uint8_t)(temp32[1]>>24);
	
	gSensorNRF.TX_BUF[18] = (uint8_t)temp32[2];
	gSensorNRF.TX_BUF[19] = (uint8_t)(temp32[2]>>8);
	gSensorNRF.TX_BUF[20] = (uint8_t)(temp32[2]>>16);
	gSensorNRF.TX_BUF[21] = (uint8_t)(temp32[2]>>24);
	
	temp[0] = gBattery.now_value*100.f;						//ch7
	temp[1] = gCtrl.alt.pid_out;//ch8
	gSensorNRF.TX_BUF[22] = (uint8_t)temp[0];
	gSensorNRF.TX_BUF[23] = (uint8_t)(temp[0]>>8);
	gSensorNRF.TX_BUF[24] = (uint8_t)temp[1];
	gSensorNRF.TX_BUF[25] = (uint8_t)(temp[1]>>8);
	
	temp[0] = gTarget.Altitude*100;						//ch9
	gSensorNRF.TX_BUF[26] = (uint8_t)temp[0];
	gSensorNRF.TX_BUF[27] = (uint8_t)(temp[0]>>8);

	
	gSensorNRF.TX_BUF[TX_PLOAD_WIDTH-1] = 0x55;//用来标识是飞行器发出的
	gSensorNRF.TXData(gSensorNRF.TX_BUF);
}

