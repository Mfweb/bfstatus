/**
  ******************************************************************************
  * @file    BF_Status_Mini_Global.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 公共变量
  ******************************************************************************
  */
#include "BF_Status_Mini_Global.h"
//Flag
type_flag flag;
//时间片
uint8_t TimeKatawa[3]={0};
//MPU数据
_sensor_data_mpu sensor_mpu;
//罗盘数据
_sensor_data_mag sensor_mag;
//气压计数据
_ms_data sensor_ms;
//功能控制
_ctrl ctrl;
//目标量
_target Target; 
//遥控数据
_rc_getdata RC_Data;
//欧拉角
EulerAngle Angle = {0};
//四元数
Quaternion NumQ = {1, 0, 0, 0};
//电池数据
bat__ battery;

