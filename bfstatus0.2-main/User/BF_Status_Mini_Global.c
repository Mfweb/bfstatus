/**
  ******************************************************************************
  * @file    BF_Status_Mini_Global.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini ��������
  ******************************************************************************
  */
#include "BF_Status_Mini_Global.h"
//Flag
type_flag flag;
//ʱ��Ƭ
uint8_t TimeKatawa[3]={0};
//MPU����
_sensor_data_mpu sensor_mpu;
//��������
_sensor_data_mag sensor_mag;
//��ѹ������
_ms_data sensor_ms;
//���ܿ���
_ctrl ctrl;
//Ŀ����
_target Target; 
//ң������
_rc_getdata RC_Data;
//ŷ����
EulerAngle Angle = {0};
//��Ԫ��
Quaternion NumQ = {1, 0, 0, 0};
//�������
bat__ battery;

