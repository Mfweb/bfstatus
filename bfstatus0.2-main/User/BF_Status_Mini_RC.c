/**
  ******************************************************************************
  * @file    BF_Status_Mini_RC.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini ���߽���
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
		/*ң������*/
		RC_Data.Pitch =	(((uint16_t)GetData[0])<<8)|GetData[1];
		RC_Data.Roll =	(((uint16_t)GetData[2])<<8)|GetData[3];
		RC_Data.Yaw =	(((uint16_t)GetData[4])<<8)|GetData[5];
		if(!flag.hold_altitude)//���ߵ�ʱ�򲻻�ȡң������ֵ
		{
			if(flag.zero_pull == 2)//�����������ģʽ��
			{
				if(((int16_t)((((uint16_t)GetData[6])<<8)|GetData[7]) - 1000) > RC_Data.Throttle)
					flag.zero_pull = 0;//�˳�����ģʽ
			}
			else
			{
				RC_Data.Throttle = (int16_t)((((uint16_t)GetData[6])<<8)|GetData[7]) - 1000;
			}
		}
		//�����޷�
		if(RC_Data.Throttle<0)RC_Data.Throttle=0;
		if(RC_Data.Throttle>TH_MAX_CHECK)RC_Data.Throttle=TH_MAX_CHECK;
		/*��Ҫ��ң�ط�������*/
		if(GetData[8] & 0x08)flag.need_back = 1;												//��Ҫ��������
		/*У׼������ʱ���봦������״̬*/
		if(GetData[8] & 0x01 && flag.Lock)flag.CalibratingACC = 200;	//У׼���ٶȼ�
		if(GetData[8] & 0x02 && flag.Lock)flag.CalibratingGYR = 200;	//У׼������
		if(GetData[8] & 0x10 && flag.Lock)flag.CalibratingMAG = 1;		//У׼����
		//if(GetData[8] & 0x40 && !flag.Lock)flag.flip = 1;						//����
		/*�����ж�*/
		if(GetData[8] & 0x04 && \
			Angle.angle.Pitch < 10.0f && Angle.angle.Pitch > -10.0f && \
			Angle.angle.Roll < 10.0f && Angle.angle.Roll > -10.0f && \
			RC_Data.Throttle < TH_MIN_CHECK && \
			(sensor_ms.get_count == 0 || !flag.IsBaro) && \
			flag.battery_alarm == 0)flag.Lock=0; //��б�������ų���������š���ѹ��û�б����ɡ���ѹ�������������
		
		/* ��������  �����ڽ���״̬��  �����Ѿ��궨�������ѹֵ*/
		if(GetData[8] & 0x20 && !flag.Lock && sensor_ms.data_baro_start!=0)
		{
			flag.hold_altitude = !flag.hold_altitude;
			if(flag.hold_altitude) //�������������  ���ȡ��ǰ�ĸ߶�
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
		if(Lost_count>=LOST_CTRL_TIME)//����  ����
		{
			flag.Lock = 1;
		}
		else
			Lost_count++;
	}
}

/* nRF ���͵�ǰ״̬ */
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

