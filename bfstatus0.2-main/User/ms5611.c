/**
  ******************************************************************************
  * @file    ms5611.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini ms5611��ѹ��
  ******************************************************************************
  */
#include "ms5611.h"
#include <math.h>
#include "stm32fx_delay.h"
#include <stdio.h>
#define MS_HW_ADDR 0xEE
//��λ��ѹ��
void ms5611_reset(void)
{
	I2C_Start();
	I2C_SendByte(MS_HW_ADDR);
	I2C_WaitAck();
	I2C_SendByte(0x1E);			//��λ
	I2C_WaitAck();
	I2C_Stop();
	sensor_ms.state = 0;
	sensor_ms.time_count = 0;
	sensor_ms.temp_index = 0;
	sensor_ms.baro_index = 0;
	sensor_ms.temp_index = 0;
	sensor_ms.baro_index = 0;
	
	DelayMs(500);
	ms5611_read_fac();
}
//��ȡ�����궨ֵ
void ms5611_read_fac(void)
{
	uint8_t temp[2],i;
	uint16_t t2[6];
	for(i=0;i<6;i++)
	{
		I2C_Read(MS_HW_ADDR,0xA2+i*2,2,temp);
		t2[i] = ((uint16_t)temp[0]<<8)|(uint16_t)temp[1];
	}
	//I2C_Read(MS_HW_ADDR,0xA0,12,temp);
	sensor_ms.data_cal.sens 		= t2[0];
	sensor_ms.data_cal.off 			= t2[1];
	sensor_ms.data_cal.tcs 			= t2[2];
	sensor_ms.data_cal.tco 			= t2[3];
	sensor_ms.data_cal.tref 		= t2[4];
	sensor_ms.data_cal.tempsens = t2[5];
	
	//printf("%u %u %u %u %u %u \r\n",sensor_ms.data_cal.sens,sensor_ms.data_cal.off,sensor_ms.data_cal.tcs,sensor_ms.data_cal.tco,sensor_ms.data_cal.tref,sensor_ms.data_cal.tempsens);
}
//��ȡadcֵ
int32_t ms5611_read_adc(void)
{
	int32_t t_data = 0x00;
	uint8_t temp[3];
	I2C_Read(MS_HW_ADDR,0x00,3,temp);
	t_data = (int32_t)temp[0]<<16 | (int32_t)temp[1]<<8 | (int32_t)temp[2];
	return t_data;
}
//��ʼת��
void ms5611_start_conversion(uint8_t mode)
{
	I2C_Start();
	I2C_SendByte(MS_HW_ADDR);
	I2C_WaitAck();
	if(mode==0)
		I2C_SendByte(0x50 + 0x08);//��ʼ�¶�ת����ת���������(9.1ms)
	else if(mode ==1)
		I2C_SendByte(0x40 + 0x08);//��ʼ��ѹת����ת���������(9.1ms)
	I2C_WaitAck();	
	I2C_Stop();
}
//��ѹת���ɸ߶�
float get_altitude(void)
{
	static float Altitude;
	if(sensor_ms.data_baro_start == 0)return 0.0f;
	//��λm ��Ը߶�
	Altitude = 4433000.0f*(1.0f-pow(sensor_ms.data_baro_now/sensor_ms.data_baro_start,0.1903f))*0.01f;
	return Altitude; 
}

//��ȡת�������ѹ
void ms5611_read_baro(void)
{
	int64_t off,sens;
	int32_t temp_baro = 0x00;
	int64_t dT = 0,TEMP,T2,Aux_64,OFF2,SENS2;
	float temp_sum = 0.0f;
	uint8_t i;
	
	temp_baro = ms5611_read_adc();
	//printf("b:%d \r\n t:%d\r\n",temp_baro,sensor_ms.temp_org_det);

	dT = sensor_ms.temp_org_det - (((int32_t)sensor_ms.data_cal.tref)<< 8);
	
	sensor_ms.temp_data_buff[sensor_ms.temp_index++] = 2000 + (dT * (int64_t)sensor_ms.data_cal.tempsens)/8388608;
	if(sensor_ms.temp_index>=5)sensor_ms.temp_index=0;
	for(i=0;i<5;i++)
		temp_sum += sensor_ms.temp_data_buff[i];
	TEMP = temp_sum/5.0f;
	

	
	off  = (((int64_t)sensor_ms.data_cal.off ) << 16) + ((((int64_t)sensor_ms.data_cal.tco) * dT) >> 7);
	sens = (((int64_t)sensor_ms.data_cal.sens) << 15) + (((int64_t)(sensor_ms.data_cal.tcs) * dT) >> 8);
	
	if (TEMP < 2000)//�����¶Ȳ���
	{
		T2 			= (((int64_t)dT) * dT) >> 31;
		Aux_64 	= (TEMP - 2000) * (TEMP - 2000);
		OFF2 		= (5 * Aux_64) >> 1;
		SENS2 	= (5 * Aux_64) >> 2;
		TEMP 		= TEMP - T2;
		off 		= off  - OFF2;
		sens 		= sens - SENS2;
	}
	
	sensor_ms.baro_data_buff[sensor_ms.baro_index++] = (((((int64_t)temp_baro) * sens) >> 21) - off) / 32768.0f;
	if(sensor_ms.baro_index>=5)sensor_ms.baro_index=0;
	temp_sum = 0.0f;
	for(i=0;i<5;i++)
		temp_sum += sensor_ms.baro_data_buff[i];
	
	sensor_ms.data_baro_now = temp_sum/5;
	sensor_ms.data_temp_now = TEMP * 0.01f;
	
	sensor_ms.data_altitude_abs = get_altitude();
}
//��ȡת������¶�
void ms5611_read_temp(void)
{
	sensor_ms.temp_org_det = ms5611_read_adc();
}
//��ѹ��״̬����
void ms5611_handle(void)
{
	switch(sensor_ms.state)
	{
		case 0:
			ms5611_start_conversion(0);//״̬Ϊ0  �����¶�ת��
			sensor_ms.state = 1;
			break;
		case 1:
			if(sensor_ms.time_count>=5)//10MS�Ѿ�����  ��ȡ�¶�ת��ֵ  ����ʼ��ѹת��
			{
				ms5611_read_temp();
				sensor_ms.time_count = 0;
				ms5611_start_conversion(1);
				sensor_ms.state = 2;
			}
			else
				sensor_ms.time_count ++;
			break;
		case 2:
			if(sensor_ms.time_count>=5)//10MS����  ��ȡ��ѹֵ ��ʼ�¶�ת��
			{
				ms5611_read_baro();
				sensor_ms.time_count = 0;
				ms5611_start_conversion(0);
				sensor_ms.state = 1;
				if(sensor_ms.get_count)sensor_ms.get_count--;
			}
			else
				sensor_ms.time_count ++;
			break;
	}
}
