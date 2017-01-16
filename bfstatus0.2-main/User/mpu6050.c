/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    MPU6050（模拟I2C） BiFang Status Mini
  ******************************************************************************
  */
#include "mpu6050.h"
#include "led.h"
#include "BF_Status_Mini_System.h"

uint8_t mpu_init(void)
{
	uint8_t ack;
	ack = Single_Read(MPU6050_ADDRESS, WHO_AM_I);
	if(!ack)return FALSE;
	Single_Write(MPU6050_ADDRESS,PWR_MGMT_1,0x00);										//解除休眠状态
	Single_Write(MPU6050_ADDRESS,SMPLRT_DIV,0x07);
	Single_Write(MPU6050_ADDRESS,CONFIGL, MPU6050_DLPF);							//低通滤波
	Single_Write(MPU6050_ADDRESS,GYRO_CONFIG,MPU6050_GYRO_FS_1000);		//陀螺仪量程 +-1000
	Single_Write(MPU6050_ADDRESS,ACCEL_CONFIG,MPU6050_ACCEL_FS_4);		//加速度量程 +-4G
	Single_Write(MPU6050_ADDRESS,0x37,0x02);													//设置为Pass-Through Mode模式  用来直接连接HMC5883L
	return TRUE;
}

void MPU6050_Read(void)
{
	int16_t temp[3];
	uint8_t mpu6050_buffer[14];
	
	I2C_Read(MPU6050_ADDRESS,0x3B,14,mpu6050_buffer);
	temp[0] = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
	temp[1] = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
	temp[2] = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	//0G状态判定
	if(math_abs(temp[0]) < ZERO_MAX && math_abs(temp[1]) < ZERO_MAX && math_abs(temp[2]) < ZERO_MAX)
		flag.zerog = 1;
	else
		flag.zerog = 0;
	
	sensor_mpu.acc.origin.x = temp[0] - sensor_mpu.acc.quiet.x;
	sensor_mpu.acc.origin.y = temp[1] - sensor_mpu.acc.quiet.y;
	sensor_mpu.acc.origin.z = temp[2];

	sensor_mpu.gyr.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor_mpu.gyr.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor_mpu.gyr.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
	
	sensor_mpu.gyr.radian.x=sensor_mpu.gyr.origin.x-sensor_mpu.gyr.quiet.x;
	sensor_mpu.gyr.radian.y=sensor_mpu.gyr.origin.y-sensor_mpu.gyr.quiet.y;
	sensor_mpu.gyr.radian.z=sensor_mpu.gyr.origin.z-sensor_mpu.gyr.quiet.z;

	
	if(flag.CalibratingACC)				//需要标定加速度计
	{
		flag.Lock = 1;							//标定时必须锁定
		if(flag.CalibratingACC==200)
		{
			sensor_mpu.temp[0] = 0;
			sensor_mpu.temp[1] = 0;
		}
		flag.CalibratingACC--;
		sensor_mpu.temp[0] += temp[0];
		sensor_mpu.temp[1] += temp[1];
		if(!flag.CalibratingACC)
		{
			sensor_mpu.acc.quiet.x = sensor_mpu.temp[0]/200;//取200次平均值
			sensor_mpu.acc.quiet.y = sensor_mpu.temp[1]/200;
			data_save();//保存数据
		}
	}
	if(flag.CalibratingGYR)			//需要标定陀螺仪
	{
		flag.Lock = 1;
		if(flag.CalibratingGYR==200)
		{
			sensor_mpu.temp[2] = 0;
			sensor_mpu.temp[3] = 0;
			sensor_mpu.temp[4] = 0;
		}
		flag.CalibratingGYR--;
		sensor_mpu.temp[2] += sensor_mpu.gyr.origin.x;
		sensor_mpu.temp[3] += sensor_mpu.gyr.origin.y;
		sensor_mpu.temp[4] += sensor_mpu.gyr.origin.z;
		if(!flag.CalibratingGYR)
		{
			sensor_mpu.gyr.quiet.x = sensor_mpu.temp[2]/200;//取200次平均值
			sensor_mpu.gyr.quiet.y = sensor_mpu.temp[3]/200;
			sensor_mpu.gyr.quiet.z = sensor_mpu.temp[4]/200;
			data_save();//保存数据
		}
	}
}
