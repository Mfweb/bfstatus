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
#include "BF_Status_Mini_Param.h"

uint8_t mpu_init(void);
void MPU6050_Read(void);
//MPU数据
sensor_data_mpu_t gSensorMpu={
.Init = &mpu_init,
.Read = &MPU6050_Read
};

/**
 * @brief  MPU6050初始化
 * @code
 *      mpu_init();
 * @endcode
 * @param  void
 * @retval 初始化状态
 *         @arg TRUE :初始化完成
 *         @arg FALSE :初始化失败（一般为Who ai I读取失败）
 */
uint8_t mpu_init(void)
{
	uint8_t ack;
	Single_Read(MPU6050_ADDRESS, WHO_AM_I,&ack);
	if(ack!=0x68)return FALSE;
	Single_Write(MPU6050_ADDRESS,PWR_MGMT_1,0x03);										//解除休眠状态
	Single_Write(MPU6050_ADDRESS,SMPLRT_DIV,0x01);										//采样率 500hz
	Single_Write(MPU6050_ADDRESS,CONFIGL, MPU6050_DLPF);							//低通滤波
	Single_Write(MPU6050_ADDRESS,GYRO_CONFIG,MPU6050_GYRO_FS_2000);		//陀螺仪量程 +-2000
	Single_Write(MPU6050_ADDRESS,ACCEL_CONFIG,MPU6050_ACCEL_FS_8);		//加速度量程 +-8G
	Single_Write(MPU6050_ADDRESS,0x37,0x02);													//设置为Pass-Through Mode
	return TRUE;
}
/**
* @brief  MPU6050读取一次数据
 * @code
 *      MPU6050_Read();
 * @endcode
 * @param  void
 * @retval void
 */
void MPU6050_Read(void)
{
	uint8_t mpu6050_buffer[14];
	
	I2C_Read(MPU6050_ADDRESS,0x3B,14,mpu6050_buffer);
	
	gSensorMpu.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
	gSensorMpu.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
	gSensorMpu.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);
	
	gSensorMpu.gyr.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	gSensorMpu.gyr.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	gSensorMpu.gyr.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
	
	
	
	gSensorMpu.acc.averag.x = gSensorMpu.acc.origin.x - gSensorMpu.acc.quiet.x;
	gSensorMpu.acc.averag.y = gSensorMpu.acc.origin.y - gSensorMpu.acc.quiet.y;
	gSensorMpu.acc.averag.z = gSensorMpu.acc.origin.z;
	
	gSensorMpu.gyr.averag.x = gSensorMpu.gyr.origin.x - gSensorMpu.gyr.quiet.x;
	gSensorMpu.gyr.averag.y = gSensorMpu.gyr.origin.y - gSensorMpu.gyr.quiet.y;
	gSensorMpu.gyr.averag.z = gSensorMpu.gyr.origin.z - gSensorMpu.gyr.quiet.z;
	
	//0G状态判定
	if(math_abs(gSensorMpu.acc.origin.x) < ZERO_MAX && \
		math_abs(gSensorMpu.acc.origin.y) < ZERO_MAX && \
	math_abs(gSensorMpu.acc.origin.z) < ZERO_MAX)
		flag.ZeroGravity = true;
	else
		flag.ZeroGravity=false;
	
	if(flag.CalibratingACC)				//需要标定加速度计
	{
		flag.Lock = true;							//标定时必须锁定
		if(flag.CalibratingACC==200)
		{
			gSensorMpu.temp[0] = 0;
			gSensorMpu.temp[1] = 0;
			gSensorMpu.temp[2] = 0;
		}
		flag.CalibratingACC--;
		gSensorMpu.temp[0] += gSensorMpu.acc.origin.x;
		gSensorMpu.temp[1] += gSensorMpu.acc.origin.y;
		gSensorMpu.temp[2] += gSensorMpu.acc.origin.z;
		if(!flag.CalibratingACC)
		{
			gSensorMpu.acc.quiet.x = gSensorMpu.temp[0]/200;//取200次平均值
			gSensorMpu.acc.quiet.y = gSensorMpu.temp[1]/200;
			gSensorMpu.acc.quiet.z = gSensorMpu.temp[2]/200;
			gParam.Save();//保存数据
			flag.CalibratingVel = 100;//开始标定Z轴速度静止值
		}
	}
	if(flag.CalibratingGYR)			//需要标定陀螺仪
	{
		flag.Lock = true;
		if(flag.CalibratingGYR==200)
		{
			gSensorMpu.temp[2] = 0;
			gSensorMpu.temp[3] = 0;
			gSensorMpu.temp[4] = 0;
		}
		flag.CalibratingGYR--;
		gSensorMpu.temp[2] += gSensorMpu.gyr.origin.x;
		gSensorMpu.temp[3] += gSensorMpu.gyr.origin.y;
		gSensorMpu.temp[4] += gSensorMpu.gyr.origin.z;
		if(!flag.CalibratingGYR)
		{
			gSensorMpu.gyr.quiet.x = gSensorMpu.temp[2]/200;//取200次平均值
			gSensorMpu.gyr.quiet.y = gSensorMpu.temp[3]/200;
			gSensorMpu.gyr.quiet.z = gSensorMpu.temp[4]/200;
			gParam.Save();//保存数据
		}
	}
}
