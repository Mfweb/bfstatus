#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "stm32f10x_i2c.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "BF_Status_Mini_Global.h"
#include "BF_Status_Mini_Math.h"

#define	SMPLRT_DIV			0x19	//陀螺仪采样率
#define	CONFIGL					0x1A	//低通滤波频率
#define	GYRO_CONFIG			0x1B	//陀螺仪自检及测量范围
#define	ACCEL_CONFIG		0x1C	//加速计自检、测量范围及高通滤波频率
#define	ACCEL_XOUT_H		0x3B
#define	ACCEL_XOUT_L		0x3C
#define	ACCEL_YOUT_H		0x3D
#define	ACCEL_YOUT_L		0x3E
#define	ACCEL_ZOUT_H		0x3F
#define	ACCEL_ZOUT_L		0x40
#define	TEMP_OUT_H			0x41
#define	TEMP_OUT_L			0x42
#define	GYRO_XOUT_H			0x43
#define	GYRO_XOUT_L			0x44	
#define	GYRO_YOUT_H			0x45
#define	GYRO_YOUT_L			0x46
#define	GYRO_ZOUT_H			0x47
#define	GYRO_ZOUT_L			0x48
#define	PWR_MGMT_1			0x6B
#define	WHO_AM_I				0x75
#define MPU6050_ADDRESS 0xD0  //IIC地址


#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

//MPU6050内部低通滤波设置

//#define MPU6050_DLPF  MPU6050_DLPF_BW_256	//256Hz低通滤波
//#define MPU6050_DLPF  MPU6050_DLPF_BW_188	//188Hz低通滤波
//#define MPU6050_DLPF  MPU6050_DLPF_BW_98	//98Hz低通滤波      
//#define MPU6050_DLPF  MPU6050_DLPF_BW_42	//42Hz低通滤波
#define MPU6050_DLPF  MPU6050_DLPF_BW_20	//20Hz低通滤波
//#define MPU6050_DLPF  MPU6050_DLPF_BW_10	//10Hz低通滤波 	     
//#define MPU6050_DLPF  MPU6050_DLPF_BW_5	//5Hz低通滤波

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x08
#define MPU6050_GYRO_FS_1000        0x10
#define MPU6050_GYRO_FS_2000        0x18

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x08
#define MPU6050_ACCEL_FS_8          0x10
#define MPU6050_ACCEL_FS_16         0x18


#define G_PER_LSB_2				(float)((2*2)/65536.f)
#define G_PER_LSB_4				(float)((2*4)/65536.f)
#define G_PER_LSB_8				(float)((2*8)/65536.f)
#define G_PER_LSB_16			(float)((2*16)/65536.f)

#define DEG_PER_LSB_250		(float)((2*250)/65536.f)
#define DEG_PER_LSB_500		(float)((2*500)/65536.f)
#define DEG_PER_LSB_1000	(float)((2*1000)/65536.f)
#define DEG_PER_LSB_2000	(float)((2*2000)/65536.f)

#define Gyro_Gr (AtR*DEG_PER_LSB_2000)

typedef struct
{
	struct d_int16 origin;//原始值
	struct d_float averag;//平均值
	struct d_int16 quiet; //静态值
	struct d_float angle;//角度/角速度
}_sensor_data;

/*  MPU6050  */
typedef struct
{
	uint8_t (*Init)(void);
	void (*Read)(void);
	_sensor_data acc;
	_sensor_data gyr;
	int temp[5];
}sensor_data_mpu_t;

extern sensor_data_mpu_t gSensorMpu;	//MPU数据

#endif
