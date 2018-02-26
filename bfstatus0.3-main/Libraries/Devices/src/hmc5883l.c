/**
  ******************************************************************************
  * @file    hmc5883l.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini hmc5883l 电子罗盘 底层驱动(受电机干扰，不建议开启)
	*						对于V0.1的硬件，需要改变罗盘方向才可以使用！切记
  ******************************************************************************
  */
#include "hmc5883l.h"
#include "BF_Status_Mini_Global.h"
#include "BF_Status_Mini_Math.h"
#include "BF_Status_Mini_System.h"
#include "math.h"
#include "stm32fx_delay.h"
//罗盘数据
_sensor_data_mag sensor_mag;
uint8_t hmc_init(void)
{
	uint8_t ack; 
	
	ack = Single_Read(MAG_ADDRESS,10);
	
	if (!ack)
			return FALSE;

	// leave test mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFA, 0x70);   // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	Single_Write(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
	DelayMs(20);

	return TRUE;	 
}
	

void HMC5883L_Read(void)
{
	static uint8_t check_mag=1;
	uint8_t buff[6],i,count=0;
	int16_t temp_data[3];
	static int32_t An[3] = {0,0,0};
	
	// 读取寄存器数据
	I2C_Read(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buff);
	
	// 十位深度滤波
	An[0] -= An[0]/10;//Z轴
	An[0] += (int16_t)(buff[2] << 8 | buff[3]);
	temp_data[0] = An[0]/10;
	
	An[1] -= An[1]/10;//X轴
	An[1] += (int16_t)(buff[0] << 8 | buff[1]);
	temp_data[1] = An[1]/10;
	
	An[2] -= An[2]/10;//Y轴
	An[2] += (int16_t)(buff[4] << 8 | buff[5]);
	temp_data[2] = An[2]/10;

	//需要校准
	if(flag.CalibratingMAG)
	{
		check_mag=1;
		flag.IsMAG = 1;
		Mag_Calibration(temp_data);
	}
	//自检
	if(check_mag)
	{
		check_mag=0;
		for(i=0;i<6;i++)
		{
			if(math_abs(sensor_mag.mag_limt[i])<20)
				count++;
		}
		if(count>=2)
			flag.IsMAG = 0;
  }
	
	//修正
	sensor_mag.origin.z = (float)(temp_data[0] -(sensor_mag.mag_limt[3] + sensor_mag.mag_limt[0])/2);
	sensor_mag.origin.x = (float)(temp_data[1] -(sensor_mag.mag_limt[4] + sensor_mag.mag_limt[1])/2);
	sensor_mag.origin.y = (float)(temp_data[2] -(sensor_mag.mag_limt[5] + sensor_mag.mag_limt[2])/2);
}

void Mag_Calibration(int16_t *array)
{
	uint8_t i;
	static uint8_t clen_flag=1; 
	static float x,y,z; 
	
	//校准之前先把之前数据清零
	if(clen_flag)
	{
		clen_flag = 0;
		x=y=z=0;
		for(i=0;i<6;i++)
			sensor_mag.mag_limt[i] = 0;
	}
  
	//找出三个轴最大值和最小值
	for(i=0;i<3;i++)
	{
		if(sensor_mag.mag_limt[i] > array[i])
			sensor_mag.mag_limt[i] = array[i];
		else if(sensor_mag.mag_limt[i+3] < array[i])
			sensor_mag.mag_limt[i+3] = array[i];
	}
	
	//利用加速度计和陀螺仪确定已经在三个轴上各旋转了360°
	if(flag.CalibratingMAG == 1 && (fabs(sensor_mpu.acc.averag.z) > 5000))//标记为1并且Z轴加速度计大于5000 认为正在对Z轴进行校正
	{
	  z += sensor_mpu.gyr.radian.z * Gyro_G * 0.002f;//对Z轴陀螺仪积分
		if(fabs(z)>360)
			flag.CalibratingMAG = 2;//如果大于360° 表示已经旋转完成  进入下一个轴的校准中
	}
	
	if(flag.CalibratingMAG == 2 && (fabs(sensor_mpu.acc.averag.x) > 5000))
	{
	  x += sensor_mpu.gyr.radian.x * Gyro_G * 0.002f;
		if(fabs(x)>360)
			flag.CalibratingMAG = 3;
	}
	
	if(flag.CalibratingMAG == 3 && (fabs(sensor_mpu.acc.averag.y) > 5000))
	{
	  y += sensor_mpu.gyr.radian.y * Gyro_G * 0.002f;
		if(fabs(y)>360)//校准完成
		{
			clen_flag = 1;
			flag.CalibratingMAG = 0;
			flag.IsMAG = 1;//启用磁阻
			data_save();//保存参数
		}
	}	
}
