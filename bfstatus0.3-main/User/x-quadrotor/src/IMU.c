#include "IMU.h"
#include "DataScope_DP.h"
#include "bmp280.h"
#include "BF_Status_Mini_Param.h"
//#define KpDef 0.8f						//+-4G 加速度计
//#define KiDef 0.0005f					//+-1000  陀螺仪   
#define KpDef 0.4f						//+-8G量程
#define KiDef 0.001f					//+-2000量程
#define SampleRateHalf 0.001f //0.001
#define POS_DT 0.004f
void Angle_Get(void);
//欧拉角
imu_t gIMU = {
.Get    = &Angle_Get,
.angle  ={0,0,0},
.radian = {0,0,0}
};
//四元数
quaternion_t gNumQ = {1, 0, 0, 0};

/**
 * @brief  获取四元数
 * @code
 *      Quaternion_Get(&q);
 * @endcode
 * @param  *pNumQ :输出四元数结构体
 * @retval void
 */
void Quaternion_Get(quaternion_t *pNumQ)
{
  float ErrX, ErrY, ErrZ;
  float AccX, AccY, AccZ;
  float GyrX, GyrY, GyrZ;
	float Normalize;
  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
	gravity_t V;
	
	if(gSensorMpu.acc.angle.x != 0 || gSensorMpu.acc.angle.y != 0 || gSensorMpu.acc.angle.z != 0)
	{
		//加速度归一化
		Normalize = Q_rsqrt(math_square(gSensorMpu.acc.angle.x) + math_square(gSensorMpu.acc.angle.y) + math_square(gSensorMpu.acc.angle.z));
		AccX = gSensorMpu.acc.angle.x*Normalize;
		AccY = gSensorMpu.acc.angle.y*Normalize;
		AccZ = gSensorMpu.acc.angle.z*Normalize;
		//提取重力分量
		V.x = 2     *(pNumQ->q1 * pNumQ->q3 - pNumQ->q0 * pNumQ->q2);								
		V.y = 2     *(pNumQ->q0 * pNumQ->q1 + pNumQ->q2 * pNumQ->q3);						  
		V.z = 1 - 2 *(pNumQ->q1 * pNumQ->q1 + pNumQ->q2 * pNumQ->q2);
		//向量差乘
		ErrX = (AccY*V.z - AccZ*V.y);
		ErrY = (AccZ*V.x - AccX*V.z);
		ErrZ = (AccX*V.y - AccY*V.x);
		//累计误差
		exInt += ErrX * KiDef * SampleRateHalf * 2;
		eyInt += ErrY * KiDef * SampleRateHalf * 2;
		ezInt += ErrZ * KiDef * SampleRateHalf * 2;
		GyrX = math_rad(gSensorMpu.gyr.angle.x) + KpDef * ErrX  +  exInt;
		GyrY = math_rad(gSensorMpu.gyr.angle.y) + KpDef * ErrY  +  eyInt;
		GyrZ = math_rad(gSensorMpu.gyr.angle.z) + KpDef * ErrZ  +  ezInt;	
	}
	GyrX *= SampleRateHalf;
	GyrY *= SampleRateHalf;
	GyrZ *= SampleRateHalf;
	//四元数更新
	quaternion_t tQ = *pNumQ;
  pNumQ->q0 += (-tQ.q1*GyrX - tQ.q2*GyrY - tQ.q3*GyrZ);
  pNumQ->q1 += ( tQ.q0*GyrX - tQ.q3*GyrY + tQ.q2*GyrZ);
  pNumQ->q2 += ( tQ.q3*GyrX + tQ.q0*GyrY - tQ.q1*GyrZ);
  pNumQ->q3 += (-tQ.q2*GyrX + tQ.q1*GyrY + tQ.q0*GyrZ);
	//四元数归一化
	Normalize = Q_rsqrt(math_square(pNumQ->q0) + math_square(pNumQ->q1) + math_square(pNumQ->q2) + math_square(pNumQ->q3));
  pNumQ->q0 *= Normalize;
  pNumQ->q1 *= Normalize;
  pNumQ->q2 *= Normalize;
  pNumQ->q3 *= Normalize;
}

/**
 * @brief  四元数转欧拉角
 * @code
 *      Quaternion_ToAngE(&q,&a);
 * @endcode
 * @param  *pNumQ: 输入四元数结构体
 * @param  *pAngE: 输出欧拉角结构体
 * @retval void
 */
void Quaternion_ToAngE(quaternion_t *pNumQ, imu_t *pAngE)
{
	float q0s = math_square(pNumQ->q0);
	float q1s = math_square(pNumQ->q1);
	float q2s = math_square(pNumQ->q2);
	float q3s = math_square(pNumQ->q3);
  float NumQ_T11 = q0s + q1s - q2s - q3s;																	//矩阵(1,1)项
  float NumQ_T12 = 2.0f*(pNumQ->q0*pNumQ->q3 + pNumQ->q1*pNumQ->q2);			//矩阵(2,1)项
	
	/*机体坐标系下的Z方向向量*/
  float vecxZ = 2.0f*(pNumQ->q1*pNumQ->q3 - pNumQ->q0*pNumQ->q2);			//矩阵(3,1)项
  float vecyZ = 2.0f*(pNumQ->q0*pNumQ->q1 + pNumQ->q2*pNumQ->q3);			//矩阵(3,2)项
  float veczZ = q0s - q1s - q2s + q3s;																//矩阵(3,3)项
	
//	vecxZ = math_limit(vecxZ,1,-1);
	//计算欧拉角
  pAngE->radian.Pitch		= asinf(vecxZ);
  pAngE->radian.Roll		= atan2f(vecyZ, veczZ);
	pAngE->radian.Yaw   	= atan2f(NumQ_T12, NumQ_T11);
	
	
	pAngE->angle.Pitch 		= -math_degree(pAngE->radian.Pitch);
	pAngE->angle.Roll 		= math_degree(pAngE->radian.Roll);
	pAngE->angle.Yaw 			= math_degree(pAngE->radian.Yaw);

	//提取机体坐标系下Z轴数据
	if(flag.CalibratingVel != 0)
	{
		if(flag.CalibratingVel == 1)
		{
			flag.CalibratingVel = 0;
			//计算静态Z轴加速度值
			gSelfState.velocityZ.base_acc_z = gSensorMpu.acc.angle.x*vecxZ + gSensorMpu.acc.angle.y*vecyZ + gSensorMpu.acc.angle.z*veczZ;
			gParam.Save();
		}
		else
		{
			flag.CalibratingVel --;
		}
	}
	static float last_vz = 0;
	//提取机体坐标系中的Z轴加速度
	gSelfState.velocityZ.acc_z = gSensorMpu.acc.angle.x*vecxZ + gSensorMpu.acc.angle.y*vecyZ + gSensorMpu.acc.angle.z*veczZ - gSelfState.velocityZ.base_acc_z;
	gSelfState.velocityZ.acc_z = LPF_1st(last_vz,gSelfState.velocityZ.acc_z,0.386f);//Z轴速度一阶低通滤波
	last_vz = gSelfState.velocityZ.acc_z;

	//Z速度、高度计算4ms 250hz
	static uint8_t hz_rote = 0;
	if(hz_rote >=2)
	{
		hz_rote=0;
		gSelfState.velocityZ.VelocityZ += (deadband(gSelfState.velocityZ.acc_z,0.04f) * POS_DT * G);
		gSelfState.velocityZ.VelocityZ *= 0.995f;//逐渐收敛
		
		float filteredZ;
		
		if(gSelfState.velocityZ.EstimatedZ == 0.0)
			filteredZ = gSensorBmp.AbsAltitude;
		else
			filteredZ = gSelfState.velocityZ.EstimatedZ * 0.95f + (1.0f - 0.95f) * gSensorBmp.AbsAltitude;
		
		gSelfState.velocityZ.EstimatedZ = filteredZ + 1.0f * gSelfState.velocityZ.VelocityZ * POS_DT;
		//gSelfState.velocityZ.h_Z = kalmanUpdate(gSelfState.velocityZ.VelocityZ,gSensorBmp.AbsAltitude,0.004);		
		//gSelfState.velocityZ.h_Z = (0.998) * (gSelfState.velocityZ.h_Z + gSelfState.velocityZ.a_h_Z * 0.004) + (0.002) *(gSensorBmp.AbsAltitude);
	}
	hz_rote++;
}


/**
 * @brief  读取传感器数据
 * @code
 *      Sensor_Get();
 * @endcode
 * @param  void
 * @retval void
 */
void Sensor_Get(void)
{
	static float x,y,z;
	xSemaphoreTake(flag.iic_sem, 0);
	//读取MPU6050数据
	gSensorMpu.Read();
	//读取气压计 or 温度计数据
	if(flag.IsBaro)
		gSensorBmp.Read();
	xSemaphoreGive(flag.iic_sem);
	//加速度计IIR滤波
	gSensorMpu.acc.angle.x = IIR_I_Filter(gSensorMpu.acc.averag.x * G_PER_LSB_8, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	gSensorMpu.acc.angle.y = IIR_I_Filter(gSensorMpu.acc.averag.y * G_PER_LSB_8, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	gSensorMpu.acc.angle.z = IIR_I_Filter(gSensorMpu.acc.averag.z * G_PER_LSB_8, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//陀螺仪一阶低通滤波
 	gSensorMpu.gyr.angle.x = LPF_1st(x,gSensorMpu.gyr.averag.x * DEG_PER_LSB_2000,0.386f);
 	gSensorMpu.gyr.angle.y = LPF_1st(y,gSensorMpu.gyr.averag.y * DEG_PER_LSB_2000,0.386f);
 	gSensorMpu.gyr.angle.z = LPF_1st(z,gSensorMpu.gyr.averag.z * DEG_PER_LSB_2000,0.386f);
	
	x = gSensorMpu.gyr.angle.x;
	y = gSensorMpu.gyr.angle.y;
	z = gSensorMpu.gyr.angle.z;
	
	//加速度计补偿
	#if 0
	float axx,axy,axz;
	float ayx,ayy,ayz;
	
	axx = gSensorMpu.acc.angle.x;
	axy = gSensorMpu.acc.angle.y * cos(gIMU.radian.Roll) + gSensorMpu.acc.angle.z * sin(gIMU.radian.Roll);
	axz = gSensorMpu.acc.angle.y * sin(gIMU.radian.Roll) + gSensorMpu.acc.angle.z * cos(gIMU.radian.Roll);
	
	ayx = axx * cos(gIMU.radian.Pitch) - axz * sin(gIMU.radian.Pitch);
	ayy = axy;
	ayz = axz * cos(gIMU.radian.Pitch) - axx * sin(gIMU.radian.Pitch);
	
	gSensorMpu.acc.angle.x = ayx;
	gSensorMpu.acc.angle.y = ayy;
	gSensorMpu.acc.angle.z = ayz;
	#endif
}


/**
 * @brief  获取一次当前角度
 * @code
 *      Angle_Get();
 * @endcode
 * @param  void
 * @retval void
 */
void Angle_Get(void)
{
	Sensor_Get();											//获取姿态数据
	Quaternion_Get(&gNumQ);						//获取四元数
	Quaternion_ToAngE(&gNumQ, &gIMU);	//四元数转欧拉角
}


