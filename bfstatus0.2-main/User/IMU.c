#include "IMU.h"
#include "DataScope_DP.h"
#include "ms5611.h"

#define KpDef 0.8f						//0.16
#define KiDef 0.0005f					//0.001
#define SampleRateHalf 0.001f //0.001

/* ��ȡ��Ч���Ҿ����е��������� */
Gravity Quaternion_vectorGravity(Quaternion *pNumQ)
{
	Gravity G;
  G.x = 2     *(pNumQ->q1 * pNumQ->q3 - pNumQ->q0 * pNumQ->q2);								
  G.y = 2     *(pNumQ->q0 * pNumQ->q1 + pNumQ->q2 * pNumQ->q3);						  
  G.z = 1 - 2 *(pNumQ->q1 * pNumQ->q1 + pNumQ->q2 * pNumQ->q2);
	return G;
}

/* ��Ԫ����һ�� */
void Quaternion_Normalize(Quaternion *pNumQ)
{
  float Normalize = 0.0f;
 
	Normalize = Q_rsqrt(math_square(pNumQ->q0) + math_square(pNumQ->q1) + math_square(pNumQ->q2) + math_square(pNumQ->q3));

  pNumQ->q0 = pNumQ->q0 * Normalize;
  pNumQ->q1 = pNumQ->q1 * Normalize;
  pNumQ->q2 = pNumQ->q2 * Normalize;
  pNumQ->q3 = pNumQ->q3 * Normalize;
}

/* ������Ԫ�� */
void Quaternion_RungeKutta(Quaternion *pNumQ, float GyrX, float GyrY, float GyrZ, float helfTimes)
{
  float tmpq0 = pNumQ->q0;
  float tmpq1 = pNumQ->q1;
  float tmpq2 = pNumQ->q2;
  float tmpq3 = pNumQ->q3;

  pNumQ->q0 = pNumQ->q0 + (-tmpq1*GyrX - tmpq2*GyrY - tmpq3*GyrZ) * helfTimes;
  pNumQ->q1 = pNumQ->q1 + ( tmpq0*GyrX - tmpq3*GyrY + tmpq2*GyrZ) * helfTimes;
  pNumQ->q2 = pNumQ->q2 + ( tmpq3*GyrX + tmpq0*GyrY - tmpq1*GyrZ) * helfTimes;
  pNumQ->q3 = pNumQ->q3 + (-tmpq2*GyrX + tmpq1*GyrY + tmpq0*GyrZ) * helfTimes;
}
/* ��ȡ��Ԫ�� */
void Quaternion_Get(Quaternion *pNumQ)
{
  float ErrX, ErrY, ErrZ;
  float AccX, AccY, AccZ;
  float GyrX, GyrY, GyrZ;
	float Normalize;
  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
	Gravity V;

	//���ٶȹ�һ��
	Normalize = Q_rsqrt(math_square(sensor_mpu.acc.averag.x)+ math_square(sensor_mpu.acc.averag.y) +math_square(sensor_mpu.acc.averag.z));
	AccX = sensor_mpu.acc.averag.x*Normalize;
  AccY = sensor_mpu.acc.averag.y*Normalize;
  AccZ = sensor_mpu.acc.averag.z*Normalize;
	//��ȡ��������
	V = Quaternion_vectorGravity(&NumQ);
	
	/**/
	//ȥ�����ٶȼƵ�����������
	//e_x = AccX-V.x;
	//e_y = AccY-V.y;
	//e_z = AccZ-V.z;
	
	//e_send(AccX-V.x,AccY-V.y,AccZ-V.z);
	/**/
	//�������
 	ErrX = (AccY*V.z - AccZ*V.y);
  ErrY = (AccZ*V.x - AccX*V.z);
  ErrZ = (AccX*V.y - AccY*V.x);
	
 	exInt = exInt + ErrX * KiDef;
  eyInt = eyInt + ErrY * KiDef;
  ezInt = ezInt + ErrZ * KiDef;
	
  GyrX = math_rad(sensor_mpu.gyr.averag.x) + KpDef * VariableParameter(ErrX) * ErrX  +  exInt;
  GyrY = math_rad(sensor_mpu.gyr.averag.y) + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
	GyrZ = math_rad(sensor_mpu.gyr.averag.z) + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt;
	
	//��Ԫ������
	Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);
	//��Ԫ����һ��
	Quaternion_Normalize(&NumQ);
}

/* ��Ԫ��תŷ���� */
void Quaternion_ToAngE(Quaternion *pNumQ, EulerAngle *pAngE)
{
  float NumQ_T11 = pNumQ->q0*pNumQ->q0 + pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 - pNumQ->q3*pNumQ->q3;
  float NumQ_T12 = 2.0f*(pNumQ->q0*pNumQ->q3 + pNumQ->q1*pNumQ->q2);
  float NumQ_T13 = 2.0f*(pNumQ->q1*pNumQ->q3 - pNumQ->q0*pNumQ->q2);
  float NumQ_T23 = 2.0f*(pNumQ->q0*pNumQ->q1 + pNumQ->q2*pNumQ->q3);
  float NumQ_T33 = pNumQ->q0*pNumQ->q0 - pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 + pNumQ->q3*pNumQ->q3;

  pAngE->radian.Pitch	= -asinf(NumQ_T13);
  pAngE->radian.Roll		= atan2f(NumQ_T23, NumQ_T33);
	
	if(!flag.IsMAG)
		pAngE->radian.Yaw    = atan2f(NumQ_T12, NumQ_T11);
}

void Sensor_Get(void)
{
	static float x,y,z;
	//��ȡMPU6050����
	MPU6050_Read();
	//��ȡ5883L����
	if(flag.IsMAG)
	{
		HMC5883L_Read();
	}
	//��ȡ��ѹ�� or �¶ȼ�����
	if(flag.IsBaro)
	{
		ms5611_handle();
	}
	//���ٶȼ�IIR�˲�
	sensor_mpu.acc.averag.x = IIR_I_Filter(sensor_mpu.acc.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor_mpu.acc.averag.y = IIR_I_Filter(sensor_mpu.acc.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor_mpu.acc.averag.z = IIR_I_Filter(sensor_mpu.acc.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//������һ�׵�ͨ�˲�
 	sensor_mpu.gyr.averag.x = LPF_1st(x,sensor_mpu.gyr.radian.x * Gyro_G,0.386f);
	x = sensor_mpu.gyr.averag.x;
 	sensor_mpu.gyr.averag.y = LPF_1st(y,sensor_mpu.gyr.radian.y * Gyro_G,0.386f);
	y = sensor_mpu.gyr.averag.y;
 	sensor_mpu.gyr.averag.z = LPF_1st(z,sensor_mpu.gyr.radian.z * Gyro_G,0.386f);
	z = sensor_mpu.gyr.averag.z;
}

//��ȡ�Ƕ�
void Angle_Get(void)
{
	Sensor_Get();											//��ȡ��̬����
	Quaternion_Get(&NumQ);						//��ȡ��Ԫ��
	Quaternion_ToAngE(&NumQ, &Angle);	//��Ԫ��תŷ����

	if(flag.IsMAG)
	{
		float sin_pitch,sin_roll,cos_roll,cos_pitch;
		float hx,hy,mag_yaw;
		sin_roll  = sin(Angle.radian.Roll);
		sin_pitch = sin(Angle.radian.Pitch);
		cos_roll  = cos(Angle.radian.Roll);
		cos_pitch = cos(Angle.radian.Pitch);
		//�ش���ǲ���
		hx = sensor_mag.origin.x * cos_pitch + sensor_mag.origin.y * sin_pitch * sin_roll - sensor_mag.origin.z * cos_roll * sin_pitch; 
		hy = sensor_mag.origin.y * cos_roll  + sensor_mag.origin.z * sin_roll;
		//���õشŽ��㺽���
		mag_yaw = -math_degree(atan2((float)hy,(float)hx));
		//�����ǻ��ֽ��㺽���
		Angle.angle.Yaw += math_degree(sensor_mpu.gyr.averag.z * Gyro_Gr * 2 * SampleRateHalf);

		if((mag_yaw>90 && Angle.angle.Yaw<-90) || (mag_yaw<-90 && Angle.angle.Yaw>90)) 
				Angle.angle.Yaw = -Angle.angle.Yaw * 0.998f + mag_yaw * 0.002f;
		else
			Angle.angle.Yaw = Angle.angle.Yaw * 0.998f + mag_yaw * 0.002f;
	}
	else 
		Angle.angle.Yaw = math_degree(Angle.radian.Yaw);
	Angle.angle.Pitch = math_degree(Angle.radian.Pitch);
	Angle.angle.Roll = math_degree(Angle.radian.Roll);
}


