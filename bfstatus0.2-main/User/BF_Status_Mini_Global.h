#ifndef __BF_STATUS_MINI_GLOBAL_H__
#define __BF_STATUS_MINI_GLOBAL_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_Conf.h"


#define RtA 		57.324841    //����->�Ƕȵ�λ 
#define AtR    	0.0174533		 //�Ƕ�->���ȵ�λ

#define Gyro_G 	0.03051756	 //  1/(65535/2000)=0.03051756   �����ǳ�ʼ��+-1000��ÿ��
#define Gyro_Gr	0.0005426		//   AtR*Gyro_G
/* Flag */
typedef struct
{
	uint8_t battery_alarm;	//�͵�������
	uint8_t LockYaw;				//��������
	uint8_t IsBaro;					//��ѹ�ƴ���
	uint8_t IsMAG;					//�شŴ��ڲ��Ҽ��ͨ��
	uint8_t CalibratingACC; //���ڱ궨���ٶȼ� 
	uint8_t CalibratingGYR;	//���ڱ궨������
	uint8_t CalibratingMAG; //���ڱ궨�ش�
	uint8_t Lock;						//�Ƿ�����
	uint8_t f10hz;					//10Hz
	uint8_t f100hz;					//100Hz
	uint8_t f250hz;					//250Hz
	uint8_t need_back;			//��Ҫ��������
	uint8_t hold_altitude;	//����
	uint8_t flip;						//�Ƿ����ڷ�ת
	uint8_t zerog;					//0G״̬
	uint8_t zero_pull;			//�Ƿ��������
}type_flag;

struct _int16
{
	int16_t x;
	int16_t y;
	int16_t z;
};
struct _float
{
	float x;
	float y;
	float z;
};

typedef struct
{
	struct _int16 origin;//ԭʼֵ
	struct _float averag;//ƽ��ֵ
	struct _float histor;//��ʷֵ
	struct _int16 quiet; //��ֵ̬
	struct _float radian;//����ֵ
}_sensor_data;



/*  ����  */
typedef struct
{
	struct _float origin;
	int16_t mag_limt[6];
}_sensor_data_mag;

/*  ��ѹ��  */
typedef struct
{
	uint16_t sens;			//ѹ��������
	uint16_t off;				//ѹ������
	uint16_t tcs;				//�¶�ѹ��������ϵ��
	uint16_t tco;				//�¶�ϵ����ѹ������
	uint16_t tref;			//�ο��¶�
	uint16_t tempsens;	//�¶�ϵ�����¶�
}_fac_dat;

typedef struct
{
	_fac_dat 	data_cal;							//�����궨ֵ
	float 		data_baro_start;			//���ʱ����ѹֵ
	float 		data_baro_now;				//��ǰ��ѹֵ(��)
	float 		data_temp_now;				//��ǰ�¶�(���϶�)
	float 		data_altitude_abs;		//��ǰ�߶�(��)
	float			hold_altitude;				//Ҫ���ֵĸ߶�(��)
	int32_t 	temp_org_det;					//�¶�ԭʼֵ
	uint8_t 	state;								//��ǰ״̬0:�����¶�ת�� 1:�����¶�ת�� 2:������ѹת�� 3:��ѹת����
	uint8_t 	time_count;						//��ʱ
	uint8_t 	baro_index;						//��ѹ�жӵ�ǰλ��
	uint8_t 	temp_index;						//�¶��жӵ�ǰλ��
	float			baro_data_buff[5];		//��ѹ�ж�
	float			temp_data_buff[5];		//�¶��ж�
	uint8_t 	get_count;						//�������ٻ�ȡ10��������ж�
}_ms_data;

/*  MPU6050  */
typedef struct
{
	_sensor_data acc;
	_sensor_data gyr;
	int temp[5];
}_sensor_data_mpu;

/*  PID  */
struct _pid
{
	float kp;
	float ki;
	float kd;
	float increment;
	float increment_max;
	float kp_out;
	float ki_out;
	float kd_out;
	float pid_out;
	float last_error;
};
struct _tache
{
	struct _pid shell;//�⻷PID����
	struct _pid core;	//�ڻ�PID����
};

typedef struct
{
	uint8_t  ctrlRate;		//����Ƶ��
	uint8_t  ctrlRate_alt;//���߿���Ƶ��
	struct _tache pitch;
	struct _tache roll;
	struct _tache yaw;
	struct _pid alt;
	int16_t throttle_out;
}_ctrl;
/* Ŀ���� */
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}_target;

/* ң�� */
typedef struct 
{
	int16_t Roll;
	int16_t Pitch;
	int16_t Throttle;//����
	int16_t Yaw;
}_rc_getdata;

/*  ����  */
typedef __IO struct
{
  float x;
  float y;
  float z;
}Gravity;

/* ��Ԫ�� */
typedef __IO struct
{
  float q0;
  float q1;
  float q2;
  float q3;
}Quaternion;

struct __Ang
{
  float Pitch;
  float Roll;
  float Yaw;
};

//�Ƕ�
typedef struct
{
	struct __Ang radian;//����ֵ
	struct __Ang angle;//�Ƕ�ֵ
}EulerAngle;

typedef struct
{
	float now_value;		//��ص���
	uint16_t adc_data;	//��ص���ԭʼAD
}bat__;

typedef struct
{
	int x;
	int (*f)(char x);
}Handle_All;


extern EulerAngle Angle;						//��̬��
extern Quaternion NumQ;							//��Ԫ��
extern bat__ battery;								//���״̬
extern uint8_t TimeKatawa[3];	 			//ʱ��Ƭ��
extern type_flag flag;							//ȫ�ֱ�־
extern _sensor_data_mpu sensor_mpu;	//MPU����
extern _sensor_data_mag sensor_mag;	//��������
extern _ms_data sensor_ms; 					//��ѹ������
extern _ctrl ctrl;									//����
extern _target Target;							//Ŀ��ֵ
extern _rc_getdata RC_Data;					//ң������
#endif
