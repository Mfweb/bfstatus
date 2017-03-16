#ifndef __BF_STATUS_MINI_GLOBAL_H__
#define __BF_STATUS_MINI_GLOBAL_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_Conf.h"


#define RtA 		57.324841    //弧度->角度单位 
#define AtR    	0.0174533		 //角度->弧度单位

#define Gyro_G 	0.03051756	 //  1/(65535/2000)=0.03051756   陀螺仪初始化+-1000度每秒
#define Gyro_Gr	0.0005426		//   AtR*Gyro_G
/* Flag */
typedef struct
{
	uint8_t battery_alarm;	//低电量警告
	uint8_t LockYaw;				//航向锁定
	uint8_t IsBaro;					//气压计存在
	uint8_t IsMAG;					//地磁存在并且检测通过
	uint8_t CalibratingACC; //正在标定加速度计 
	uint8_t CalibratingGYR;	//正在标定陀螺仪
	uint8_t CalibratingMAG; //正在标定地磁
	uint8_t Lock;						//是否锁定
	uint8_t f10hz;					//10Hz
	uint8_t f100hz;					//100Hz
	uint8_t f250hz;					//250Hz
	uint8_t need_back;			//需要返回数据
	uint8_t hold_altitude;	//定高
	uint8_t flip;						//是否正在翻转
	uint8_t zerog;					//0G状态
	uint8_t zero_pull;			//是否丢起来起飞
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
	struct _int16 origin;//原始值
	struct _float averag;//平均值
	struct _float histor;//历史值
	struct _int16 quiet; //静态值
	struct _float radian;//弧度值
}_sensor_data;



/*  罗盘  */
typedef struct
{
	struct _float origin;
	int16_t mag_limt[6];
}_sensor_data_mag;

/*  气压计  */
typedef struct
{
	uint16_t sens;			//压力灵敏度
	uint16_t off;				//压力抵消
	uint16_t tcs;				//温度压力灵敏度系数
	uint16_t tco;				//温度系数的压力抵消
	uint16_t tref;			//参考温度
	uint16_t tempsens;	//温度系数的温度
}_fac_dat;

typedef struct
{
	_fac_dat 	data_cal;							//工厂标定值
	float 		data_baro_start;			//起飞时的气压值
	float 		data_baro_now;				//当前气压值(帕)
	float 		data_temp_now;				//当前温度(摄氏度)
	float 		data_altitude_abs;		//当前高度(米)
	float			hold_altitude;				//要保持的高度(米)
	int32_t 	temp_org_det;					//温度原始值
	uint8_t 	state;								//当前状态0:启动温度转换 1:正在温度转换 2:启动气压转换 3:气压转换中
	uint8_t 	time_count;						//计时
	uint8_t 	baro_index;						//气压列队当前位置
	uint8_t 	temp_index;						//温度列队当前位置
	float			baro_data_buff[5];		//气压列队
	float			temp_data_buff[5];		//温度列队
	uint8_t 	get_count;						//开机至少获取10次以填充列队
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
	struct _pid shell;//外环PID参数
	struct _pid core;	//内环PID参数
};

typedef struct
{
	uint8_t  ctrlRate;		//控制频率
	uint8_t  ctrlRate_alt;//定高控制频率
	struct _tache pitch;
	struct _tache roll;
	struct _tache yaw;
	struct _pid alt;
	int16_t throttle_out;
}_ctrl;
/* 目标量 */
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}_target;

/* 遥控 */
typedef struct 
{
	int16_t Roll;
	int16_t Pitch;
	int16_t Throttle;//油门
	int16_t Yaw;
}_rc_getdata;

/*  重力  */
typedef __IO struct
{
  float x;
  float y;
  float z;
}Gravity;

/* 四元数 */
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

//角度
typedef struct
{
	struct __Ang radian;//弧度值
	struct __Ang angle;//角度值
}EulerAngle;

typedef struct
{
	float now_value;		//电池电量
	uint16_t adc_data;	//电池电量原始AD
}bat__;

typedef struct
{
	int x;
	int (*f)(char x);
}Handle_All;


extern EulerAngle Angle;						//姿态角
extern Quaternion NumQ;							//四元数
extern bat__ battery;								//电池状态
extern uint8_t TimeKatawa[3];	 			//时间片轮
extern type_flag flag;							//全局标志
extern _sensor_data_mpu sensor_mpu;	//MPU数据
extern _sensor_data_mag sensor_mag;	//罗盘数据
extern _ms_data sensor_ms; 					//气压计数据
extern _ctrl ctrl;									//控制
extern _target Target;							//目标值
extern _rc_getdata RC_Data;					//遥控数据
#endif
