#ifndef __BF_STATUS_MINI_CTRL_H__
#define __BF_STATUS_MINI_CTRL_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_Global.h"
#include "BF_Status_Mini_System.h"
#include "BF_Status_Mini_Math.h"
#include "IMU.h"

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
	float error;
	float last_error;//上次误差
};
struct _tache
{
	struct _pid shell;//外环PID参数
	struct _pid core;	//内环PID参数
};

struct _pid_alt
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
	float last_error;//上次误差
	float prev_error;//上上次误差
	float base_throttle;//基础油门
	float base_throttle_set;
};

typedef struct
{
	uint8_t  ctrlRate;		//控制频率
	uint8_t  ctrlRate_alt;//定高控制频率
	struct _tache pitch;
	struct _tache roll;
	struct _tache yaw;
	struct _pid alt;
	struct _pid throttle_rate;
	int32_t throttle_out;
}data_ctrl_t;
/* 目标量 */
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
	float Altitude;
	float base_throttle;//基础油门
	float base_throttle_set;//定高基础油门
	float HodeHeader;//无头模式起飞标定头方向
}data_target_t;

extern data_ctrl_t gCtrl;									//控制
extern data_target_t gTarget;							//目标值

void Control(void);
#endif
