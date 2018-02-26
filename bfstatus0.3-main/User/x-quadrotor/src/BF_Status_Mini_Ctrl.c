/**
  ******************************************************************************
  * @file    BF_Status_Mini_Ctrl.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 控制文件
  ******************************************************************************
  */
#include "BF_Status_Mini_Ctrl.h"
#include "BF_Status_Mini_RC.h"
#include "BF_Status_Mini_Math.h"
#include "bmp280.h"


data_ctrl_t		gCtrl;		//控制
data_target_t gTarget;	//目标量

#define FLIP_RATE 10		//最大翻滚速率

#define CORE_CTRL_DT		0.002f
#define SHELL_CTRL_DT		0.004f


uint8_t take_off_status = 0;
_Bool StopHodeHeader = false;
uint16_t Lock_Count  = 0;//丢控锁定计数
float roll_flip_int  = 0;//roll方向翻滚陀螺仪积分角度


void CorePID(void);
void MotorOutPut(void);
void IntegralClear(void);
/**
 * @brief  一键起飞检测
 * @code
 *      TakeOff();
 * @endcode
 * @param  void
 * @retval void
 */
static void TakeOff(void)
{
	switch(take_off_status)
	{
		case 0:
			if(flag.HoldAltitude == true)//如果已经启动了定高
			{
				take_off_status = 1;
				break;
			}
			if(flag.LockYaw == true)//已经锁定机头  可以起飞
			{
				flag.HoldAltitude = true;
				gTarget.Altitude = gSelfState.velocityZ.EstimatedZ;//设置目标高度
				gTarget.base_throttle_set = 300;//设置定高油门
				gTarget.base_throttle = gTarget.base_throttle_set;
				gCtrl.alt.increment = 0;
				gCtrl.alt.last_error = 0;
				take_off_status = 1;
			}
			break;
		case 1:
			if(flag.LockYaw == false)
			{
				flag.HoldAltitude = false;
				take_off_status = 0;
			}
			break;
	}
}
/**
 * @brief  计算目标量
 * @code
 *      Calculate_Target();
 * @endcode
 * @param void
 * @retval void
 */
static void Calculate_Target(void)
{
	float radDiff,cosDiff,sinDiff,tr,tp;

	int16_t ftemp=0;
	gTarget.Pitch	=	-40.0f 		* ScaleLinear((float)(gRCData.Pitch	- 1500),500.0f,50.0f);
	gTarget.Roll 	=	-40.0f 		* ScaleLinear((float)(gRCData.Roll  - 1500),500.0f,50.0f);
	//使用遥控器配置无头模式
	if(gRCData.Setting.Hode_ALT_Mode)
	{
		if(StopHodeHeader==false)
		{
			//无头模式计算目标量
			radDiff = gIMU.radian.Yaw - gTarget.HodeHeader;//弧度

			cosDiff = cosf(radDiff);
			sinDiff = sinf(radDiff);
			
			tr=gTarget.Roll;
			tp=gTarget.Pitch;
			
			gTarget.Roll  = tr * cosDiff + tp * sinDiff;
			gTarget.Pitch = tp * cosDiff - tr * sinDiff;
		}
		else//翻滚时不进行无头计算
		{
			gTarget.Pitch = 0;
			gTarget.Roll = 0;
		}	
	}

	//如果0G状态且允许手抛
	if(flag.ZeroGravity && flag.Throwing ==1)
	{
		flag.Lock = false;
		flag.Throwing = 2;//进入手抛模式
		gRCData.Throttle = 700;//手抛油门
	}
  //油门超过最低检查值，用户期望起飞
	if(gRCData.Throttle > TH_MIN_CHECK)
	{
		if(flag.LockYaw != true)//如果航向没有锁定，则设置当前航向为目标航向
		{
			flag.LockYaw = true;
			gTarget.Yaw = gIMU.angle.Yaw; //将当前的航向做为目标航向
			if(gRCData.Setting.Hode_ALT_Mode)
				gTarget.HodeHeader = gIMU.radian.Yaw; //将当前的航向作为机头方向
		}
		Lock_Count = 0;
	}
	else
	{
		flag.Throwing = gRCData.Setting.Hand_Throwing==true?1:0;//手抛起飞设置
		if(!flag.Lock)
		{
			Lock_Count ++;
			if(Lock_Count > AUTO_LOCK_TIME)//解锁时间超限 重新锁定
			{
				Lock_Count = 0;
				flag.Lock = true;
			}
		}
		flag.LockYaw = false;	
		gTarget.Yaw   = gIMU.angle.Yaw;
		if(gRCData.Setting.Hode_ALT_Mode)
			gTarget.HodeHeader = gIMU.radian.Yaw;
	}
	
	//航向死区
	if((gRCData.Yaw > 1600)||(gRCData.Yaw < 1400))
	{
		ftemp = gRCData.Yaw - 1500; 
		gTarget.Yaw += (ftemp / 200.0f)*0.1f;
		//-180 +180处理
		if(gTarget.Yaw >180.0f)
			gTarget.Yaw -= 360.0f;	
		else if(gTarget.Yaw <-180.0f)
			gTarget.Yaw += 360.0f;
	}
	//设置里开启了定高起飞
	if(gRCData.Setting.Hode_ALT_Mode == true)
	{
		TakeOff();
	}
	//定高模式下目标高度控制
	if(flag.HoldAltitude)
	{
		if(gRCData.Throttle>650 || gRCData.Throttle<450)
		{
			float temp_th = (float)gRCData.Throttle-550;
			temp_th = temp_th>0?temp_th-100:temp_th;
			temp_th = temp_th<0?temp_th+100:temp_th;
			gTarget.Altitude += temp_th/400*0.001;//400/400*0.001*500 = 0.5M/S
		}
	}
	else
		gTarget.Altitude = gSelfState.velocityZ.EstimatedZ;


//printf("P:%3.2f R:%3.2f Y:%3.2f T:%d\r\n",Target.Pitch,Target.Roll,Target.Yaw,RC_Data.THROTTLE);
}

/**
 * @brief  估算定高油门
 * @code
 *      detecWeight(&t,&nt,&vz);
 * @endcode
 * @param  *thrust: 上次高度PID输出的相对油门+基础油门量
 * @param  *newThrust: 本次高度PID输出的相对油门量
 * @param  *velocity: Z轴速度
 * @retval void
 */
static void detecWeight(float *thrust, float *newThrust, float *velocity)
{
	static uint16_t cnt = 0;
	static float sum = 0.0;
	static float detaThrust = 0;
	
	if(math_abs(*velocity) < 0.02f && *thrust > 100.f)//飞行器在起飞状态  且Z轴相对静止
	{
		sum += *newThrust;//加上本次高度PID输出的相对油门
		if(cnt >= (uint16_t)(2.f/SHELL_CTRL_DT))	/*2S */
		{
			cnt = 0;
			uint16_t temp = math_limit(gTarget.base_throttle + sum/500.0f,900,15);//上1000次PID输出的平均值
			if(math_abs(gTarget.base_throttle_set - temp) > 15)//与之前设定基础油门偏差过大
			{
				gTarget.base_throttle_set = temp;//设置新的基础油门
				detaThrust = (gTarget.base_throttle_set - gTarget.base_throttle)/500.0f;//将偏差分成100分
				detaThrust = math_limit(detaThrust,1,-1);//限定小于每次补偿油门数，防止变化率过大
				LED1 = LED2 = LED3 = LED4 = OFF;
			}	
		}
		cnt++;
	}
	else
	{
		cnt = 0;	
		sum = 0.0;
	}
	
	if(math_abs(gTarget.base_throttle_set - gTarget.base_throttle)>1.0f)//次逐渐补偿到基础油门上
	{
		cnt = 0;	
		sum = 0.0;
		gTarget.base_throttle += detaThrust;
	}	
}
/**
 * @brief  更新PID
 * @code
 *      RunPID(&pid,true,&sd,0.01);
*      更新一次PID数据，PID对象为pid，使用传感器数据作为微分参数，传感器数据为sd，更新周期为0.01s
 * @endcode
 * @param  *pidOBJ: PID结构体对象
 * @param  d_sensor: 使用传感器数据作为D参数
 * @param  *sensor_data: 传感器数据（d_sensor为true时有效）
 * @param  dt:更新周期
 * @retval void
 */
static void RunPID(struct _pid* pidOBJ,const bool d_sensor,float *sensor_data,const float dt)
{
	pidOBJ->increment  += pidOBJ->error * dt; //积分
	pidOBJ->increment 	= math_limit(pidOBJ->increment,pidOBJ->increment_max,-pidOBJ->increment_max);//积分限幅
	pidOBJ->kp_out 			= pidOBJ->kp * pidOBJ->error;
	pidOBJ->ki_out 			= pidOBJ->ki * pidOBJ->increment;
	if(d_sensor == true)//使用传感器作为微分的参数 一般只用在角速率内环上
	{
		pidOBJ->kd_out 			= pidOBJ->kd * (pidOBJ->last_error - *sensor_data)/dt;
		pidOBJ->last_error  = *sensor_data;
	}
	else//不使用传感器作为微分参数
	{
		pidOBJ->kd_out 			= pidOBJ->kd * (pidOBJ->error - pidOBJ->last_error)/dt;
		pidOBJ->last_error 	= pidOBJ->error;	
	}
	//PID输出
	pidOBJ->pid_out 		= pidOBJ->kp_out + pidOBJ->ki_out + pidOBJ->kd_out;
}

/**
 * @brief  定高（高度环）
 * @code
 *      AltControl();
 * @endcode
 * @param  void
 * @retval void
 */
static float AltControl(void)
{
	static float last_output = 0;
	//高度环
	if(flag.HoldAltitude)
	{
		gCtrl.alt.error = gTarget.Altitude - gSelfState.velocityZ.EstimatedZ;
		if(math_abs(gCtrl.alt.error)<0.02f)
		{
			LED1 = LED2 = LED3 = LED4 = ON;
			gCtrl.alt.error=0;
		}
		RunPID(&gCtrl.alt,false,NULL,SHELL_CTRL_DT);
		gCtrl.throttle_rate.error = gCtrl.alt.pid_out - gSelfState.velocityZ.VelocityZ;
	}
	else//非定高模式 Z轴速度控制
	{
		gCtrl.throttle_rate.error = (gRCData.Throttle - gCtrl.throttle_rate.pid_out) - gSelfState.velocityZ.VelocityZ;
	}
	RunPID(&gCtrl.throttle_rate,true,&gSelfState.velocityZ.VelocityZ,SHELL_CTRL_DT);
	if(flag.HoldAltitude)//定高模式下加上基础油门以及基础油门检测
	{
		detecWeight(&last_output,&gCtrl.throttle_rate.pid_out,&gSelfState.velocityZ.VelocityZ);
		gCtrl.throttle_rate.pid_out += gTarget.base_throttle;//加上计算完的基础油门
		last_output = gCtrl.throttle_rate.pid_out;//保留结果	
	}
	//gCtrl.throttle_rate.pid_out = math_limit(gCtrl.throttle_rate.pid_out,TH_MAX_CHECK,TH_MIN_CHECK);
	if(flag.HoldAltitude)
		return gCtrl.throttle_rate.pid_out;
	else
		return gRCData.Throttle;
}
/**
 * @brief  飞行器控制（内环2ms，外环4ms）
 * @code
 *      Control();
 * @endcode
 * @param  void
 * @retval void
 */
void Control(void)   
{
	Calculate_Target();//计算目标量
	//外环
	if(gCtrl.ctrlRate >= 2)//2*2=4ms
	{
		gCtrl.ctrlRate = 0; 
		gCtrl.pitch.shell.error = gTarget.Pitch - gIMU.angle.Pitch;
		gCtrl.roll.shell.error  = gTarget.Roll  - gIMU.angle.Roll;
		//yaw偏差计算
    if((gTarget.Yaw - gIMU.angle.Yaw)>180 || (gTarget.Yaw - gIMU.angle.Yaw)<-180)
		{
       if(gTarget.Yaw>0 && gIMU.angle.Yaw<0)
				 gCtrl.yaw.shell.error = (-180 - gIMU.angle.Yaw) + (gTarget.Yaw - 180);
			 
       if(gTarget.Yaw<0 && gIMU.angle.Yaw>0)
				 gCtrl.yaw.shell.error = (180 - gIMU.angle.Yaw) + (gTarget.Yaw + 180);
    }
    else
		{
			gCtrl.yaw.shell.error = gTarget.Yaw - gIMU.angle.Yaw;
		}
		
		RunPID(&gCtrl.pitch.shell,false,NULL,SHELL_CTRL_DT);
		RunPID(&gCtrl.roll.shell,false,NULL,SHELL_CTRL_DT);
	  RunPID(&gCtrl.yaw.shell,false,NULL,SHELL_CTRL_DT);
	}
	
	if(gCtrl.ctrlRate_alt >= 2)//4ms
	{
		gCtrl.ctrlRate_alt = 0;
		gCtrl.throttle_out = AltControl();
	}
	gCtrl.ctrlRate ++;
	gCtrl.ctrlRate_alt ++;
  CorePID();//内环
	MotorOutPut();
}

/**
 * @brief  内环（角速率环）
 * @code
 *      CorePID();
 * @endcode
 * @param  void
 * @retval void
 */
static void CorePID(void)
{
	//翻滚控制
	if(flag.Flipping == 3)
	{
		gCtrl.pitch.shell.pid_out = 0;
		gCtrl.roll.shell.pid_out = 100;
		gCtrl.yaw.shell.pid_out = 0;
		roll_flip_int += gSensorMpu.gyr.angle.x * CORE_CTRL_DT;
		if(math_abs(roll_flip_int) >= 360)
		{
			flag.Flipping = 0;
			StopHodeHeader = false;
			IntegralClear();
		}
	}
	
	gCtrl.pitch.core.error = gCtrl.pitch.shell.pid_out - gSensorMpu.gyr.angle.y;
	gCtrl.roll.core.error  = gCtrl.roll.shell.pid_out  - gSensorMpu.gyr.angle.x;
	gCtrl.yaw.core.error   = gCtrl.yaw.shell.pid_out   - gSensorMpu.gyr.angle.z;
	RunPID(&gCtrl.pitch.core,true,&gSensorMpu.gyr.angle.y,CORE_CTRL_DT);
	RunPID(&gCtrl.roll.core ,true,&gSensorMpu.gyr.angle.x,CORE_CTRL_DT);
	RunPID(&gCtrl.yaw.core  ,true,&gSensorMpu.gyr.angle.z,CORE_CTRL_DT);
	
	gCtrl.pitch.core.pid_out = gCtrl.pitch.core.pid_out * 0.8 + gCtrl.pitch.shell.pid_out/2;
	gCtrl.roll.core.pid_out  = gCtrl.roll.core.pid_out  * 0.8 + gCtrl.roll.shell.pid_out/2;
}

/**
 * @brief  电机输出
 * @code
 *      MotorOutPut();
 * @endcode
 * @param  void
 * @retval void
 */
static void MotorOutPut(void)
{
	int pitch,roll,yaw;
	
	pitch = gCtrl.pitch.core.pid_out;
  roll  = gCtrl.roll.core.pid_out;    
 	yaw   = -gCtrl.yaw.core.pid_out;
	
  if(gRCData.Throttle > TH_MIN_CHECK)
	{
		gMotor.base = gCtrl.throttle_out;
		#if 0		//油门矫正
		if(!flag.flip)//翻滚的时候关闭油门矫正
			date_throttle	= (ctrl.throttle_out)/cos(Angle.radian.Roll)/cos(Angle.radian.Pitch);//油门校正
		#endif

		
		if(flag.Flipping)
		{
			static uint8_t flap_wait = 0;
//			static int16_t mt_out = 0;
//			static uint16_t m_rate = 0;
			
			if(flag.Flipping == 1)//第一阶段 提高上升速度
			{
				StopHodeHeader = true;
				gMotor.base += 300;
				flap_wait++;
				if(flap_wait > 50)//100MS
				{
					flag.Flipping = 2;
					flap_wait = 0;
				}
			}
			else if(flag.Flipping == 2)//第二阶段 左偏  用来抵消一部分反转后的速度
			{
				roll = -50;
				flap_wait++;
				if(flap_wait > 50)
				{
					roll = 0;
					flap_wait = 0;
					flag.Flipping = 3;
					roll_flip_int = 0;
				}
			}
		}
		gMotor.base = math_limit(gMotor.base,TH_MAX_CHECK,0);
		gMotor.M1 = gMotor.base + pitch + roll + yaw;
		gMotor.M2 = gMotor.base + pitch - roll - yaw;
		gMotor.M3 = gMotor.base - pitch - roll + yaw;
		gMotor.M4 = gMotor.base - pitch + roll - yaw;
		
		
		if(flag.Flipping==3)
		{
			gMotor.M2 = 0;
			gMotor.M3 = 0;
		}
	}
	else
	{	
		gMotor.M1 = gMotor.M2 = gMotor.M3 = gMotor.M4 = 0;
		IntegralClear();		
	}
	
	if(!flag.Lock)
		gMotor.Set();
	else
		gMotor.Stop();
}

/**
 * @brief  积分清零
 * @code
 *      IntegralClear();
 * @endcode
 * @param  void
 * @retval void
 */
static void IntegralClear(void)
{
	gCtrl.pitch.shell.increment	= 0;
  gCtrl.pitch.core.increment 	= 0;
	
  gCtrl.roll.core.increment		= 0;	
	gCtrl.roll.shell.increment	= 0;
	
	gCtrl.yaw.shell.increment		= 0;
	gCtrl.yaw.core.increment 		= 0;
	
	gCtrl.alt.increment		= 0;
}
