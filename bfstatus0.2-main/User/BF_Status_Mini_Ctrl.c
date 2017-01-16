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

int16_t Moto_duty[4];
uint16_t Lock_Count=0;
#ifdef HODE_HEADER
float HodeHeader=0.0f;
#endif
//计算目标
void Calculate_Target(void)
{
	#ifdef HODE_HEADER
	float radDiff,cosDiff,sinDiff,tr,tp;
	#endif
	int16_t ftemp=0;
	Target.Pitch	=	-40.0f 		* ScaleLinear((float)(RC_Data.Pitch	-1500),500.0f,50.0f);
	Target.Roll 	=	-40.0f 		* ScaleLinear((float)(RC_Data.Roll-1500),500.0f,50.0f);
	
	#ifdef HODE_HEADER
	if(!flag.flip)//翻滚时不进行无头计算
	{
		//无头模式计算目标量
		radDiff = Angle.radian.Yaw - HodeHeader;//弧度

		cosDiff = cosf(radDiff);
		sinDiff = sinf(radDiff);
		
		tr=Target.Roll;
		tp=Target.Pitch;
		
		Target.Roll  = tr * cosDiff - tp * sinDiff;
		Target.Pitch = tp * cosDiff + tr * sinDiff;
	}
	#endif
	//如果0G状态且允许手抛
	if(flag.zerog && flag.zero_pull ==1)
	{
		flag.Lock = 0;
		flag.zero_pull = 2;//进入手抛模式
		RC_Data.Throttle = 700;//手抛油门
	}
  //油门超过最低检查值，用户期望起飞
	if(RC_Data.Throttle > TH_MIN_CHECK)
	{
		if(flag.LockYaw != 1)//如果航向没有锁定，则设置当前航向为目标航向
		{
			flag.LockYaw = 1;
			Target.Yaw = Angle.angle.Yaw; //将当前的航向做为目标航向
			#ifdef HODE_HEADER
			HodeHeader = Angle.radian.Yaw; //将当前的航向作为机头方向
			#endif
			if(flag.IsBaro)//如果气压计自检完成并且已校准则记录起飞气压
				sensor_ms.data_baro_start = sensor_ms.data_baro_now;
		}
		Lock_Count = 0;
	}
	else
	{
		flag.zero_pull = 1;//禁止手抛起飞
		if(!flag.Lock)
		{
			Lock_Count ++;
			if(Lock_Count > AUTO_LOCK_TIME)//解锁时间超限 重新锁定
			{
				Lock_Count = 0;
				flag.Lock = 1;
			}
		}
		flag.LockYaw = 0;	
		Target.Yaw   = Angle.angle.Yaw;
		#ifdef HODE_HEADER
		HodeHeader = Angle.radian.Yaw;
		#endif
	}
	
	//航向死区
	if((RC_Data.Yaw > 1600)||(RC_Data.Yaw < 1400))
	{
		ftemp = RC_Data.Yaw - 1500; 
		Target.Yaw += (ftemp / 200.0f)*0.1f;
		//-180 +180处理
		if(Target.Yaw >180.0f)
			Target.Yaw -= 360.0f;	
		else if(Target.Yaw <-180.0f)
			Target.Yaw += 360.0f;
	}
	
//printf("P:%3.2f R:%3.2f Y:%3.2f T:%d\r\n",Target.Pitch,Target.Roll,Target.Yaw,RC_Data.THROTTLE);
}

void alt_ctrl(void)
{
	float deviation_alt;
	if(ctrl.ctrlRate_alt >= 10)//20ms控制周期
	{
		if(fabs(Angle.angle.Pitch)>60.0f || fabs(Angle.angle.Roll)>60.0f)flag.Lock =1;//强行锁定 调试时候用。。
		
		ctrl.ctrlRate_alt = 0;
		deviation_alt = sensor_ms.data_altitude_abs -  sensor_ms.hold_altitude;
		if(fabs(deviation_alt)<0.02)deviation_alt = 0;//高度死区2cm
		
		ctrl.alt.increment += deviation_alt;
		//积分限幅
		ctrl.alt.increment = math_limit(ctrl.alt.increment,ctrl.alt.increment_max,-ctrl.alt.increment_max);
		
		ctrl.alt.kp_out = deviation_alt 					* ctrl.alt.kp;
		ctrl.alt.ki_out = ctrl.alt.increment			* ctrl.alt.ki;
		ctrl.alt.kd_out = (deviation_alt-ctrl.alt.last_error) * ctrl.alt.kd;
		
		ctrl.alt.last_error 		= deviation_alt;
		
		ctrl.throttle_out = (int16_t)((float)RC_Data.Throttle - (ctrl.alt.kp_out + ctrl.alt.ki_out + ctrl.alt.kd_out));
		
		if(ctrl.throttle_out>TH_MAX_CHECK)ctrl.throttle_out = TH_MAX_CHECK;
		if(ctrl.throttle_out<0)ctrl.throttle_out = 0;
	}
	ctrl.ctrlRate_alt ++;
}
//PID控制   外环4ms  内环2ms
void Control(void)   
{
	float error_pitch,error_roll,error_yaw;//误差
	Calculate_Target();//计算目标量
	//外环PID
	if(ctrl.ctrlRate >= 2)
	{
		//pitch
	  error_pitch = Target.Pitch - Angle.angle.Pitch;		//计算偏差
		ctrl.pitch.shell.increment += error_pitch;	//积分
		//积分限幅
		ctrl.pitch.shell.increment = math_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * error_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//roll  在翻滚的时候是不执行roll控制的
		if(!flag.flip)
		{
			error_roll = Target.Roll - Angle.angle.Roll;			//计算偏差
			ctrl.roll.shell.increment += error_roll;		//积分
			//积分限幅
			ctrl.roll.shell.increment = math_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);
			ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * error_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		}
		else
			ctrl.roll.shell.increment = 0;
		
		//航向
    if((Target.Yaw - Angle.angle.Yaw)>180 || (Target.Yaw - Angle.angle.Yaw)<-180)
		{
       if(Target.Yaw>0 && Angle.angle.Yaw<0)
				 error_yaw = (-180 - Angle.angle.Yaw) + (Target.Yaw - 180);
			 
       if(Target.Yaw<0 && Angle.angle.Yaw>0)
				 error_yaw = (180 - Angle.angle.Yaw) + (Target.Yaw + 180);
    }
    else
		{
			error_yaw = Target.Yaw - Angle.angle.Yaw;
		}
		
	  ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * error_yaw;
    ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
  CorePID();//内环
	if(flag.hold_altitude && flag.IsBaro)//定高
		alt_ctrl();
	else
		ctrl.throttle_out = RC_Data.Throttle;
	
	Motor_OutPut();
}

//内环PID
void CorePID(void)
{
  float error_pitch,error_roll,error_yaw;
	
	// 计算偏差  
	error_pitch = ctrl.pitch.shell.pid_out - sensor_mpu.gyr.averag.y;
	error_roll  = ctrl.roll.shell.pid_out  - sensor_mpu.gyr.averag.x;
	error_yaw   = ctrl.yaw.shell.pid_out   - sensor_mpu.gyr.averag.z;
	
	// 积分
	ctrl.pitch.core.increment += error_pitch;
	if(!flag.flip)
		ctrl.roll.core.increment  += error_roll;
	else
		ctrl.roll.core.increment = 0;	
	ctrl.yaw.core.increment   += error_yaw;
	
	// 积分限幅
	ctrl.pitch.core.increment = math_limit(ctrl.pitch.core.increment,20,-20);
	if(!flag.flip) ctrl.roll.core.increment  = math_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = math_limit(ctrl.yaw.core.increment,20,-20);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * error_pitch;
	if(!flag.flip) ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * error_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * error_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
	if(!flag.flip) ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	// 微分
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor_mpu.gyr.histor.y - sensor_mpu.gyr.averag.y)*33;
	if(!flag.flip) ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (sensor_mpu.gyr.histor.x - sensor_mpu.gyr.averag.x)*33;
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (sensor_mpu.gyr.histor.z - sensor_mpu.gyr.averag.z)*33;	
	
	sensor_mpu.gyr.histor.x = sensor_mpu.gyr.averag.x;
	sensor_mpu.gyr.histor.y = sensor_mpu.gyr.averag.y;
  sensor_mpu.gyr.histor.z = sensor_mpu.gyr.averag.z;	
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8 + ctrl.pitch.shell.pid_out/2;
	ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8 + ctrl.roll.shell.pid_out/2; 
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out;

}
uint16_t int_left = 0;
//电机输出
void Motor_OutPut(void)
{
	int16_t pitch,roll,yaw;
	
	pitch = ctrl.pitch.core.pid_out;
  roll  = ctrl.roll.core.pid_out;    
 	yaw   = -ctrl.yaw.core.pid_out;
	
  if(RC_Data.Throttle > TH_MIN_CHECK)
	{
		#if 0
		int date_throttle	= (ctrl.throttle_out)/cos(Angle.angle.Roll/RtA)/cos((Angle.angle.Pitch)/RtA);//油门校正
		#else
		int date_throttle	= ctrl.throttle_out;
		#endif
		if(date_throttle > TH_MAX_CHECK)date_throttle = TH_MAX_CHECK;
		if(flag.flip)
		{
			if(flag.flip == 1)//第一阶段 在0点附近 可能不稳定 要强行转到一定角度
			{
				int_left += 5;
				Moto_duty[0] = date_throttle - int_left - pitch + yaw;
				Moto_duty[1] = date_throttle + int_left - pitch - yaw;
				Moto_duty[2] = date_throttle + int_left + pitch + yaw;
				Moto_duty[3] = date_throttle - int_left + pitch - yaw;
				if(Angle.angle.Roll > 3.0f)
				{
					flag.flip = 2;
				}
			}
			else if(flag.flip == 2)//第二阶段 超过了稳定角度  开始大幅度加速旋转
			{
				Moto_duty[0] = date_throttle - 100 - pitch + yaw;
				Moto_duty[1] = date_throttle + 100 - pitch - yaw;
				Moto_duty[2] = date_throttle + 100 + pitch + yaw;
				Moto_duty[3] = date_throttle - 100 + pitch - yaw;	
				if(Angle.angle.Roll < 0.0f || Angle.angle.Roll > 160.0f )
				{
					flag.flip = 3;
				}
			}
			else if(flag.flip == 3)
			{
				Moto_duty[0] = date_throttle + 200 - pitch + yaw;
				Moto_duty[1] = date_throttle - 200 - pitch - yaw;
				Moto_duty[2] = date_throttle - 200 + pitch + yaw;
				Moto_duty[3] = date_throttle + 200 + pitch - yaw;	
				if(Angle.angle.Roll > -30.0f)
				{
					int_left  = 0;
					flag.flip = 0;
					Integral_Clear();
				}				
			}
		}
		else
		{
			Moto_duty[0] = date_throttle - pitch - roll + yaw;
			Moto_duty[1] = date_throttle - pitch + roll - yaw;
			Moto_duty[2] = date_throttle + pitch + roll + yaw;
			Moto_duty[3] = date_throttle + pitch - roll - yaw;
		}
	}
	else
	{	
		Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = MOTOT_DIE;
		Integral_Clear();		
	}

	if(!flag.Lock)
		motor_output(Moto_duty[2],Moto_duty[3],Moto_duty[0],Moto_duty[1]);
	else
		Motor_Stop();	
}

//积分清零
void Integral_Clear(void)
{
	ctrl.pitch.shell.increment	= 0;
	ctrl.roll.shell.increment		= 0;	
  ctrl.pitch.core.increment 	= 0;		
  ctrl.roll.core.increment 		= 0;		
	ctrl.yaw.core.increment 		= 0;
}
