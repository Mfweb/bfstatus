#ifndef __BF_STATUS_MINI_MATH_H__
#define __BF_STATUS_MINI_MATH_H__
#include "stm32f10x.h"

#define RtA	57.324841    //弧度->角度单位 
#define AtR	0.0174533		 //角度->弧度单位

#define G		9.81f
#define PI	3.14159265358979323846

#define math_square(num) 				(((float)num)*((float)num))
#define math_abs(num) 					((num)<0?-(num):(num))
#define math_limit(num,max,min) ((num)>max?max:(num)<min?min:(num))//数据限幅
#define math_rad(num)						((num) * PI / 180.0f)//角度转弧度
#define math_degree(num)				((num) / PI * 180.0f)//弧度转角度

float Q_rsqrt(float number);
float VariableParameter(float error);
float ScaleLinear(float data, float x_end, float die);
void SelectSort(float *arr,float *out,int16_t count) ;
float deadband(float value, const float threshold);
#endif
