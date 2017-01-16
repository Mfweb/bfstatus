#ifndef __BF_STATUS_MINI_MATH_H__
#define __BF_STATUS_MINI_MATH_H__
#include "stm32f10x.h"
#define PI 3.1415926535f
#define math_square(num) 				(((float)num)*((float)num))
#define math_abs(num) 					((num)<0?-(num):(num))
#define math_limit(num,max,min) ((num)>max?max:(num)<min?min:(num))//�����޷�
#define math_rad(num)						((num) * PI / 180.0f)//�Ƕ�ת����
#define math_degree(num)				((num) / PI * 180.0f)//����ת�Ƕ�

float Q_rsqrt(float number);
float VariableParameter(float error);
float ScaleLinear(float data, float x_end, float die);
#endif
