/**
  ******************************************************************************
  * @file    BF_Status_Mini_Math.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 数学函数
  ******************************************************************************
  */
#include "BF_Status_Mini_Math.h"

/* 快速计算 1/Sqrt(x) */
float Q_rsqrt(float number)
{
	float halfx = 0.5f * number;
	float y = number;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/* 可变增益自适应参数 */
float VariableParameter(float error)
{
	float  result = 0;
	if(error < 0)
		error = -error;
	if(error >0.6f)
		error = 0.6f;
	result = 1 - 1.667 * error;
	if(result < 0)
		result = 0;
	return result;
}

/*遥控数据归一化*/
float ScaleLinear(float data, float x_end, float die)
{
	if (data > die)
		return (data - die)/(x_end - die);
	else if(data < -die)
		return (data + die)/(x_end - die);
	else
		return 0.0f;
}
//排序
void SelectSort(float *arr,float *out,int16_t count)  
{
	int16_t i,j,min,temp;
	for(i=0;i<count;i++)
		out[i] = arr[i];
	for(i = 0;i < count;i++)
	{
		min = out[i];
		for(j = i;j < count;j++)
		{
			if(min>out[j])
			{
				temp = out[j];
				out[j] = min;
				min = temp;
			}
		}
	}
}
//死区
float deadband(float value, const float threshold)
{
	if (math_abs(value) < threshold)
	{
		value = 0;
	}
	else if (value > 0)
	{
		value -= threshold;
	}
	else if (value < 0)
	{
		value += threshold;
	}
	return value;
}
