/**
  ******************************************************************************
  * @file    BF_Status_Mini_Math.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini ��ѧ����
  ******************************************************************************
  */
#include "BF_Status_Mini_Math.h"

/* ���ټ��� 1/Sqrt(x) */
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5f;
	x2 = number*0.5f;
	y  = number;
	i  = *(long *) &y;
	i  = 0x5f375a86-(i>>1);
	y  = *(float *)&i;
	y  = y*(threehalfs-(x2*y*y));
	return y;
}

/* �ɱ���������Ӧ���� */
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

/*ң�����ݹ�һ��*/
float ScaleLinear(float data, float x_end, float die)
{
	if (data > die)
		return (data - die)/(x_end - die);
	else if(data < -die)
		return (data + die)/(x_end - die);
	else
		return 0.0f;
}
