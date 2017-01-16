#include "BF_Status_Mini_Filter.h"

double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};
/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
    x[i]=x[i-1];
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
    z1 += x[i]*b[i];
  
  for(i=na-1; i>0; i--)
    y[i]=y[i-1];
  
  for(z2=0,i=1; i<na; i++)
    z2 += y[i]*a[i];
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

//一阶低通滤波
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1.0f - lpf_factor) + newData * lpf_factor;
}
