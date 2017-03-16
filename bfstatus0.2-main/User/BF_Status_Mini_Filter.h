#ifndef __BF_STATUS_MINI_FILTER_H__
#define __BF_STATUS_MINI_FILTER_H__
#include "stm32f10x.h"

/* IIR参数 */
#define  IIR_ORDER     4      //IIR滤波器的阶数
extern double b_IIR[IIR_ORDER+1];  //系数b
extern double a_IIR[IIR_ORDER+1];  //系数a
extern double InPut_IIR[3][IIR_ORDER+1];
extern double OutPut_IIR[3][IIR_ORDER+1];

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
float  LPF_1st(float oldData, float newData, float lpf_factor);
#endif
