#ifndef __BF_STATUS_MINI_FILTER_H__
#define __BF_STATUS_MINI_FILTER_H__
#include "stm32f10x.h"
#include "kalman_filter.h"
#include <stdio.h>
#include <stdlib.h>
/* IIR参数 */
#define  IIR_ORDER     4      //IIR滤波器的阶数
extern double b_IIR[IIR_ORDER+1];  //系数b
extern double a_IIR[IIR_ORDER+1];  //系数a
extern double b_IIR_b[IIR_ORDER+1];
extern double a_IIR_b[IIR_ORDER+1];
extern double InPut_IIR[3][IIR_ORDER+1];
extern double OutPut_IIR[3][IIR_ORDER+1];

typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
} lpf2pData;

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
float  LPF_1st(float oldData, float newData, float lpf_factor);
void Least_Square_push(float x,float y);
float Least_Square_Calc(float x);
void presssureFilter(float* in, float* out);

//二阶低通滤波
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
#endif
