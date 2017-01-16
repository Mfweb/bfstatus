#include "BF_Status_Mini_Filter.h"

double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //ϵ��b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//ϵ��a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};
/*====================================================================================================*/
/*====================================================================================================*
** ��������: IIR_I_Filter
** ��������: IIRֱ��I���˲���
** ��    ��: InData Ϊ��ǰ����
**           *x     ����δ�˲�������
**           *y     �����˲��������
**           *b     ����ϵ��b
**           *a     ����ϵ��a
**           nb     ����*b�ĳ���
**           na     ����*a�ĳ���
**           LpfFactor
** ��    ��: OutData         
** ˵    ��: ��
** ����ԭ��: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
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

//һ�׵�ͨ�˲�
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1.0f - lpf_factor) + newData * lpf_factor;
}
