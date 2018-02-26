#include "BF_Status_Mini_Filter.h"
#include "BF_Status_Mini_Math.h"
#include <math.h>

double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};
//气压计用滤波参数
double b_IIR_b[IIR_ORDER+1] = {0.002570856077857,0.01028342431143,0.01542513646714,0.01028342431143,0.002570856077857};
double a_IIR_b[IIR_ORDER+1] = {1,-2.895894355896,3.478823912095,-2.003187935728,0.4636171034895};

/**
 * @brief  IIR直接I型滤波器
 * @code
 *      	gSensorMpu.acc.angle.x = IIR_I_Filter(gSensorMpu.acc.averag.x * G_PER_LSB_8, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
 * @endcode
 * @param  InData: 输入的当前数据
 * @param  *x: 存储未滤波的数据
 * @param  *y: 存储滤波后的数据
 * @param  *b: 存储系数B
 * @param  nb: B系数长度
 * @param  *a: 存储系数A
 * @param  na: A系数长度
 * @retval 滤波后的数据
 */
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

/**
 * @brief  一阶低通滤波
 * @code
 *      x = LPF_1st(a,b,0.3);
 *			a = x;
 * @endcode
 * @param  oldData: 上次的数据
 * @param  newData: 本次的数据
 * @param  lpf_factor: 滤波系数
 * @retval 滤波后的数据
 */
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1.0f - lpf_factor) + newData * lpf_factor;
}

/* 以下为最小二乘 */
#define Least_Square_LEN 10
struct Least_Square_Data_Unit
{
	float x;
	float y;
};
  
struct Least_Square_Data
{
	struct Least_Square_Data_Unit data[Least_Square_LEN];
	int len;
};
struct Least_Square_Data Data =
{
	.len = 0
};
/**
 * @brief  最小二乘压入数据
 * @code
 *      Least_Square_push(1.1,5.0);
 * @endcode
 * @param  x: 数据X坐标
 * @param  y: 数据y坐标
 * @retval void
 */
void Least_Square_push(float x,float y)
{
	uint8_t i = 0;
	if (Data.len < Least_Square_LEN)
	{
		Data.data[Data.len].x = x;
		Data.data[Data.len++].y = y;
		return;
	}

	//数据移动,去掉最后一个数据  
	for (i = 0;i < Least_Square_LEN - 2;i++)
	{
		Data.data[i].x = Data.data[i + 1].x;
		Data.data[i].y = Data.data[i + 1].y;
	}
	Data.data[Least_Square_LEN-1].x = x;
	Data.data[Least_Square_LEN-1].y = y;
	Data.len = Least_Square_LEN;
}
/**
 * @brief  最小二乘获得曲线参数以及计算x对应的y
 * @code
 *      float x7 = Least_Square_Calc(7.0);
 * @endcode
 * @param  x: 要计算的x坐标
 * @retval 对应x的y值
 */
float Least_Square_Calc(float x)
{
	int i = 0;
	float mean_x = 0;
	float mean_y = 0;
	float num1 = 0;
	float num2 = 0;
	float a = 0;
	float b = 0;
	//求t,y的均值  
	for (i = 0;i < Data.len;i++)
	{
		mean_x += Data.data[i].x;
		mean_y += Data.data[i].y;
	}
	mean_x /= Data.len;
	mean_y /= Data.len;
	for (i = 0;i < Data.len;i++)
	{
		num1 += (Data.data[i].x - mean_x) * (Data.data[i].y - mean_y);
		num2 += (Data.data[i].x - mean_x) * (Data.data[i].x - mean_x);
	}
	b = num1 / num2;
	a = mean_y - b * mean_x;
	return (a + b * x);
}


/* 以下为限幅平均滤波 */
#define PresssureFilter_NUM	5
#define PresssureFilter_A	1.f

/**
 * @brief  限幅平均滤波
 * @code
 *      presssureFilter(&n,&o);
 *      输入n，输出o
 * @endcode
 * @param  in: 输入的数据
 * @param  out: 输出滤波后的数据
 * @retval void
 */
void presssureFilter(float* in, float* out)
{	
	static uint8_t i=0;
	static float filter_buf[PresssureFilter_NUM]={0.0};
	double filter_sum=0.0;
	uint8_t cnt=0;	
	float deta;

	if(filter_buf[i] == 0.0f)
	{
		filter_buf[i]=*in;
		*out=*in;
		if(++i>=PresssureFilter_NUM)	i=0;
	}
	else 
	{
		if(i)
			deta=*in-filter_buf[i-1];
		else
			deta=*in-filter_buf[PresssureFilter_NUM-1];

		if(fabs(deta)<PresssureFilter_A)
		{
			filter_buf[i] = *in;
			if(++i>=PresssureFilter_NUM)
				i=0;
		}
		for(cnt=0;cnt<PresssureFilter_NUM;cnt++)
		{
			filter_sum+=filter_buf[cnt];
		}
		*out=filter_sum /PresssureFilter_NUM;
	}
}


/* 以下为二阶低通滤波 */

/**
 * @brief  执行一次二阶低通滤波
 * @code
 *      lpf2pApply(&lp2,0.002);
 * @endcode
 * @param  *lpfData: 二阶低通滤波参数结构体
 * @param  sample: 当前数据
 * @retval 滤波后的数据
 */
float lpf2pApply(lpf2pData* lpfData, float sample)
{
  float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
  if (!isfinite(delay_element_0)) {
    // don't allow bad values to propigate via the filter
    delay_element_0 = sample;
  }

  float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

  lpfData->delay_element_2 = lpfData->delay_element_1;
  lpfData->delay_element_1 = delay_element_0;
  return output;
}

/**
 * @brief  二阶低通滤波设置截止频率
 * @code
 *      lpf2pSetCutoffFreq(&lp2,500,30);
 *      lp2参数设置采样频率500hz，截至频率30hz
 * @endcode
 * @param  *lpfData: 要设置的结构体
 * @param  sample_freq: 采样频率
 * @param  cutoff_freq: 截至频率
 * @retval void
 */
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  float fr = sample_freq/cutoff_freq;
  float ohm = tanf(PI/fr);
  float c = 1.0f+2.0f*cosf(PI/4.0f)*ohm+ohm*ohm;
  lpfData->b0 = ohm*ohm/c;
  lpfData->b1 = 2.0f*lpfData->b0;
  lpfData->b2 = lpfData->b0;
  lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
  lpfData->a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
  lpfData->delay_element_1 = 0.0f;
  lpfData->delay_element_2 = 0.0f;
}

/**
 * @brief  二阶低通滤波初始化
 * @code
 *      lpf2pInit(&lp2,500,30);
 *			lp2参数设置采样频率500hz，截至频率30hz
 * @endcode
 * @param  *lpfData: 要设置的结构体
 * @param  sample_freq: 采样频率
 * @param  cutoff_freq: 截至频率
 * @retval void
 */
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  if (lpfData == NULL || cutoff_freq <= 0.0f)
	{
    return;
  }
  lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

