#ifndef __YG_H__
#define __YG_H__
#include "stm32f10x.h"
#include "stm32f10x_adc.h"

typedef struct {
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
	int16_t throttle;
}__dr;
typedef struct {
	float pitch;
	float roll;
	float yaw;
}__fr;
typedef struct {
	__dr dat;	//ҡ��ֵ(������)
	__dr org;	//ԭʼ
	__dr max;	//���
	__dr min;	//��С
	__dr nor;	//��һ��֮��
	__dr sta;	//��̬
	__fr ratio_n;//��ƫ�����У��
	__fr ratio_p;//��ƫ�����У��
}yg_dt;
extern yg_dt volatile YG_DATA;
void Get_YG(void);
void YG_INIT(void);
void wait_cab(uint8_t check);
float ScaleLinear(float x, float x_end, float deadband);
uint16_t Read_ADC1_MultiChannel(uint8_t channNo);
#endif
