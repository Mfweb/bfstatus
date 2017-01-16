#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

extern unsigned char DataScope_OutPut_Buffer[42];
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);
void Send_Once(void);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);
void Send_Th_dat(float a,float b,float c);
#endif 



