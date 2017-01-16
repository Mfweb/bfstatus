#ifndef __MS5611_H__
#define __MS5611_H__
#include "stm32f10x.h"
#include "BF_Status_Mini_Global.h"
void ms5611_reset(void);
void ms5611_read_fac(void);
void ms5611_handle(void);
#endif
