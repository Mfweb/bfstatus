#ifndef __BF_STATUS_MINI_PARAM_H__
#define __BF_STATUS_MINI_PARAM_H__
#include "stm32f10x.h"
#include <stdbool.h>

typedef struct{
	void (*Init)(void);
	void (*Save)(void);
	void (*Load)(void);
	void (*ClearFlag)(void);
}param_t;

extern param_t gParam;
#endif

