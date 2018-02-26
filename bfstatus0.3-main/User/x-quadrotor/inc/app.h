#ifndef __APP_H__
#define __APP_H__
#include "BF_Status_Mini_System.h"
#include "BF_Status_Mini_Ctrl.h"
#include "BF_Status_Mini_RC.h"
#include "DataScope_DP.h"
#include "hw_config.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "battery.h"
#include "task.h"
#include "semphr.h"

extern TaskHandle_t startTaskHandle;
void Task_Start(void *prm);
#endif
