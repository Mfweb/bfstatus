#ifndef __APP_H__
#define __APP_H__
#include "stm32f10x.h"
#include "battery.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"


typedef struct{
	uint8_t rx_dr;//成功收到一次数据
	uint16_t lost_count;//丢控计数
	uint8_t is_lost;//已经丢控
	
	uint8_t send_power_off;//发送关机指令（持续）
	uint8_t send_hold;//发送定高指令
	
	uint8_t send_flip;//发送翻滚指令
	uint8_t power_off;//发送关机指令
}app_data_;

extern app_data_ app_data;
extern TaskHandle_t startTaskHandle;

void Task_Start(void *prm);
#endif
