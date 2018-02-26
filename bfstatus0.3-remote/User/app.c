#include "app.h"
#include "yg.h"
#include "nrf24l01.h"
#include "key_led.h"
#include <stdio.h>
#include "stm32fx_delay.h"
#include "menu.h"

app_data_ app_data={
	.rx_dr = 0,//成功收到一次数据
	.lost_count = 0,//丢控计数
	.is_lost = 1,//已经丢控
	.send_power_off = 0,//发送关机指令（持续）
	.send_hold = 0,//发送定高指令
	.send_flip = 0,//发送翻滚指令
	.power_off = 0//发送关机指令
};
//任务句柄
TaskHandle_t startTaskHandle;//开始任务句柄
TaskHandle_t SendTaskHandle;
TaskHandle_t DisplayTaskHandle;
TaskHandle_t KeyTaskHandle;
//信号量
SemaphoreHandle_t adc_sem;
//数据发送进程
static void Task_Send(void *prm)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(20));//10MS
		//PAout(10) = 1;
		if(NRF_Rx_Dat(gSensorNRF.RX_BUF) == RX_DR)
		{
			app_data.rx_dr=1;
			app_data.lost_count = 0;
			app_data.is_lost = 0;
			LED1 = !LED1;
			LED4 = OFF;
		}
		else
		{
			app_data.rx_dr = 0;
			if(app_data.is_lost == 0)
			{
				app_data.lost_count ++;
				if(app_data.lost_count >= 100)//1S  超时1S则进入丢控状态
				{
					app_data.is_lost = 1;
					app_data.lost_count = 0;
					LED4 = ON;
				}
			}
		}
		Get_YG(1);
		gSensorNRF.TX_BUF[0] = (YG_DATA.dat.pitch >> 8);
		gSensorNRF.TX_BUF[1] = (uint8_t)YG_DATA.dat.pitch;
		
		gSensorNRF.TX_BUF[2] = (YG_DATA.dat.roll >> 8);
		gSensorNRF.TX_BUF[3] = (uint8_t)YG_DATA.dat.roll;
		
		gSensorNRF.TX_BUF[4] = (YG_DATA.dat.yaw >> 8);
		gSensorNRF.TX_BUF[5] = (uint8_t)YG_DATA.dat.yaw;
		
		gSensorNRF.TX_BUF[6] = (YG_DATA.dat.throttle >> 8);
		gSensorNRF.TX_BUF[7] = (uint8_t)YG_DATA.dat.throttle;
		gSensorNRF.TX_BUF[8] = 0x00;//0-7：校准加速度计  校准陀螺仪  解锁  请求返回数据  校准地磁 定高（测试）  翻滚  关机
		if(menu.select_run==1)
		{
			menu.select_run = 0;
			gSensorNRF.TX_BUF[8]|=0x01;//校准加速度计
		}
		
		if(menu.select_run==2)
		{
			menu.select_run = 0;
			gSensorNRF.TX_BUF[8]|=0x02;//校准陀螺仪
		}
		
		if(KEY6==KEY_DOWN)//解锁指令不需要判断按键抬起，直接发送
			gSensorNRF.TX_BUF[8]|=0x04;//解锁
		
		gSensorNRF.TX_BUF[8]|=0x08;	//请求返回数据

		if(menu.select_run==3)
		{
			menu.select_run = 0;
			gSensorNRF.TX_BUF[8]|=0x10;//校准地磁
		}
		
		if(app_data.send_hold)
		{
			app_data.send_hold = 0;
			gSensorNRF.TX_BUF[8]|=0x20;//定高
		}
		
		if(app_data.send_flip)//翻滚
		{
			app_data.send_flip = 0;
			gSensorNRF.TX_BUF[8]|=0x40;
		}
		
		if(app_data.power_off || app_data.send_power_off)//关机  关机要连续发送到丢控为止
		{
			if(!app_data.send_power_off)//第一次进入关机状态
			{
				app_data.is_lost = 0;//清空标志
				app_data.send_power_off = 1;//一直到丢控为止都要发送关机指令
			}
			else
			{
				if(app_data.is_lost)
				{
					app_data.send_power_off = 0;
				}
			}
			app_data.power_off = 0;
			gSensorNRF.TX_BUF[8]|=0x80;
		}
		
		gSensorNRF.TX_BUF[9] = 0;//byte9为菜单配置输出，将配置输出到飞行器
		
		if(menu_setting.data1 & SET_HODE_ALT)//定高模式
			gSensorNRF.TX_BUF[9] |= 0x01;
		if(menu_setting.data1 & SET_HODE_HEADER)//无头模式
			gSensorNRF.TX_BUF[9] |= 0x02;
		if(menu_setting.data1 & SET_COLLISION)//碰撞检测
			gSensorNRF.TX_BUF[9] |= 0x04;
		if(flag.throwing)//手抛起飞
			gSensorNRF.TX_BUF[9] |= 0x08;
		
		NRF_Tx_Dat(gSensorNRF.TX_BUF);
		LED2 = !LED2;
		//PAout(10) = 0;
	}
}

//显示进程
static void Task_Display(void *prm)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(100));//100MS
		power_get();
		menu_loop();
	}
}

//按键处理进程
static void Task_KeyHandle(void *prm)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(50));//20MS
		
		if(menu.select_run==4)
		{
			menu.select_run = 0;
			//vTaskSuspend(SendTaskHandle);
			vTaskSuspend(DisplayTaskHandle);
			oled_clear();
			wait_cab(0);
			oled_clear();
			oled_printf(12	,0,"X-Mini V0.1");
			//vTaskResume(SendTaskHandle);
			vTaskResume(DisplayTaskHandle);
		}

		if(KEY5==KEY_DOWN)
		{
			vTaskDelay(pdMS_TO_TICKS(2));
			while(KEY5==KEY_DOWN)vTaskDelay(pdMS_TO_TICKS(2));
			app_data.send_hold = 1;
		}
		
		if(KEY3==KEY_DOWN)
		{
			vTaskDelay(pdMS_TO_TICKS(2));
			while(KEY3==KEY_DOWN)vTaskDelay(pdMS_TO_TICKS(2));
			app_data.send_flip = 1;
		}
		if(KEY4==KEY_DOWN)
		{
			vTaskDelay(pdMS_TO_TICKS(2));
			while(KEY4==KEY_DOWN)vTaskDelay(pdMS_TO_TICKS(2));
			app_data.power_off = 1;
		}
		LED3 = !LED3;
	}
}

void Task_Start(void *prm)
{
	taskENTER_CRITICAL();//进入临界区
	adc_sem = xSemaphoreCreateMutex();
	xTaskCreate(Task_Send,			"M_Send",				1200,NULL, 4, &SendTaskHandle);
	xTaskCreate(Task_KeyHandle,	"M_KeyHandle",	1200,NULL, 3, &KeyTaskHandle);
	xTaskCreate(Task_Display,		"M_Display",		1500,NULL, 2, &DisplayTaskHandle);
	
	oled_show_image(0	,0,&start);
	oled_printf(64,0,"fheap:%db",xPortGetFreeHeapSize());
	//vTaskDelay(pdMS_TO_TICKS(1000));
	DelayMs(1000);
	oled_clear();
	oled_printf(12	,0,"X-Mini V0.1");
	//printf("Free heap:%d bytes\n",xPortGetFreeHeapSize());
	vTaskDelete(startTaskHandle);
	taskEXIT_CRITICAL();//退出临界区
}
