#include "app.h"
#include <stdlib.h>
TaskHandle_t startTaskHandle;
//控制进程
static void Task_Ctrl(void *prm)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(2));//2MS
		gIMU.Get();		//姿态解算
		Control();			//控制输出
	}
}

//USB数据处理进程
static void Task_USB(void *prm)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	uint8_t USB_Rec_Data[30];
	uint8_t read_lean = 0;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(50));//50MS
		Send_Once();
		read_lean = USB_RxRead(USB_Rec_Data, 30);
		if(read_lean != 0)
		{
			if(USB_Rec_Data[0] == 'r' && USB_Rec_Data[1] == 'e' && 
				USB_Rec_Data[2] == 'b' && USB_Rec_Data[3] == 'o' && 
			USB_Rec_Data[4] == 'o' && USB_Rec_Data[5] == 't' && flag.Lock == true)
			{
				Jump_To_IAP();
			}
		}
	}
}
//系统状态处理进程
static void Task_SysSta(void *prm)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(100));//100MS
		led_ref();
		gBattery.Read();//获取电池电量
		if(flag.PowerOff || POWER_KEY==KEY_DOWN)//关机
		{
			while(POWER_KEY==KEY_DOWN);
			POWER_OFF();
			gBattery.Reboot(ENTER_STOP);//复位进入关机状态
		}
	}
}
//遥控接收进程
static void Task_RC(void *prm)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(10));//5MS
		//遥控数据获取
		if(RC_GetData() == TRUE)
		{
			RC_SendData();//回发参数
			LED1 = !LED1;
		}
	}
}

//启动进程
void Task_Start(void *prm)
{
	taskENTER_CRITICAL();//进入临界区
	xTaskCreate(Task_Ctrl,	"M_CTRL",	600,NULL, 6, NULL);//控制
	xTaskCreate(Task_RC,		"M_RC",		600,NULL, 5, NULL);//遥控接收
	xTaskCreate(Task_USB,		"M_USB",	400,NULL, 4, NULL);//USB发送接收
	xTaskCreate(Task_SysSta,"M_SYS",	100,NULL, 3, NULL);//系统状态处理
	printf("Free heap:%d bytes\n",xPortGetFreeHeapSize());
	vTaskDelete(startTaskHandle);
	taskEXIT_CRITICAL();//退出临界区
}

