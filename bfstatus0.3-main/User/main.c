#include "BF_Status_Mini_System.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"

int main(void)
{
	system_init();
	xTaskCreate(Task_Start, "Start!", 100,NULL, 1, &startTaskHandle);
	vTaskStartScheduler();//启动调度
	
	while(1);
}
