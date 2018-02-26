#include "battery.h"
#include "yg.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "battery.h"
#include "task.h"
int16_t power_adc = 0;
float power_value = 0.0f;
extern SemaphoreHandle_t adc_sem;

void power_get(void)
{
	xSemaphoreTake(adc_sem, 0);
	power_adc = (int16_t)Read_ADC1_MultiChannel(8);
	xSemaphoreGive(adc_sem);
	power_value = (3.67f/2.77f)*((float)power_adc/4096.0f*3.3f)+0.1;
}
