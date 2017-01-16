#include "battery.h"
#include "yg.h"
int16_t power_adc = 0;
float power_value = 0.0f;

void power_get(void)
{
	power_adc = (int16_t)Read_ADC1_MultiChannel(8);
	power_value = (3.95f/2.99f)*((float)power_adc/4096.0f*3.3f) + 0.80;
}
