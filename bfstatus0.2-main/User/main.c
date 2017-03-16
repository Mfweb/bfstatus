/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    http://mfweb.top/     h@mfweb.pw
	*						关于硬件：V0.1硬件中的AT24C02的I2C没有挂接在MPU使用的总线中，建议修改硬件
  ******************************************************************************
  */
	
#include "BF_Status_Mini_System.h"
#include "BF_Status_Mini_RC.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "DataScope_DP.h"
#include "battery.h"
#include "hw_config.h"

#ifdef __USB_ENABLE__
uint8_t buf[200] = {0};
#endif
int main(void)
{
	system_init();
	
	while(1)
	{
		if(flag.f250hz)
		{
			flag.f250hz = 0;
			if(flag.need_back)
			{
				#ifdef RC_SEND_DATA
				RC_SendData();//回发参数
				#endif
				//printf("send_success\r\n");
				flag.need_back = 0;
			}
		}
		if(flag.f100hz)
		{
			#ifdef __USB_ENABLE__
			uint32_t len = 0;
			#endif
			flag.f100hz = 0;
			battery_read();//获取电池电量
			#ifdef __USB_ENABLE__
			len = USB_RxRead(buf, sizeof(buf));
			
			if (len > 0)
			{
				USB_TxWrite(buf, len);
			}
			#endif
	 	}
		if(flag.f10hz)
		{
			flag.f10hz = 0;
			led_ref();
			//printf("%d %d %d \r\n",MAG[0],MAG[1],MAG[2]);
			//Send_Once();
			//printf("%.2f	%.2f	     %.2f	%.2f\r\n",Angle.Pitch,sensor_mpu.gyr.averag.x,Angle.Roll,sensor_mpu.gyr.averag.y);
			//printf("%.2f %.2f %.2f \r\n",sensor_ms.data_baro_now,sensor_ms.data_temp_now,sensor_ms.data_altitude_abs);
			//printf("%.2f	%.2f	%d\r\n",sensor_ms.data_altitude_abs,sensor_ms.hold_altitude - sensor_ms.data_altitude_abs,ctrl.throttle_out);
			//printf("%d\r\n",battery_value);
		}
	}
}
