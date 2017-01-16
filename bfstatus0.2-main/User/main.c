/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    http://mfweb.top/     h@mfweb.pw
	*						����Ӳ����V0.1Ӳ���е�AT24C02��I2Cû�йҽ���MPUʹ�õ������У������޸�Ӳ��
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
				RC_SendData();//�ط�����
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
			battery_read();//��ȡ��ص���
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
