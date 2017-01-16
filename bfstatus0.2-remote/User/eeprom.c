#include "eeprom.h"
#include "oled.h"
#include "yg.h"
#include "stdio.h"

void read_ee(void)
{
	uint8_t i,buff[24]={0xff};
	I2C_EE_BufferRead(buff,0,24);
	YG_DATA.max.pitch		 = (int16_t)buff[0] | ((int16_t)buff[1]<<8);
	YG_DATA.max.roll		 = (int16_t)buff[2] | ((int16_t)buff[3]<<8);
	YG_DATA.max.yaw			 = (int16_t)buff[4] | ((int16_t)buff[5]<<8);
	YG_DATA.max.throttle = (int16_t)buff[6] | ((int16_t)buff[7]<<8);
	
	YG_DATA.min.pitch		 = (int16_t)buff[8] | ((int16_t)buff[9]<<8);
	YG_DATA.min.roll		 = (int16_t)buff[10] | ((int16_t)buff[11]<<8);
	YG_DATA.min.yaw			 = (int16_t)buff[12] | ((int16_t)buff[13]<<8);
	YG_DATA.min.throttle = (int16_t)buff[14] | ((int16_t)buff[15]<<8);
	
	YG_DATA.sta.pitch		 = (int16_t)buff[16] | ((int16_t)buff[17]<<8);
	YG_DATA.sta.roll		 = (int16_t)buff[18] | ((int16_t)buff[19]<<8);
	YG_DATA.sta.yaw			 = (int16_t)buff[20] | ((int16_t)buff[21]<<8);
	YG_DATA.sta.throttle = (int16_t)buff[22] | ((int16_t)buff[23]<<8);
	for(i=0;i<24;i++)
		printf("0x%02x ",buff[i]);
	printf("\r\n");
}


void save_ee(void)
{
	uint8_t buff[24],i;
	buff[0] = (uint8_t)YG_DATA.max.pitch;
	buff[1] = (uint8_t)(YG_DATA.max.pitch>>8);
	buff[2] = (uint8_t)YG_DATA.max.roll;
	buff[3] = (uint8_t)(YG_DATA.max.roll>>8);
	buff[4] = (uint8_t)YG_DATA.max.yaw;
	buff[5] = (uint8_t)(YG_DATA.max.yaw>>8);
	buff[6] = (uint8_t)YG_DATA.max.throttle;
	buff[7] = (uint8_t)(YG_DATA.max.throttle>>8);
	
	buff[8] = (uint8_t)YG_DATA.min.pitch;
	buff[9] = (uint8_t)(YG_DATA.min.pitch>>8);
	buff[10] = (uint8_t)YG_DATA.min.roll;
	buff[11] = (uint8_t)(YG_DATA.min.roll>>8);
	buff[12] = (uint8_t)YG_DATA.min.yaw;
	buff[13] = (uint8_t)(YG_DATA.min.yaw>>8);
	buff[14] = (uint8_t)YG_DATA.min.throttle;
	buff[15] = (uint8_t)(YG_DATA.min.throttle>>8);
	
	buff[16] = (uint8_t)YG_DATA.sta.pitch;
	buff[17] = (uint8_t)(YG_DATA.sta.pitch>>8);
	buff[18] = (uint8_t)YG_DATA.sta.roll;
	buff[19] = (uint8_t)(YG_DATA.sta.roll>>8);
	buff[20] = (uint8_t)YG_DATA.sta.yaw;
	buff[21] = (uint8_t)(YG_DATA.sta.yaw>>8);
	buff[22] = (uint8_t)YG_DATA.sta.throttle;
	buff[23] = (uint8_t)(YG_DATA.sta.throttle>>8);
//	if(!write_bytes(0xA0,0,buff,24))
//	{
//		oled_printf(0,0,"save error");
//		while(1);
//	}
	I2C_EE_BufferWrite(buff,0,24);
	for(i=0;i<24;i++)
	{
		printf("0x%02x ",buff[i]);
	}
	printf("\r\n");
}

uint8_t check_null(void)
{
	uint8_t i,buff[24]={0xff};
	I2C_EE_BufferRead(buff,0,24);
	for(i=0;i<24;i++)
	{
		if(buff[i] != 0xff)return 1;
	}
	return 0;
}
