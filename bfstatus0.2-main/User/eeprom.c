/**
  ******************************************************************************
  * @file    eeprom.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini eeprom驱动(使用模拟i2c、AT24C02) 对于V0.1的硬件
	*					 EEPROM的引脚并未接在PB6、PB7上，需要使用飞线连接引脚才可使用本代码
  ******************************************************************************
  */
#include "eeprom.h"
#include "stm32f10x_gpio.h"
#include "stm32fx_delay.h"

uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize)
{
	I2C_Read(EE_DEV_ADDR, _usAddress, _usSize, _pReadBuf);
	return 1;
}


uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i;
	for(i=0;i<_usSize;i++)
	{
		Single_Write(EE_DEV_ADDR,_usAddress+i,_pWriteBuf[i]);
		DelayMs(10);
	}
	return 1;
}

void ee_Erase(void)
{
	uint16_t i;
	uint8_t buf[256];
	/* 填充缓冲区 */
	for (i = 0; i < 256; i++)
	{
		buf[i] = 0xFF;
	}
	/* 写EEPROM, 起始地址 = 0，数据长度为 256 */
	if (ee_WriteBytes(buf, 0, 256) == 0)
	{
		return;
	}
	else
	{
	}
}
