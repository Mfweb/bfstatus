/**
  ******************************************************************************
  * @file    eeprom.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini eeprom����(ʹ��ģ��i2c��AT24C02) ����V0.1��Ӳ��
	*					 EEPROM�����Ų�δ����PB6��PB7�ϣ���Ҫʹ�÷����������Ųſ�ʹ�ñ�����
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
	/* ��仺���� */
	for (i = 0; i < 256; i++)
	{
		buf[i] = 0xFF;
	}
	/* дEEPROM, ��ʼ��ַ = 0�����ݳ���Ϊ 256 */
	if (ee_WriteBytes(buf, 0, 256) == 0)
	{
		return;
	}
	else
	{
	}
}
