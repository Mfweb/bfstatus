#ifndef __EEPROM_H__
#define __EEPROM_H__
#include "stm32f10x.h"
#include "stm32f10x_flash.h"

#define EE_DEV_ADDR			0xA0


uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize);
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize);

#endif
