#ifndef __EEPROM_H__
#define __EEPROM_H__
#include "stm32f10x_i2c.h"
uint8_t read_ee(void);
void save_ee(void);
#endif
