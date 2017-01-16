#ifndef __EEPROM_H__
#define __EEPROM_H__
#include "stm32f10x_i2c.h"
void read_ee(void);
void save_ee(void);
uint8_t check_null(void);
#endif
