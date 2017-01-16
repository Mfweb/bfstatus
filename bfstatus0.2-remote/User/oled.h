#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f10x.h"
void oled_init(void);
void oled_show_bit(uint8_t bit);
void oled_write_char(char c);
void oled_set_xy(uint8_t x,uint8_t y);
void oled_clear(void);
void oled_write_english_string(unsigned char X,unsigned char Y,char *s);
void oled_printf(uint8_t x,uint8_t y,const char *fmt, ...);
void APP_OLEDShow(void *p_arg);
#endif
