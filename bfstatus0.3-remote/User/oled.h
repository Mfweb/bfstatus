#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f10x.h"
typedef struct
{
	uint16_t width;
	uint16_t height;
	unsigned char **data;
}image_data;

extern const image_data data_img_lock;
extern const image_data data_img_unlock;
extern const image_data singal_full;
extern const image_data singal_null;
extern const image_data bat_00;
extern const image_data bat_01;
extern const image_data bat_02;
extern const image_data bat_03;
extern const image_data bat_04;
extern const image_data bat_05;
extern const image_data hold_mode_data;
extern const image_data hand_mode_data;
extern const image_data uheard_mode_data;
extern const image_data heard_mode_data;
//一级菜单
extern const image_data menu_fly_mode;
extern const image_data menu_ctrl_mode;
extern const image_data menu_ctrl_rate;
extern const image_data menu_function;
extern const image_data menu_calibration;
//二级菜单，选项
extern const image_data menu_fly_mode_hode;
extern const image_data menu_fly_mode_hand;
extern const image_data menu_ctrl_mode_uheard;
extern const image_data menu_ctrl_mode_heard;
extern const image_data menu_ctrl_rate_slow;
extern const image_data menu_ctrl_rate_medium;
extern const image_data menu_ctrl_rate_fast;
extern const image_data menu_function_collision;
extern const image_data menu_function_throwing;
extern const image_data menu_calibration_acc;
extern const image_data menu_calibration_gyr;
extern const image_data menu_calibration_joy;
//菜单选择及其他符号
extern const image_data menu_select;
extern const image_data menu_unselect;
extern const image_data menu_pointer;
extern const image_data menu_clear_pointer;
extern const image_data menu_clear_line;

extern const image_data start;

void oled_init(void);
void oled_write_char(char *c);
void oled_set_xy(uint8_t x,uint8_t y);
void oled_clear(void);
void oled_write_english_string(unsigned char X,unsigned char Y,char *s);
void oled_printf(uint8_t x,uint8_t y,const char *fmt, ...);
void APP_OLEDShow(void *p_arg);
void oled_show_image(uint8_t x,uint8_t y,const image_data *buf);

#endif
