#ifndef __MENU_H__
#define __MENU_H__
#include "stm32f10x.h"
#include "oled.h"
#include "key_led.h"
#include "stm32fx_delay.h"
#define MENU_LEN 5
struct menu_p{
	const image_data *data;			//本菜单显示数据
	int8_t selected;						//本菜单是否已经选择
	struct menu_dt * next_menu;	//本菜单指向的下一级菜单
};

struct menu_dt{
	int8_t now;		//当前在第几个
	uint8_t select_run;	//选择执行了什么功能
	struct menu_p data[MENU_LEN];//菜单列表
	struct menu_dt *previous;//本菜单指向的上一级菜单
	int8_t menu_len;//本层菜单数量
};

#define SET_HODE_ALT 	(1<<0)		//定高模式1		手动模式0
#define SET_HODE_HEADER  	(1<<1)		//无头模式1		有头模式0
#define SET_CTRLRATEL (1<<2)		//控制速度低 0 //低	1	//中	0	//高
#define SET_CTRLRATEH (1<<3)		//控制速度高 0			0				1
#define SET_COLLISION	(1<<4)		//碰撞检测开1 关0

typedef struct{
	uint8_t data1;
}menu_s;

typedef struct{
	uint8_t flip;				//翻滚
	uint8_t throwing;		//手抛起飞
}flag_;
extern flag_ flag;
extern struct menu_dt menu;
extern menu_s menu_setting;
void menu_init(void);
void menu_loop(void);

#endif
