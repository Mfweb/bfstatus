/**
  ******************************************************************************
  * @file    menu.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 菜单
  ******************************************************************************
  */
#include "menu.h"
#include <stdio.h>
#include <string.h>
#include "yg.h"
#include "battery.h"
#include "freertos.h"
#include "task.h"
#include "nrf24l01.h"
#include "app.h"

float Roll=0,Pitch=0,Yaw=0;

unsigned char show_dt_[1][128]={0};
const image_data show_dt={
	.width=128,
	.height=1,
	.data = (unsigned char**)show_dt_
};

struct menu_dt menu;//主菜单
struct menu_dt fly_mode;	//飞行模式
struct menu_dt ctrl_mode;	//控制模式
struct menu_dt ctrl_rate;	//控制速率
struct menu_dt function;	//功能
struct menu_dt calibration;	//校准

struct menu_dt * menu_now;	//当前正在显示的菜单

menu_s menu_setting={0x1A};//默认配置
flag_ flag = {0,0};
/* 将保存的数据载入到菜单显示中 */
void load_setting(void)
{
	//清空菜单配置
	fly_mode.data[0].selected = 0;
	fly_mode.data[1].selected = 0;
	ctrl_mode.data[0].selected = 0;
	ctrl_mode.data[1].selected = 0;
	ctrl_rate.data[0].selected = 0;
	ctrl_rate.data[1].selected = 0;
	ctrl_rate.data[2].selected = 0;
	function.data[0].selected = 0;
	function.data[1].selected = 0;
	calibration.data[0].selected = 0;
	calibration.data[1].selected = 0;
	calibration.data[2].selected = 0;
	//配置信息更新到菜单
	if(menu_setting.data1 & SET_HODE_ALT)//定高模式  手动模式
		fly_mode.data[0].selected = 1;
	else
		fly_mode.data[1].selected = 1;
	
	if(menu_setting.data1 & SET_HODE_HEADER)//无头模式  有头模式
		ctrl_mode.data[0].selected = 1;
	else
		ctrl_mode.data[1].selected = 1;
	
	if(!(menu_setting.data1 & SET_CTRLRATEL) && !(menu_setting.data1 & SET_CTRLRATEH))//控制速率
		ctrl_rate.data[0].selected = 1;
	else if((menu_setting.data1 & SET_CTRLRATEL) && !(menu_setting.data1 & SET_CTRLRATEH))
		ctrl_rate.data[1].selected = 1;
	else
		ctrl_rate.data[2].selected = 1;
	
	if(menu_setting.data1 & SET_COLLISION)//碰撞检测
		function.data[0].selected = 1;
	
	if(flag.throwing)//手抛起飞
		function.data[1].selected = 1;
}
/* 将菜单中设置的数据保存 */
void save_setting(void)
{
	menu_setting.data1 = 0x00;
	flag.throwing = 0;
	if(fly_mode.data[0].selected)menu_setting.data1 |= SET_HODE_ALT;
	if(ctrl_mode.data[0].selected)menu_setting.data1 |= SET_HODE_HEADER;
	//if(ctrl_rate.data[0].selected){}
	if(ctrl_rate.data[1].selected){menu_setting.data1 |= SET_CTRLRATEL;}
	else if(ctrl_rate.data[2].selected){menu_setting.data1 |= SET_CTRLRATEH;}
	
	if(function.data[0].selected)menu_setting.data1 |= SET_COLLISION;
	if(function.data[1].selected)flag.throwing = 1;
}
/* 菜单参数初始化 */
void menu_init(void)
{
	//配置主菜单
	menu.now = 0;
	menu.select_run = 0;
	menu.menu_len = MENU_LEN;
	menu.data[0].data = &menu_fly_mode;
	menu.data[0].next_menu = &fly_mode;
	menu.data[1].data = &menu_ctrl_mode;
	menu.data[1].next_menu = &ctrl_mode;
	menu.data[2].data = &menu_ctrl_rate;
	menu.data[2].next_menu = &ctrl_rate;
	menu.data[3].data = &menu_function;
	menu.data[3].next_menu = &function;
	menu.data[4].data = &menu_calibration;
	menu.data[4].next_menu = &calibration;
	menu.previous = NULL;
	//配置子菜单
	for(uint8_t i=0;i<MENU_LEN;i++)
	{
		menu.data[i].next_menu->previous = &menu;
		menu.data[i].next_menu->now = 0;
		menu.data[i].next_menu->select_run = 0;
		menu.data[i].next_menu->data[0].next_menu = NULL;
		menu.data[i].next_menu->data[1].next_menu = NULL;
		menu.data[i].next_menu->data[2].next_menu = NULL;
		menu.data[i].next_menu->data[3].next_menu = NULL;
		menu.data[i].next_menu->data[4].next_menu = NULL;
	}
	fly_mode.data[0].data = &menu_fly_mode_hode;
	fly_mode.data[1].data = &menu_fly_mode_hand;
	fly_mode.menu_len = 2;
	
	ctrl_mode.data[0].data = &menu_ctrl_mode_uheard;
	ctrl_mode.data[1].data = &menu_ctrl_mode_heard;
	ctrl_mode.menu_len = 2;
	
	ctrl_rate.data[0].data = &menu_ctrl_rate_slow;
	ctrl_rate.data[1].data = &menu_ctrl_rate_medium;
	ctrl_rate.data[2].data = &menu_ctrl_rate_fast;
	ctrl_rate.menu_len = 3;
	
	function.data[0].data = &menu_function_collision;
	function.data[1].data = &menu_function_throwing;
	function.menu_len = 2;
	
	calibration.data[0].data = &menu_calibration_acc;
	calibration.data[1].data = &menu_calibration_gyr;
	calibration.data[2].data = &menu_calibration_joy;
	calibration.menu_len = 3;
	//当前所在的菜单
	menu_now = NULL;
	load_setting();
}
/* 根据电池电量获取贴图 */
static const image_data *get_bat_icon(float bat_data,uint8_t target)
{
	const float bat_value_rm[2][5] = {
	{4.1,4.0,3.9,3.8,3.7},//遥控
	{4.1,4.0,3.9,3.8,3.2}//飞机
	};
	
	if(bat_data >= bat_value_rm[target][0])
		return &bat_05;
	if(bat_data >= bat_value_rm[target][1])
		return &bat_04;
	if(bat_data >= bat_value_rm[target][2])
		return &bat_03;
	if(bat_data >= bat_value_rm[target][3])
		return &bat_02;
	if(bat_data >= bat_value_rm[target][4])
		return &bat_01;
	return &bat_00;
}
/* 显示摇杆数据 */
static void show_yg_data(void)
{
	float p,r,y,t;
	/* 显示遥控的摇杆状态 */
	p = 40.0f* ScaleLinear((float)(YG_DATA.dat.pitch-1500),500.0f,50.0f);
	r = 40.0f* ScaleLinear((float)(YG_DATA.dat.roll	-1500),500.0f,50.0f);
	y = 180.0f/3.1415926f * ScaleLinear((float)(YG_DATA.dat.yaw-1500),500.0f,70.0f);
	t = (float)(YG_DATA.dat.throttle-1000);

	
	unsigned char temp_data_p = p+64;
	unsigned char temp_data_r = r+64;
	unsigned char temp_data_y = y*0.9 + 64;
	unsigned char temp_data_t = t*0.127;
	unsigned char temp_data_fp = ((int16_t)Pitch+180)*0.3556;
	unsigned char temp_data_fr = ((int16_t)Roll+180)*0.3556;
	unsigned char temp_data_fy = ((int16_t)Yaw+180)*0.3556;
	for(uint8_t i=0;i<128;i++)
	{
		if(i<=temp_data_fp)
			show_dt_[0][i] |= (1<<0);
		else
			show_dt_[0][i] &= ~(1<<0);
		if(i<=temp_data_fr)
			show_dt_[0][i] |= (1<<1);
		else
			show_dt_[0][i] &= ~(1<<1);
		if(i<=temp_data_fy)
			show_dt_[0][i] |= (1<<2);
		else
			show_dt_[0][i] &= ~(1<<2);
		
		if(i<=temp_data_p)
			show_dt_[0][i] |= (1<<4);
		else
			show_dt_[0][i] &= ~(1<<4);
		
		if(i<=temp_data_r)
			show_dt_[0][i] |= (1<<5);
		else
			show_dt_[0][i] &= ~(1<<5);
		
		if(i<=temp_data_y)
			show_dt_[0][i] |= (1<<6);
		else
			show_dt_[0][i] &= ~(1<<6);
		
		if(i<=temp_data_t)
			show_dt_[0][i] |= (1<<7);
		else
			show_dt_[0][i] &= ~(1<<7);
	}
	oled_show_image(0		,7	,&show_dt);
}
/* 菜单显示(不在菜单内时接收数据显示) */
void menu_display(void)
{
	if(app_data.is_lost)//如果已经丢控
	{
		oled_show_image(0		,0	,&singal_null);				//信号  							singal_full  singal_null
		oled_show_image(104	,0	,&bat_00);						//航模电池   					bat_00 bat_01 bat_02 bat_03 bat_04 bat_05
		oled_printf(110	,0,"00");//航模电量
		if(menu_now==NULL)//在菜单里的时候不更新主界面的东西
		{
			oled_show_image(29	,3	,&data_img_lock);			//锁定状态   					data_img_lock  data_img_unlock		
			//丢控了就显示本机配置的设置
			if(menu_setting.data1 & SET_HODE_ALT)//定高模式  手动模式
				oled_show_image(1		,3	,&hold_mode_data);		//手动、定高模式   		hand_mode_data  hold_mode_data
			else
				oled_show_image(1		,3	,&hand_mode_data);		//手动、定高模式   		hand_mode_data  hold_mode_data
			if(menu_setting.data1 & SET_HODE_HEADER)//无头模式  有头模式
				oled_show_image(109	,3	,&uheard_mode_data);	//无头模式  有头模式  uheard_mode_data   heard_mode_data
			else
				oled_show_image(109	,3	,&heard_mode_data);	//无头模式  有头模式  uheard_mode_data   heard_mode_data
		}
		Roll = Pitch = Yaw = 0;
	}
	else
	{
		if(app_data.rx_dr)//收到一次数据
		{
			int16_t temp[4];
			temp[0] = (int16_t)gSensorNRF.RX_BUF[0]|((int16_t)gSensorNRF.RX_BUF[1]<<8);//pitch*100
			temp[1] = (int16_t)gSensorNRF.RX_BUF[2]|((int16_t)gSensorNRF.RX_BUF[3]<<8);//roll*100
			temp[2] = (int16_t)gSensorNRF.RX_BUF[4]|((int16_t)gSensorNRF.RX_BUF[5]<<8);//yaw*100
			Pitch = temp[0]/100;
			Roll = temp[1]/100;
			Yaw = temp[2]/100;
			//RX_BUF[6]:0:lock   1:mag
			temp[3] = (int16_t)gSensorNRF.RX_BUF[7]|((int16_t)gSensorNRF.RX_BUF[8]<<8);
			float power_now = ((float)temp[3]/100.0f-3.2)/(4.2-3.2)*100;
			power_now = power_now>99?99:power_now;
			power_now = power_now<0?0:power_now;
			oled_printf(110	,0,"%2.0f",power_now);//航模电量
			oled_show_image(104	,0	,get_bat_icon((float)temp[3]/100.0f,1));//航模电量图标   					bat_00 bat_01 bat_02 bat_03 bat_04 bat_05
			oled_show_image(0		,0	,&singal_full);				//信号  							singal_full  singal_null
			if(menu_now==NULL)//在菜单里的时候不更新主界面的东西
			{
				if(gSensorNRF.RX_BUF[6] & 0x01)
					oled_show_image(29	,3	,&data_img_unlock);			//锁定状态   					data_img_lock  data_img_unlock
				else
					oled_show_image(29	,3	,&data_img_lock);			//锁定状态   					data_img_lock  data_img_unlock				
				
				if(gSensorNRF.RX_BUF[9] & 0x01)//定高模式  手动模式
					oled_show_image(1		,3	,&hold_mode_data);		//手动、定高模式   		hand_mode_data  hold_mode_data
				else
					oled_show_image(1		,3	,&hand_mode_data);		//手动、定高模式   		hand_mode_data  hold_mode_data
				
				if(gSensorNRF.RX_BUF[9] & 0x02)//无头模式  有头模式
					oled_show_image(109	,3	,&uheard_mode_data);	//无头模式  有头模式  uheard_mode_data   heard_mode_data
				else
					oled_show_image(109	,3	,&heard_mode_data);	//无头模式  有头模式  uheard_mode_data   heard_mode_data
			}
			app_data.rx_dr = 0;
		}
	}

	//电量处理显示
	float power_now = (power_value-3.7)/(4.2-3.7)*100;
	power_now = power_now>99?99:power_now;
	power_now = power_now<0?0:power_now;
	oled_printf(90	,0,"%2.0f",power_now);//遥控电量
	oled_show_image(84	,0	,get_bat_icon(power_value,0));//遥控电量图标  					bat_00 bat_01 bat_02 bat_03 bat_04 bat_05
	show_yg_data();

	//菜单显示
	if(menu_now!=NULL)
	{
		uint8_t start_point = 0;
		uint8_t end_point = 0;
		if(menu_now->now < 3)
			start_point = 0;
		else
			start_point = menu_now->now-2;

		end_point = start_point+3;
		if(end_point>=menu_now->menu_len)end_point=menu_now->menu_len;
		
		for(uint8_t i=start_point;i<end_point;i++)
		{
			//显示左侧箭头
			if(i==menu_now->now)
				oled_show_image(23,1+(i-start_point)*2,&menu_pointer);
			else
				oled_show_image(23,1+(i-start_point)*2,&menu_clear_pointer);
			//显示中间菜单
			oled_show_image(39,1+(i-start_point)*2,menu_now->data[i].data);
			//显示右侧选择对号
			if(menu_now->data[i].selected)
				oled_show_image(91,1+(i-start_point)*2,&menu_select);
			else
				oled_show_image(91,1+(i-start_point)*2,&menu_unselect);
		}
	}
}
/* 菜单确认处理 */
void menu_handle(void)
{
	if(menu_now == &fly_mode)//飞行模式
	{
		for(uint8_t i=0;i<menu_now->menu_len;i++)
		{
			if(i == menu_now->now)
				menu_now->data[i].selected = 1;
			else
				menu_now->data[i].selected = 0;
		}
		
	}
	else if(menu_now == &ctrl_mode)//控制模式
	{
		for(uint8_t i=0;i<menu_now->menu_len;i++)
		{
			if(i == menu_now->now)
				menu_now->data[i].selected = 1;
			else
				menu_now->data[i].selected = 0;
		}
	}
	else if(menu_now == &ctrl_rate)//控制速率
	{
		for(uint8_t i=0;i<menu_now->menu_len;i++)
		{
			if(i == menu_now->now)
				menu_now->data[i].selected = 1;
			else
				menu_now->data[i].selected = 0;
		}
	}
	else if(menu_now == &function)//功能
	{
		menu_now->data[menu_now->now].selected = !menu_now->data[menu_now->now].selected;
	}
	else if(menu_now == &calibration)//校准
	{
		switch(menu_now->now)
		{
			case 0:
				menu.select_run = 1;//加速度计校准
				break;
			case 1:
				menu.select_run = 2;//陀螺仪校准
				break;
			case 2:
				menu.select_run = 4;//摇杆校准
				break;
		}
	}
}
/* 菜单主循环 */
void menu_loop(void)
{
	if(menu_now!=NULL)
	{
		if(YG_DATA.dat.pitch-1500>200)//选择下一个
		{
			vTaskDelay(pdMS_TO_TICKS(2));
			while(YG_DATA.dat.pitch-1500>200)
				vTaskDelay(pdMS_TO_TICKS(2));
			menu_now->now ++;
			if(menu_now->now > menu_now->menu_len-1)
				menu_now->now=0;
		}
		else if(YG_DATA.dat.pitch-1500<-200)//选择上一个菜单
		{
			vTaskDelay(pdMS_TO_TICKS(2));
			while(YG_DATA.dat.pitch-1500<-200)
				vTaskDelay(pdMS_TO_TICKS(2));
			menu_now->now --;
			if(menu_now->now < 0)
				menu_now->now	=	menu_now->menu_len-1;
		}
		if(KEY1==KEY_DOWN)//返回上一层菜单
		{
			vTaskDelay(pdMS_TO_TICKS(2));
			while(KEY1==KEY_DOWN)
				vTaskDelay(pdMS_TO_TICKS(2));
			if(menu_now->previous == NULL)//已经没有上一层了 退出
			{
				save_setting();
				menu_now =NULL;
				oled_clear();
				oled_printf(12	,0,"X-Mini V0.1");
			}
			else
			{
				menu_now = menu_now->previous;
			}
		}
	}

	if(KEY2==KEY_DOWN)//进入菜单
	{
		vTaskDelay(pdMS_TO_TICKS(2));
		while(KEY2==KEY_DOWN)vTaskDelay(pdMS_TO_TICKS(2));
		oled_clear();
		oled_printf(12	,0,"X-Mini V0.1");
		if(menu_now == NULL)//进入第一层菜单
		{
			load_setting();
			menu_now = &menu;
			menu_now->now = 0;
		}
		else
		{
			if(menu_now->data[menu_now->now].next_menu!=NULL)//还有下一级菜单
			{
				menu_now = menu_now->data[menu_now->now].next_menu;
			}
			else//没有下一级菜单了  应该执行对应的功能
			{
				menu_handle();
			}
		}
	}
	menu_display();
}

