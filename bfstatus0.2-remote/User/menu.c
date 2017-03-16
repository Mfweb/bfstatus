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
uint8_t Cal_Sel = 0;
extern uint8_t TX_BUF[16];
extern uint8_t RX_BUF[16];
uint8_t menu_buff[8][22]={0x00};
uint8_t cal_text[4][6]=
{
{"  ACC"},
{" GYRO"},
{"  MAG"},
{"  Joy"}
};
int8_t  select = 0;
void menu_init(void)
{
	sprintf((char *)menu_buff[0]," BF State Mini V0.1 " );
	sprintf((char *)menu_buff[1],"P:        R:         ");
	sprintf((char *)menu_buff[2],"UnArmed NoMag Y:     ");
	sprintf((char *)menu_buff[3],"P:        R:         ");
	sprintf((char *)menu_buff[4],"Y:        T:         ");
	sprintf((char *)menu_buff[5],"    0.00V  0.00V     ");
	sprintf((char *)menu_buff[6],"Calibration:  ACC    ");
	sprintf((char *)menu_buff[7],"                     ");
}

void menu_display(void)
{
	int16_t temp[4];
	uint8_t i;
	float p,r,y,t;
	temp[0] = (int16_t)RX_BUF[0]|((int16_t)RX_BUF[1]<<8);
	temp[1] = (int16_t)RX_BUF[2]|((int16_t)RX_BUF[3]<<8);
	temp[2] = (int16_t)RX_BUF[4]|((int16_t)RX_BUF[5]<<8);
	temp[3] = (int16_t)RX_BUF[7]|((int16_t)RX_BUF[8]<<8);
	
	/* 显示姿态 */
	sprintf((char *)menu_buff[1],"P:%7.2f R:%7.2f  ",(float)temp[0]/100.0f,(float)temp[1]/100.0f);
	/* 显示是否解锁、是否使用罗盘、偏航 */
	if(RX_BUF[6] & 0x01)
		strcpy((char *)menu_buff[2],"Unlock ");
	else
		strcpy((char *)menu_buff[2],"Locked ");
	if(RX_BUF[6] & 0x02)
		strcpy((char *)menu_buff[2]+7,"Mag  ");
	else
		strcpy((char *)menu_buff[2]+7,"NoMag");
	
	sprintf((char *)menu_buff[2]+12,"  %3.0f",(float)temp[2]/100.0f);
	//sprintf((char *)menu_buff[2]+12,"  %3.0f",(float)temp[2]/100.0f);
	/* 显示遥控的摇杆状态 */
	p = 40.0f* ScaleLinear((float)(YG_DATA.dat.pitch-1500),500.0f,50.0f);
	r = 40.0f* ScaleLinear((float)(YG_DATA.dat.roll	-1500),500.0f,50.0f);
	y = 180.0f/3.1415926f * ScaleLinear((float)(YG_DATA.dat.yaw-1500),500.0f,70.0f);
	t = (float)(YG_DATA.dat.throttle-1000);
	sprintf((char *)menu_buff[3],"P:%7.2f R:%7.2f  ",p,r);
	sprintf((char *)menu_buff[4],"Y:%7.2f T:%4.0f  ",y,t);
	/*显示电压状态*/
	sprintf((char *)menu_buff[5],"    %04.2fV %04.2fV     ",power_value,(float)temp[3]/100.0f);
	/*显示当前选中的标定功能*/
	strcpy((char *)menu_buff[6]+12,(char *)cal_text[select]);
	for(i=0;i<8;i++)
		oled_write_english_string(0,i,(char *)menu_buff[i]);
}

void menu_loop(void)
{
	if(KEY1==KEY_DOWN)
	{
		DelayMs(1);
		while(KEY1==KEY_DOWN);
		select ++;
		if(select>3)select=0;
	}
	if(KEY2==KEY_DOWN)
	{
		DelayMs(1);
		while(KEY2==KEY_DOWN);
		Cal_Sel = select+1;
	}
	menu_display();
}

