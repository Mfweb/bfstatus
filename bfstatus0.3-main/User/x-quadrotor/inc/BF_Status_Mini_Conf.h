/**
  ******************************************************************************
  * @file    BF_Status_Mini_Conf.h
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 系统配置
  ******************************************************************************
  */
#ifndef __BF_STATUS_MINI_CONF_H__
#define __BF_STATUS_MINI_CONF_H__

//电机死区
#define MOTOT_DIE1 10
#define MOTOT_DIE2 10
#define MOTOT_DIE3 10
#define MOTOT_DIE4 10
//油门限幅
#define TH_MIN_CHECK   50
#define TH_MAX_CHECK   900
//自动锁定时间
#define AUTO_LOCK_TIME	3000
//丢控锁定时间
#define LOST_CTRL_TIME	1000
//0G状态阈值
#define ZERO_MAX 100
//地址配置
#define IAP_ADDRESS 0x08000000
//配置信息存储地址
#define FLASH_BASE_ADDR 0x800F400
//丢控关机
#define LOST_POWER_OFF
//插入USB则跳转BOOTLOADER
//#define USB_GOTO_IAP

#endif
