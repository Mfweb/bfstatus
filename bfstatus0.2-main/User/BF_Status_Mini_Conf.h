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
#define MOTOT_DIE 15
//油门限幅
#define TH_MIN_CHECK   100
#define TH_MAX_CHECK   950
//自动锁定时间
#define AUTO_LOCK_TIME	3000
//丢控锁定时间
#define LOST_CTRL_TIME	1000
//无头模式
#define HODE_HEADER
//无线发送数据
#define RC_SEND_DATA
//0G状态阈值
#define ZERO_MAX 300
#endif
