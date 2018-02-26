#ifndef __BF_STATUS_MINI_SYSTEM_H__
#define __BF_STATUS_MINI_SYSTEM_H__
#include "stm32f10x.h"
#include "mpu6050.h"
#include "led.h"
#include "imu.h"
#include "debug_usart.h"
#include "motor.h"
#include <stdio.h>
#include "stm32fx_delay.h"
#include "nrf24l01.h"
#include "stdbool.h"
void system_init(void);
void Jump_To_IAP (void);
#endif
