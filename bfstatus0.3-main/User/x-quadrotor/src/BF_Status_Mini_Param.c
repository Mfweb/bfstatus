#include "BF_Status_Mini_Param.h"
#include "BF_Status_Mini_Global.h"
#include "BF_Status_Mini_Ctrl.h"
#include "stm32f10x_flash.h"
void Flag_Clear(void);
void param_init(void);
void data_load(void);
void data_save(void);
param_t gParam={
	.Init = &param_init,
	.Save = &data_save,
	.Load = &data_load,
	.ClearFlag = Flag_Clear
};
//PID参数
#define SHELL_KP			5		//4			3.5   5
#define SHELL_KI			3		//0.02	0.01  3
#define YAW_SHELL_KP 	10

#define CORE_KP				1.4			//1.4		0.7   1.7
#define CORE_KI				0.00		//0.07	0.5
#define CORE_KD				0.02		//0.03	0.03  0.35
#define YAW_CORE_KP		2.1
#define YAW_CORE_KI		0.0
#define YAW_CORE_KD		0.02

#define ALT_KP				21
#define ALT_KI				0.5
#define ALT_KD				60

/**
 * @brief  标志位复位
 * @code
 *      Flag_Clear();
 * @endcode
 * @param  void
 * @retval void
 */
void Flag_Clear(void)
{
	flag.Lock						= true;
	flag.BatteryAlarm		= false;
	flag.CalibratingACC	= 0;
	flag.CalibratingGYR	= 0;
	flag.CalibratingVel	= 0;
	flag.CalibratingMAG	= 0;
	flag.IsBaro					= true;
	flag.IsMAG					= false;
	flag.LockYaw				= false;
	flag.HoldAltitude		= false;
	flag.Flipping				= 0;
	flag.ZeroGravity		= false;
	flag.Throwing				= 0;
	flag.PowerOff				= false;
	flag.iic_sem				= xSemaphoreCreateMutex();
	flag.spi_sem				= xSemaphoreCreateMutex();
	gCtrl.ctrlRate			= 0;
}

/**
 * @brief  载入参数（从flash）
 * @code
 *      param_init();
 * @endcode
 * @param  void
 * @retval void
 */
void param_init(void)
{
	gCtrl.pitch.shell.kp = SHELL_KP;
	gCtrl.pitch.shell.ki = SHELL_KI;
	gCtrl.pitch.shell.kd = 0;
	
	gCtrl.roll.shell.kp = SHELL_KP;
	gCtrl.roll.shell.ki = SHELL_KI;
	gCtrl.roll.shell.kd = 0;
	
	gCtrl.pitch.core.kp = CORE_KP;
	gCtrl.pitch.core.ki = CORE_KI;
	gCtrl.pitch.core.kd = CORE_KD;

	gCtrl.roll.core.kp = CORE_KP;
	gCtrl.roll.core.ki = CORE_KI;
	gCtrl.roll.core.kd = CORE_KD;
	
	//YAW参数
	gCtrl.yaw.shell.kp = YAW_SHELL_KP;
	gCtrl.yaw.shell.ki = 0;
	gCtrl.yaw.shell.kd = 0;
	
	gCtrl.yaw.core.kp = YAW_CORE_KP;
	gCtrl.yaw.core.ki = YAW_CORE_KI;
	gCtrl.yaw.core.kd = YAW_CORE_KD;
	
	//Roll和Pitch积分限幅
	gCtrl.pitch.shell.increment_max = 100;
	gCtrl.roll.shell.increment_max = 100;
	gCtrl.pitch.core.increment_max = 33;
	gCtrl.roll.core.increment_max = 33;
	gCtrl.yaw.core.increment_max = 360;
	
	//油门PID控制
	gCtrl.throttle_rate.kp = 1.5;
	gCtrl.throttle_rate.ki = 1.;
	gCtrl.throttle_rate.kd = 0.;
	gCtrl.throttle_rate.increment_max = 250;
	//气压定高参数
	gCtrl.alt.kp = ALT_KP;
	gCtrl.alt.ki = ALT_KI;
	gCtrl.alt.kd = ALT_KD;
	
	gCtrl.alt.increment_max = 20;//高度PID积分限幅
	
	gCtrl.ctrlRate = 0;

	data_load();//读取参数
}


/**
 * @brief  从flash载入参数
 * @code
 *      data_load();
 *      存储区域 0x800F400 - 0x8010000 = 3kByte
 * @endcode
 * @param  void
 * @retval void
 */
void data_load(void)
{
	gSensorMpu.acc.quiet.x = (int16_t)FLASH_ReadHalfWord(FLASH_BASE_ADDR + 0x00);
	gSensorMpu.acc.quiet.y = (int16_t)FLASH_ReadHalfWord(FLASH_BASE_ADDR + 0x02);
	gSensorMpu.acc.quiet.z = (int16_t)FLASH_ReadHalfWord(FLASH_BASE_ADDR + 0x04);

	gSensorMpu.gyr.quiet.x = (int16_t)FLASH_ReadHalfWord(FLASH_BASE_ADDR + 0x06);
	gSensorMpu.gyr.quiet.y = (int16_t)FLASH_ReadHalfWord(FLASH_BASE_ADDR + 0x08);
	gSensorMpu.gyr.quiet.z = (int16_t)FLASH_ReadHalfWord(FLASH_BASE_ADDR + 0x0A);

	gSelfState.velocityZ.base_acc_z = (float)((int32_t)FLASH_ReadWord(FLASH_BASE_ADDR + 0x0C))/1000.f;
}


/**
 * @brief  保存参数到flash
 * @code
 *      data_save();
 * @endcode
 * @param  void
 * @retval void
 */
void data_save(void)
{
	int32_t temp_vz = (int32_t)(gSelfState.velocityZ.base_acc_z * 1000);
	FLASH_Unlock();
	FLASH_ErasePage(FLASH_BASE_ADDR + 0x000);//擦除前1page（1K）
	FLASH_ProgramHalfWord(FLASH_BASE_ADDR + 0x00,(uint16_t)gSensorMpu.acc.quiet.x);
	FLASH_ProgramHalfWord(FLASH_BASE_ADDR + 0x02,(uint16_t)gSensorMpu.acc.quiet.y);
	FLASH_ProgramHalfWord(FLASH_BASE_ADDR + 0x04,(uint16_t)gSensorMpu.acc.quiet.z);
	
	FLASH_ProgramHalfWord(FLASH_BASE_ADDR + 0x06,(uint16_t)gSensorMpu.gyr.quiet.x);
	FLASH_ProgramHalfWord(FLASH_BASE_ADDR + 0x08,(uint16_t)gSensorMpu.gyr.quiet.y);
	FLASH_ProgramHalfWord(FLASH_BASE_ADDR + 0x0A,(uint16_t)gSensorMpu.gyr.quiet.z);
	
	FLASH_ProgramWord(FLASH_BASE_ADDR + 0x0C,(uint32_t)temp_vz);
	FLASH_Lock();
//	temp[12] = (uint8_t)sensor_mag.mag_limt[0];
//	temp[13] = (uint8_t)(sensor_mag.mag_limt[0]>>8);
//	temp[14] = (uint8_t)sensor_mag.mag_limt[1];
//	temp[15] = (uint8_t)(sensor_mag.mag_limt[1]>>8);
//	temp[16] = (uint8_t)sensor_mag.mag_limt[2];
//	temp[17] = (uint8_t)(sensor_mag.mag_limt[2]>>8);
//	temp[18] = (uint8_t)sensor_mag.mag_limt[3];
//	temp[19] = (uint8_t)(sensor_mag.mag_limt[3]>>8);
//	temp[20] = (uint8_t)sensor_mag.mag_limt[4];
//	temp[21] = (uint8_t)(sensor_mag.mag_limt[4]>>8);
//	temp[22] = (uint8_t)sensor_mag.mag_limt[5];
//	temp[23] = (uint8_t)(sensor_mag.mag_limt[5]>>8);
	

	//ee_WriteBytes(temp,0x00,28);
//	printf("save:%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\r\n",
//	gSensorMpu.acc.quiet.x,gSensorMpu.acc.quiet.y,gSensorMpu.gyr.quiet.x,gSensorMpu.gyr.quiet.y,gSensorMpu.gyr.quiet.z,
//	sensor_mag.mag_limt[0],sensor_mag.mag_limt[1],sensor_mag.mag_limt[2],sensor_mag.mag_limt[3],sensor_mag.mag_limt[4],sensor_mag.mag_limt[5]);
}
