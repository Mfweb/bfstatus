#ifndef __HMC5883L_H__
#define __HMC5883L_H__
#include "stm32f10x_i2c.h"
#define MAG_ADDRESS 0x3c
#define MAG_DATA_REGISTER 0x03

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

/*  罗盘  */
typedef struct
{
	struct d_float origin;
	int16_t mag_limt[6];
}_sensor_data_mag;
extern _sensor_data_mag sensor_mag;	//罗盘数据

uint8_t hmc_init(void);
void HMC5883L_Read(void);
void Mag_Calibration(int16_t *array);
#endif
