#include "bmp280.h"
#include "stm32f10x_i2c.h"
#include <stdio.h>
#include <math.h>
#include "BF_Status_Mini_Filter.h"
uint8_t bmp280_init(void);
void bmp280GetData(void);

typedef struct 
{
    uint16_t dig_T1;/* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1;/* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
    int32_t t_fine; /* calibration t_fine data */
}bmp_;



typedef struct
{
	uint8_t bmp280ID;
	bool isInit;
	bool isGroundSet;
	uint8_t GroundCount;
	int32_t bmp280RawPressure;
	int32_t bmp280RawTemperature;
}bmp_dt;

bmp_ 		bmp280_cal_data;
bmp_dt	bmp280_data={0,false,false,10,0,0};
sensor_data_bmp_t	gSensorBmp={
.Init = &bmp280_init,
.Read = &bmp280GetData};
uint8_t bmp280_init(void)
{
	uint8_t ack;
	Single_Read(BMP280_ADDRESS, BMP280_CHIP_ID,&ack);
	if(ack != 0x58)
		return FALSE;
	//读取校准数据
	I2C_Read(BMP280_ADDRESS,BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG,24,(uint8_t *)&bmp280_cal_data);
	//设置工作模式
	Single_Write(BMP280_ADDRESS,BMP280_CTRL_MEAS_REG, BMP280_OVERSAMP_2X << 2 | BMP280_OVERSAMP_16X << 5 | BMP280_NORMAL_MODE);	
	//配置IIR滤波
	Single_Write(BMP280_ADDRESS,BMP280_CONFIG_REG, 5<<2);	
	printf("BMP Calibrate:%u %d %d %d %d %d %d %d %d %u %d %d %d\n",
	bmp280_cal_data.dig_P1,bmp280_cal_data.dig_P2,
	bmp280_cal_data.dig_P3,bmp280_cal_data.dig_P4,
	bmp280_cal_data.dig_P5,bmp280_cal_data.dig_P6,
	bmp280_cal_data.dig_P7,bmp280_cal_data.dig_P8,
	bmp280_cal_data.dig_P9,
	bmp280_cal_data.dig_T1,bmp280_cal_data.dig_T2,
	bmp280_cal_data.dig_T3,
	bmp280_cal_data.t_fine);
	bmp280_data.bmp280ID = ack;
	bmp280_data.isInit = true;
	gSensorBmp.Pressure = 1;
	gSensorBmp.NowAltitude = 0;
	gSensorBmp.ctrl_rate = 20;
	return TRUE;
}

static void bmp280GetPressure(void)
{
	uint8_t data[BMP280_DATA_FRAME_SIZE];
	// read data from sensor
	I2C_Read(BMP280_ADDRESS, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
	bmp280_data.bmp280RawPressure = (s32)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
	bmp280_data.bmp280RawTemperature = (s32)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
}

static int32_t bmp280CompensateT(int32_t adcT)
{
	int32_t var1, var2, T;
	var1 = ((((adcT >> 3) - ((s32)bmp280_cal_data.dig_T1 << 1))) * ((s32)bmp280_cal_data.dig_T2)) >> 11;
	var2  = (((((adcT >> 4) - ((s32)bmp280_cal_data.dig_T1)) * ((adcT >> 4) - ((s32)bmp280_cal_data.dig_T1))) >> 12) * ((s32)bmp280_cal_data.dig_T3)) >> 14;
	bmp280_cal_data.t_fine = var1 + var2;
	T = (bmp280_cal_data.t_fine * 5 + 128) >> 8;
	return T;
}

static uint32_t bmp280CompensateP(int32_t adcP)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_cal_data.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal_data.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal_data.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal_data.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal_data.dig_P7) << 4);
    return (uint32_t)p;
}

#define CONST_PF 0.1902630958	//(1/5.25588f) Pressure factor
#define FIX_TEMP 25				// Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
								// TLDR: Adjusting for temp changes does more harm than good.
/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
static float bmp280PressureToAltitude(float* pressure)
{
    if (*pressure > 0)
    {
			//return 1015.7f*(1 + *temperature / 273) * log(*groundPressure / *pressure);
      return ((pow((1015.7f / *pressure), CONST_PF) - 1.0f) * (gSensorBmp.Temperature + 273.15f)) / 0.0065f;
    }
    else
    {
        return 0;
    }
}


void bmp280GetData(void)
{
	float alt;
	if(gSensorBmp.ctrl_rate++ >= 20)//40ms
	{
		gSensorBmp.ctrl_rate = 0;
		bmp280GetPressure();

		gSensorBmp.Temperature = bmp280CompensateT(bmp280_data.bmp280RawTemperature)/100.f;		
		gSensorBmp.Pressure		 = bmp280CompensateP(bmp280_data.bmp280RawPressure)/25600.f;		
		
		alt = bmp280PressureToAltitude(&gSensorBmp.Pressure);
		presssureFilter(&alt,&gSensorBmp.NowAltitude);//限幅平均滤波
		gSensorBmp.AbsAltitude = gSensorBmp.NowAltitude;
	}
}

