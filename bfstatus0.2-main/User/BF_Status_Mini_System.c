/**
  ******************************************************************************
  * @file    BF_Status_Mini_System.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini 系统文件
  ******************************************************************************
  */
#include "BF_Status_Mini_System.h"
#include "eeprom.h"
#include "ms5611.h"
#include "battery.h"
#include "hw_config.h"

//系统初始化
void system_init(void)
{
	//uint8_t i;
	usart_init();
	motor_init();
	led_init();
	I2C_INIT();
	battery_init();
	#ifdef __USB_ENABLE__
	USB_Config();
	#endif
	DelayMs(500);
//  printf("AT\r\n");
//	printf("AT+NAMEBF_State_Mini\r\n");
//	DelayMs(1000);
//	printf("AT+PIN000000\r\n");
//	DelayMs(1000);
//	printf("AT+TYPE2\r\n");
//	DelayMs(1000);
//	printf("AT+BAUD8\r\n");
//	DelayMs(1000);
//	for(i=0;i<254;i++)
//	{
		//printf("%02x\r\n",Single_Read(0x3c,0x15));
//		DelayMs(10);
//	}
	Flag_Clear();
	if(NRF_Init()==ERROR)
	{
		printf("nRF Error\n");
		while(1)
		{
			DelayMs(200);
			LED2=!LED2;
		}
	}
	else
		printf("nRF Init OK\n");
	DelayMs(20);
	if(mpu_init()==FALSE)
	{
		printf("MPU Error\n");
		while(1)
		{
			DelayMs(200);
			LED1=!LED1;
		}
	}
	else
		printf("MPU Init OK\n");
	DelayMs(20);
	if(hmc_init()==FALSE)
	{
		printf("HMC Error\n");
		flag.IsMAG = 0;//罗盘有问题 关闭
	}
	else
		printf("HMC Init OK\n");

	DelayMs(20);
	if(flag.IsBaro)
		ms5611_reset();
	param_init();
	
	start_led();
	time_init();
}

//定时器初始化
void time_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period=2000;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//时间片
void Time_slice(void)
{
	static uint8_t tick[3]={0,0,0};
	tick[0]++;
	tick[1]++;
	tick[2]++;

	if(tick[0]>=2)
	{
		tick[0] = 0;
		flag.f250hz=1;
	}
	if(tick[1]>=5)
	{
		tick[1] = 0;
		flag.f100hz=1;
	}	
	if(tick[2] >= 50)
	{
		tick[2] = 0;
		flag.f10hz=1;
	}
}

/*清空标志*/
void Flag_Clear(void)
{
	flag.Lock							= 1;
	flag.battery_alarm		= 0;
	flag.CalibratingACC		= 0;
	flag.CalibratingGYR		= 0;
	flag.CalibratingMAG		= 0;
	flag.f100hz						= 0;
	flag.f10hz						= 0;
	flag.f250hz						= 0;
	flag.IsBaro						= 0;
	flag.IsMAG						= 0;
	flag.LockYaw					= 0;
	flag.need_back				= 0;
	flag.hold_altitude  	= 0;
	flag.flip							= 0;
	flag.zerog						= 0;
	flag.zero_pull				= 1;
	ctrl.ctrlRate 				= 0;
	sensor_ms.get_count  	= 10;
}

//Roll和Pitch的参数
#define SHELL_KP 3.5		//4			3.5
#define SHELL_KI 0.00		//0.02	0.01

#define CORE_KP 1.4			//1.4		0.7
#define CORE_KI 0.00		//0.07	0.5
#define CORE_KD 0.35		//0.03	0.03
/*载入参数*/
void param_init(void)
{
	ctrl.pitch.shell.kp = SHELL_KP;
	ctrl.pitch.shell.ki = SHELL_KI;
	
	ctrl.roll.shell.kp = SHELL_KP;
	ctrl.roll.shell.ki = SHELL_KI;	
	
	ctrl.pitch.core.kp = CORE_KP;
	ctrl.pitch.core.ki = CORE_KI;
	ctrl.pitch.core.kd = CORE_KD;

	ctrl.roll.core.kp = CORE_KP;
	ctrl.roll.core.ki = CORE_KI;
	ctrl.roll.core.kd = CORE_KD;
	
	//YAW参数
	ctrl.yaw.shell.kp = 5.0;	//10
	ctrl.yaw.shell.kd = 0.0;	//0.01
	
	ctrl.yaw.core.kp = 1.8;		//2.1
	ctrl.yaw.core.ki = 0.0;		//0
	ctrl.yaw.core.kd = 0.1;		//0.2
	
	//Roll和Pitch积分限幅
	ctrl.pitch.shell.increment_max = 50;
	ctrl.roll.shell.increment_max = 50;
	
	//气压定高参数
	ctrl.alt.kp = 40.0f;
	ctrl.alt.ki = 1.0;
	ctrl.alt.kd = 0.0;
	ctrl.alt.increment_max = 700;//高度PID积分限幅
	
	ctrl.ctrlRate = 0;
	ctrl.ctrlRate_alt = 0;

	data_load();//读取EEPROM中的参数
}

/* 读取静态量 */
void data_load(void)
{
	uint8_t temp[22]={0x00};
	uint8_t i,null=1;
	ee_ReadBytes(temp,0x00,22);
	for(i=0;i<10;i++)//检查EEPROM是否为空
	{
		if(temp[i]!=0xff)
		{
			null = 0;
			break;
		}
	}
	if(!null)
	{
		sensor_mpu.acc.quiet.x = (int16_t)temp[0]|(((int16_t)temp[1])<<8);
		sensor_mpu.acc.quiet.y = (int16_t)temp[2]|(((int16_t)temp[3])<<8);
		
		sensor_mpu.gyr.quiet.x = (int16_t)temp[4]|(((int16_t)temp[5])<<8);
		sensor_mpu.gyr.quiet.y = (int16_t)temp[6]|(((int16_t)temp[7])<<8);
		sensor_mpu.gyr.quiet.z = (int16_t)temp[8]|(((int16_t)temp[9])<<8);
		
		sensor_mag.mag_limt[0] = (int16_t)temp[10]|(((int16_t)temp[11])<<8);
		sensor_mag.mag_limt[1] = (int16_t)temp[12]|(((int16_t)temp[13])<<8);
		sensor_mag.mag_limt[2] = (int16_t)temp[14]|(((int16_t)temp[15])<<8);
		sensor_mag.mag_limt[3] = (int16_t)temp[16]|(((int16_t)temp[17])<<8);
		sensor_mag.mag_limt[4] = (int16_t)temp[18]|(((int16_t)temp[19])<<8);
		sensor_mag.mag_limt[5] = (int16_t)temp[20]|(((int16_t)temp[21])<<8);
		printf("load:%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\r\n",
		sensor_mpu.acc.quiet.x,sensor_mpu.acc.quiet.y,sensor_mpu.gyr.quiet.x,sensor_mpu.gyr.quiet.y,sensor_mpu.gyr.quiet.z,
		sensor_mag.mag_limt[0],sensor_mag.mag_limt[1],sensor_mag.mag_limt[2],sensor_mag.mag_limt[3],sensor_mag.mag_limt[4],sensor_mag.mag_limt[5]);
	}
}
/*保存静态量*/
void data_save(void)
{
	uint8_t temp[22]={0x00};
	temp[0] = (uint8_t)sensor_mpu.acc.quiet.x;
	temp[1] = (uint8_t)(sensor_mpu.acc.quiet.x>>8);
	temp[2] = (uint8_t)sensor_mpu.acc.quiet.y;
	temp[3] = (uint8_t)(sensor_mpu.acc.quiet.y>>8);
	
	temp[4] = (uint8_t)sensor_mpu.gyr.quiet.x;
	temp[5] = (uint8_t)(sensor_mpu.gyr.quiet.x>>8);
	temp[6] = (uint8_t)sensor_mpu.gyr.quiet.y;
	temp[7] = (uint8_t)(sensor_mpu.gyr.quiet.y>>8);	
	temp[8] = (uint8_t)sensor_mpu.gyr.quiet.z;
	temp[9] = (uint8_t)(sensor_mpu.gyr.quiet.z>>8);
	
	temp[10] = (uint8_t)sensor_mag.mag_limt[0];
	temp[11] = (uint8_t)(sensor_mag.mag_limt[0]>>8);
	temp[12] = (uint8_t)sensor_mag.mag_limt[1];
	temp[13] = (uint8_t)(sensor_mag.mag_limt[1]>>8);
	temp[14] = (uint8_t)sensor_mag.mag_limt[2];
	temp[15] = (uint8_t)(sensor_mag.mag_limt[2]>>8);
	temp[16] = (uint8_t)sensor_mag.mag_limt[3];
	temp[17] = (uint8_t)(sensor_mag.mag_limt[3]>>8);
	temp[18] = (uint8_t)sensor_mag.mag_limt[4];
	temp[19] = (uint8_t)(sensor_mag.mag_limt[4]>>8);
	temp[20] = (uint8_t)sensor_mag.mag_limt[5];
	temp[21] = (uint8_t)(sensor_mag.mag_limt[5]>>8);

	ee_WriteBytes(temp,0x00,22);
	printf("save:%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\r\n",
	sensor_mpu.acc.quiet.x,sensor_mpu.acc.quiet.y,sensor_mpu.gyr.quiet.x,sensor_mpu.gyr.quiet.y,sensor_mpu.gyr.quiet.z,
	sensor_mag.mag_limt[0],sensor_mag.mag_limt[1],sensor_mag.mag_limt[2],sensor_mag.mag_limt[3],sensor_mag.mag_limt[4],sensor_mag.mag_limt[5]);
}
