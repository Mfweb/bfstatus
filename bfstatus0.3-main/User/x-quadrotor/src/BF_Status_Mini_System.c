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
#include "BF_Status_Mini_Ctrl.h"
#include "BF_Status_Mini_Param.h"
#include "eeprom.h"
#include "battery.h"
#include "usb_lib.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "battery.h"
#include "task.h"
#include "bmp280.h"

void BSP_init(void)
{
	//usart_init();
	gMotor.Init();
	USB_Config();
	led_init();
	I2C_INIT();
	gBattery.Init();
}
void Sensor_init(void)
{
	if(gSensorNRF.Init()==ERROR)
	{
		printf("nRF Error\n");
		while(1)
		{
			DelayMs(200);
			LED2=!LED2;
		}
	}
	if(gSensorMpu.Init()==FALSE)
	{
		printf("MPU Error\n");
		while(1)
		{
			DelayMs(200);
			LED1=!LED1;
		}
	}
	if(flag.IsBaro)
	{
		if(gSensorBmp.Init() == FALSE)
		{
			flag.IsBaro = false;
			printf("BMP280 Error\n");
		}
	}
}
/**
 * @brief  系统初始化
 * @code
 *      system_init();
 * @endcode
 * @param  void
 * @retval void
 */
void system_init(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,(0x08003400+4));//重新配置中断向量表到APP
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//FreeRTOS必须设置所有优先级位为抢占式优先级
	BSP_init();
	gParam.Init();
	gParam.ClearFlag();
	Sensor_init();
	start_led();
}

/**
 * @brief  NVIC复位
 * @code
 *      NVIC_DeInit();
 * @endcode
 * @param  void
 * @retval void
 */
void NVIC_DeInit(void)
{
  uint32_t index = 0;
  
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0x000007FF;
  NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICPR[1] = 0x000007FF;
  
  for(index = 0; index < 0x0B; index++)
  {
     NVIC->IP[index] = 0x00000000;
  } 
}


/**
 * @brief  跳转到IAP引导
 * @code
 *      Jump_To_IAP();
 *			跳转到IAP引导位置，此后不再返回
 * @endcode
 * @param  void
 * @retval void
 */
void Jump_To_IAP (void)
{
	uint32_t SpInitVal;
	uint32_t JumpAddr;
	void (*pFun)(void);
	
	taskENTER_CRITICAL();
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, DISABLE);
	RCC_DeInit();
	TIM_DeInit(TIM2);
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	USART_DeInit(USART1);
	NVIC_DeInit();
	
	SpInitVal = *(uint32_t *)IAP_ADDRESS;
	JumpAddr = *(uint32_t *)(IAP_ADDRESS + 4);
	__set_MSP(SpInitVal);
	pFun = (void (*)(void))JumpAddr;
	(*pFun)();
}
