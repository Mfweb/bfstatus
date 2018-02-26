#include "stm32f10x.h"
#include "stm32fx_delay.h"
#include "hw_config.h"
#include <stdio.h>
#include "usb_core.h"
#include "usb_pwr.h"
#include <string.h>
#include "core_cm3.h"
#include "stm32f10x_flash.h"

#define USER_ADDRESS 0x08003400  //用户程序地址
#define SYS_ADDRESS 0x0800F800	//系统存储区
#define MODE_ADDRESS 0x0800FC00	//复位模式配置区

#define LED1 	PAout(4)
#define LED2 	PAout(8)
#define LED3 	PBout(4)
#define LED4 	PBout(5)
#define ON		0
#define OFF		!ON
extern void EXTI3_IRQHandler(void);
__asm void SystemReset(void)
{
 MOV R0, #1           //; 
 MSR FAULTMASK, R0    //; 清除FAULTMASK 禁止一切中断产生
 LDR R0, =0xE000ED0C  //;
 LDR R1, =0x05FA0004  //; 
 STR R1, [R0]         //; 系统软件复位   
 
deadloop
		B deadloop        //; 死循环使程序运行不到下面的代码
}

/*
boot:				0x8000000 - 0x8003400
user:				0x8003400 - 0x800F400
sys setting:0x800F400 - 0x8010000
41692
*/
void NVIC_DeInit(void)
{
  volatile uint32_t index = 0;
  
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0x000007FF;
  NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICPR[1] = 0x000007FF;
  
  for(index = 0; index < 0x0B; index++)
  {
     NVIC->IP[index] = 0x00000000;
  } 
}
//跳转到用户程序执行
void Jump_To_APP (void)
{
	uint32_t SpInitVal; //要跳转到程序的SP初值.
	uint32_t JumpAddr; 	//要跳转到程序的地址.即,要跳转到程序的入口
	void (*pFun)(void); //定义一个函数指针.用于指向APP程序入口
	//恢复复位设置，这些必须恢复，否则可能跳过去不能运行
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, DISABLE);
	RCC_DeInit();																	//关闭外设
	NVIC_DeInit();																//复位NVIC，禁用中断
	
	SpInitVal = *(uint32_t *)USER_ADDRESS;				//栈顶地址
	JumpAddr	= *(uint32_t *)(USER_ADDRESS + 4);	//中断向量地址
	__set_MSP(SpInitVal);													//设置SP为堆栈栈顶
	pFun = (void (*)(void))JumpAddr;							//将地址转换成跳转函数
	(*pFun)();																		//执行跳转函数，跳转到用户程序
	while(1);
}

//初始化LED
void led_init(void)
{
	GPIO_QuickInit(GPIOA,GPIO_Pin_4,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOA,GPIO_Pin_8,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_4,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_5,GPIO_Mode_Out_PP);
	LED1 = LED2 = LED3 = LED4 = ON;
}
//LED启动快闪
void led_disp(void)
{
	LED1 = LED2 = LED3 = LED4 = ON;
	DelayMs_Tick(100);
	LED1 = LED2 = LED3 = LED4 = OFF;
	DelayMs_Tick(300);
	LED1 = LED2 = LED3 = LED4 = ON;
	DelayMs_Tick(100);
	LED1 = LED2 = LED3 = LED4 = OFF;
}

uint8_t soft_update_buffer[1010]={'\0'};

	
//软件升级检测
void sf_update_check(void)
{
	volatile  uint16_t time_out = 0;
	volatile uint16_t read_len = 0;
	volatile uint32_t i;
	
	for(i=0;i<1010;i++)
		soft_update_buffer[i] = 0x00;
	read_len = USB_RxRead(soft_update_buffer, 20);
	if(read_len>0)
	{
		if(strcmp((const char*)soft_update_buffer,"x-q-update-start") == 0)//收到软件升级指令，准备进行升级操作
		{
			FLASH_Unlock();
			for(i=0;i<48;i++)//开始擦除flash  一共擦除48K的用户程序空间
			{
				FLASH_ErasePage((uint32_t)(USER_ADDRESS + 0x400*i));//每块1024byte
			}
			printf("s-q-update-start");//确认启动
			
			time_out = 20*10;//超时时间 20S
			uint32_t now_address = USER_ADDRESS;//写入的地址位置
			while(1)
			{
				for(i=0;i<1010;i++)//清空缓冲区
					soft_update_buffer[i] = 0x00;
				
				read_len = USB_RxRead(soft_update_buffer, 1010);
				if(read_len == 0)//没有接收到  超时判定
				{
					DelayMs_Tick(100);
					time_out--;
				}
				else//接收到了  处理数据
				{
					time_out = 200;
					uint8_t temp_check = 0;
					if(strcmp((const char*)soft_update_buffer,"x-q-update-restart") == 0)//复位指令
					{
						printf("s-q-update-restart");
						FLASH_Lock();
						DelayMs_Tick(1000);//1S后复位
//						SystemReset();
						Jump_To_APP();
					}
					else//不是指令  进行flash写入
					{
						for(i = 0;i < read_len-1;i++)//计算校验
							temp_check ^= soft_update_buffer[i];
						if(temp_check == soft_update_buffer[read_len-1])//校验成功
						{
							uint8_t fs_ok = 1;//本次写入是否出问题
							for(i=0;i<read_len-1;i+=4)//一次写4字节
							{
								uint32_t temp_data = 0;
								temp_data = soft_update_buffer[i];
								temp_data |= ((uint32_t)soft_update_buffer[i+1]<<8);
								temp_data |= ((uint32_t)soft_update_buffer[i+2]<<16);
								temp_data |= ((uint32_t)soft_update_buffer[i+3]<<24);
								FLASH_Status fs = FLASH_ProgramWord(now_address,temp_data);
								if(fs != FLASH_COMPLETE)
								{
									printf("prm-error-%d\r\n",fs);
									fs_ok = 0;//写入失败
									break;
								}
								
								uint32_t temp_read = FLASH_ReadWord(now_address);//写完以后再读出来，比较是否写入成功
								if(temp_read != temp_data)//读出来的和写入的不一样
								{
									printf("write-error-%u %u\r\n",temp_data,temp_read);
									fs_ok = 0;//写入失败
									break;
								}
								
								now_address = now_address+4;
							}
							if(fs_ok==1)//本次写入全部成功
								printf("s-q-check-ok");
						}
						else
						{
							//校验失败 软复位
							printf("s-q-check-fail");
							DelayMs_Tick(1000);
							SystemReset();
						}					
					}

				}
				if(time_out == 0)
				{
					//printf("Time out\r\n");
					break;//超时
				}
			}
			FLASH_Lock();
		}
		else if(strcmp((const char*)soft_update_buffer,"x-q-get-sversion") == 0)//获取当前软件版本
		{
			uint16_t s_ver = FLASH_ReadHalfWord(SYS_ADDRESS+0x00);//读取软件版本
			printf("s-q-sversion-%u",s_ver);
		}
		else if(strcmp((const char*)soft_update_buffer,"x-q-get-hversion") == 0)//获取当前硬件版本
		{
			uint16_t s_ver = FLASH_ReadHalfWord(SYS_ADDRESS+0x02);//读取硬件版本
			printf("s-q-hversion-%u",s_ver);
		}
		else if(strcmp((const char*)soft_update_buffer,"x-q-get-hardid") == 0)//获取当前唯一硬件号
		{
			u32 temp0,temp1,temp2;
			temp0 = *(__IO u32*)(0x1FFFF7E8);    //产品唯一身份标识寄存器（96位）
			temp1 = *(__IO u32*)(0x1FFFF7EC);
			temp2 = *(__IO u32*)(0x1FFFF7F0);
			printf("s-q-hardid-%08X%08X%08X",temp0,temp1,temp2);
		}
	}
}

void bq_reset(void)
{
	Single_Write(0x6b<<1,0x01,0x9B);//复位寄存器
	DelayMs_Tick(100);
	Single_Write(0x6b<<1,0x00,0x35);// 最大电流输入1.5A
	DelayMs_Tick(5);
	Single_Write(0x6b<<1,0x01,0x1F);//最低系统电压3.7V  禁用OTG升压模式
	DelayMs_Tick(5);
	Single_Write(0x6b<<1,0x04,0xB0);//最低电池电压2.8V
	DelayMs_Tick(5);
	Single_Write(0x6b<<1,0x05,0x86);//关掉看门狗
	DelayMs_Tick(5);
	Single_Write(0x6b<<1,0x06,0x7F);//关闭升压温度检测 
	DelayMs_Tick(5);
}

void PowerOFF(void)
{
	Single_Write(0x6b<<1,0x05,0x86);//关掉看门狗
	DelayMs_Tick(5);
	Single_Write(0x6b<<1,0x07,0x6B);//关掉电池输出
	DelayMs_Tick(5);
}

void charge_handle(void)
{
	uint8_t temp_reg,VBUS,CHRG,last_VBUS,status,time_count=0;

	bq_reset();
	
reload:
	status = Single_Read(0x6b<<1,0x09,&temp_reg);
	if(status!= FALSE && temp_reg)
	{
		if(temp_reg & 0x80)//看门狗溢出
		{
			bq_reset();
			PowerOFF();
		}
		else if(temp_reg & 0x10)//输出电压不正常
		{
			LED3 = !LED3;
		}
		else if(temp_reg)
		{
			while(1)
			{
				printf("failed:%02X\r\n",temp_reg);
				LED3 = !LED3;
				DelayMs_Tick(100);
			}		
		}

	}
	else
		LED3 = OFF;
	status = Single_Read(0x6b<<1,0x08,&temp_reg);
	if(status!=FALSE)
	{
		VBUS = (temp_reg>>6)&0x03;//电源接入状态
		CHRG = (temp_reg>>4)&0x03;//充电状态
		
		if(VBUS == 0x02)//如果电源已经接入
		{
			LED1 = !LED1;
			if(CHRG == 0x01 || CHRG == 0x02)
				LED2 = !LED2;
			else
				LED2 = 1;
			LED4 = 1;
			last_VBUS = VBUS;
			while(time_count++<5)//500MS闪烁  期间进行指令检测
			{
				sf_update_check();
				DelayMs_Tick(100);
			}
			time_count=0;
			goto reload;
		}
		else//如果电源没接入
		{
			if(last_VBUS == 0x02)//但是上次电源是接入状态，则表示电源被拔出了
			{
				Single_Write(0x6b<<1,0x05,0x8C);//关掉看门狗
				DelayMs_Tick(1);
				Single_Write(0x6b<<1,0x07,0x6B);//关掉电池输入
				DelayMs_Tick(1000);
			}
		}
	}
	else
	{
		LED1 = LED2 = LED3 = LED4 = ON;
		goto reload;
	}
}

void Key_Init(void)
{
	GPIO_QuickInit(GPIOB,GPIO_Pin_3,GPIO_Mode_IPU);
	GPIO_QuickInit(GPIOA,GPIO_Pin_9,GPIO_Mode_Out_PP);
}

void Key_Int_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_QuickInit(GPIOB,GPIO_Pin_3,GPIO_Mode_IPU);
	//配置中断
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3); 
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
	EXTI_ClearITPendingBit(EXTI_Line3);
  /* 配置中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3)!=RESET) //确保是否产生了EXTI Line中断
	{
		EXTI_ClearITPendingBit(EXTI_Line3);     //清除中断标志位
		SystemInit();
		//唤醒回来，直接系统复位
		SystemReset();
	}
}
void Power_OFF(void)
{
	
	//所有IO上拉，减少天线效应电流
	GPIO_QuickInit(GPIOA,GPIO_Pin_All,GPIO_Mode_IPU);
	GPIO_QuickInit(GPIOB,GPIO_Pin_All,GPIO_Mode_IPU);
	GPIO_QuickInit(GPIOC,GPIO_Pin_All,GPIO_Mode_IPU);
	GPIO_QuickInit(GPIOD,GPIO_Pin_All,GPIO_Mode_IPU);
	GPIO_QuickInit(GPIOA,GPIO_Pin_9,GPIO_Mode_Out_PP);//唤醒按键
	GPIO_QuickInit(GPIOA,GPIO_Pin_15,GPIO_Mode_IPD);//USB使能端口，拉低
	PAout(9)=0;//关闭外部IC电源和升压
	//配置唤醒中断
	Key_Int_Init();
	//进入STOP模式
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	while(1);
}

void normal_start_scan(void)
{
	uint8_t * pmode = NULL;
	pmode = (uint8_t*)(0x20000000+0x5000-4);
	//uint16_t mode = FLASH_ReadHalfWord(MODE_ADDRESS + 0x00);//读取用户程序指令
	uint8_t mode = *pmode;
	if(mode != 0xff)
	{
		//FLASH_Unlock();
		//do{}while(FLASH_ErasePage(MODE_ADDRESS + 0x000)!=FLASH_COMPLETE);
		//FLASH_Lock();
		*pmode = 0xff;
		if(mode == 0x01)
		{
			LED1 = ON;
			LED2 = OFF;
			LED3 = OFF;
			LED4 = OFF;
			DelayMs_Tick(100);
			Power_OFF();
		}
		else
		{

			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) != Bit_RESET)
			{Power_OFF();}	
		}
		
	}
	else
	{

		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) != Bit_RESET)
		{Power_OFF();}		
	}

	
//	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10) == Bit_RESET)//如果按键没有按下  则表示非正常开机
//	{
//		PowerOFF();
//		DelayMs(100);
//	}
}

int main(void)
{
	//I2C_INIT();
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,(0x08000000+4));//重新配置中断向量表到IAP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	Key_Init();
	PAout(9)=1;
	Init_SysTick();
	led_init();
	normal_start_scan();
	
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) == Bit_RESET);//等待按键弹起
	//USB_Config();
	led_disp();
	//charge_handle();
	//DelayMs_Tick(100);
	Jump_To_APP();
}

