#include "nrf24l01.h"
#include "stm32fx_delay.h"
#include "key_led.h"
#include "FreeRTOS.h"
#include "battery.h"
#include "task.h"
#include "semphr.h"
#include "oled.h"

nrf_data_ gSensorNRF = {
	.TX_BUF={0},
	.RX_BUF={0},
	.TX_ADDRESS = {0xCD,0xEF,0xFF,0xCF,0x0E},
	.RX_ADDRESS = {0xCD,0xEF,0xFF,0xCF,0x0E},
	.Channel		= 30
};
static void NRF_SETUP(void);
/**
  * @brief  SPI的 I/O配置
  * @param  无
  * @retval 无
  */
void SPI_NRF_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

 /*开启相应IO端口的时钟*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
                         |RCC_APB2Periph_GPIOB
												 |RCC_APB2Periph_AFIO
												 ,ENABLE);

	/*使能SPI1时钟*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/*配置 SPI_NRF_SPI的 SCK,MISO,MOSI引脚，GPIOA^5,GPIOA^6,GPIOA^7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  /*配置SPI_NRF_SPI的CE引脚,和SPI_NRF_SPI的 CSN 引脚*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*配置SPI_NRF_SPI的IRQ引脚*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
//	/*中断配置*/
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//  /* 配置中断源 */
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//	
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0); 
//  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //下降沿中断
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure); 

//	EXTI_ClearITPendingBit(EXTI_Line0);
  /* 拉高csn引脚，NRF进入空闲状态 */
  NRF_CSN_HIGH(); 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//双线全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;												//主模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;										//数据大小8位
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;													//时钟极性，空闲时为低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;												//第1个边沿有效，上升沿为采样时刻
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;														//NSS信号由软件产生
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //8分频，9MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;									//高位在前
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
	
	if(NRF_Check()==ERROR)
	{
		oled_printf(0,1,"nRF error!");
		while(1)
		{
			LED2 = !LED2;
			DelayMs(1000);
		}
	}
	else
	{
		DelayMs(100);
		NRF_SETUP();
		NRF_RX_Mode();
	}
	
//	NRF_CE_LOW();
//	SPI_NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0	,gSensorNRF.RX_ADDRESS,RX_ADR_WIDTH);		//写RX节点地址
//	SPI_NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR		,gSensorNRF.TX_ADDRESS,TX_ADR_WIDTH); 	//写TX节点地址 
//	SPI_NRF_WriteReg(NRF_WRITE_REG + SETUP_AW		,0x03);																	//设置地址长度5字节
//	SPI_NRF_WriteReg(NRF_WRITE_REG + RX_PW_P0		,RX_PLOAD_WIDTH);												//选择通道0的有效数据宽度
//	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_AA			,0x00);																	//不使用自动应答
//	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_CH			,gSensorNRF.Channel);      							//设置RF通信频率
//	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_SETUP		,0xE); 																	//设置TX发射参数,0db增益,2mbps,低噪声增益开启
//	SPI_NRF_WriteReg(NRF_WRITE_REG + SETUP_RETR	,0x00);																	//禁用重传
//	NRF_CE_HIGH();
}

u8 SPI_NRF_RW(u8 dat)
{  	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, dat);		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}

u8 SPI_NRF_WriteReg(u8 reg,u8 dat)
{
	u8 status;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg);
	SPI_NRF_RW(dat); 
	NRF_CSN_HIGH();	
	return(status);
}

u8 SPI_NRF_ReadReg(u8 reg)
{
 	u8 reg_val;
 	NRF_CSN_LOW();
	SPI_NRF_RW(reg); 
	reg_val = SPI_NRF_RW(NOP);
	NRF_CSN_HIGH();		 
	return reg_val;
}	

u8 SPI_NRF_ReadBuf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status, byte_cnt;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg); 
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)		  
		pBuf[byte_cnt] = SPI_NRF_RW(NOP); //从NRF24L01读取数据  
	NRF_CSN_HIGH();	
	return status;		//返回寄存器状态值
}

u8 SPI_NRF_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes)
{
	u8 status,byte_cnt;
	NRF_CSN_LOW();			
	status = SPI_NRF_RW(reg); 
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		SPI_NRF_RW(pBuf[byte_cnt]);	//写数据到缓冲区 	 
	NRF_CSN_HIGH();
	return (status);	//返回NRF24L01的状态 		
}

static void NRF_SETUP(void)
{
	NRF_CE_LOW();
	SPI_NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0	,gSensorNRF.RX_ADDRESS,RX_ADR_WIDTH);		//写RX节点地址
	SPI_NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR		,gSensorNRF.TX_ADDRESS,TX_ADR_WIDTH); 	//写TX节点地址 
	SPI_NRF_WriteReg(NRF_WRITE_REG + SETUP_AW		,0x03);																	//设置地址长度5字节
	SPI_NRF_WriteReg(NRF_WRITE_REG + RX_PW_P0		,RX_PLOAD_WIDTH);												//选择通道0的有效数据宽度
	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_AA			,0x00);																	//不使用自动应答
	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_CH			,gSensorNRF.Channel);      							//设置RF通信频率
	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_SETUP		,0xE); 																	//设置TX发射参数,0db增益,2mbps,低噪声增益开启
	SPI_NRF_WriteReg(NRF_WRITE_REG + SETUP_RETR	,0x00);																	//禁用重传
	NRF_CE_HIGH();
}

void NRF_RX_Mode(void)
{
	NRF_CE_LOW();
	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG,0x0f);	//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	NRF_CE_HIGH();
}

/**
  * @brief  配置发送模式
  * @param  无
  * @retval 无
  */
void NRF_TX_Mode(void)
{  
	NRF_CE_LOW();
	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG,0x0e);	//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断
	NRF_CE_HIGH();
	vTaskDelay(pdMS_TO_TICKS(1));
}


/**
  * @brief  主要用于NRF与MCU是否正常连接
  * @param  无
  * @retval SUCCESS/ERROR 连接正常/连接失败
  */
u8 NRF_Check(void)
{
	u8 buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2};
	u8 buf1[5];
	u8 i; 
	/*写入5个字节的地址.  */  
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);
	/*读出写入的地址 */
	SPI_NRF_ReadBuf(TX_ADDR,buf1,5); 
	/*比较*/               
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0xC2)
		break;
	} 
	if(i==5)
		return SUCCESS ;	//MCU与NRF成功连接 
	else
		return ERROR ;		//MCU与NRF不正常连接
}

u8 NRF_Tx_Dat(u8 *txbuf) 
{
	NRF_TX_Mode();
	NRF_CE_LOW();
	SPI_NRF_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	NRF_CE_HIGH();
	uint8_t sta = SPI_NRF_ReadReg(STATUS);
	int time_out = 10000;
	while(!(sta & TX_DS))
	{
		sta = SPI_NRF_ReadReg(STATUS);
		if(--time_out == 0)break;
	}
	SPI_NRF_WriteReg(FLUSH_TX,NOP);
	SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS,sta);
	NRF_RX_Mode();
	vTaskDelay(pdMS_TO_TICKS(1));
	return TX_DS;
} 


u8 NRF_Rx_Dat(u8 *rxbuf)
{
	uint8_t sta = SPI_NRF_ReadReg(STATUS);
	if(sta & RX_DR)
	{
		NRF_CE_LOW();
		SPI_NRF_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS,sta);
		NRF_CE_HIGH();
		return RX_DR;
	}
	return ERROR;
}
