/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini nRF24L01驱动
  ******************************************************************************
  */
#include "nrf24l01.h"
#include "stm32fx_delay.h"
#include <stdio.h>
#include "led.h"
#include "app.h"



uint8_t NRF_Init(void);
static void NRF_SETUP(void);
static uint8_t SPI_NRF_RW(uint8_t dat);
static uint8_t SPI_NRF_ReadReg(uint8_t reg );
static uint8_t SPI_NRF_WriteReg(uint8_t reg,uint8_t dat);
static uint8_t SPI_NRF_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t bytes);
static uint8_t SPI_NRF_WriteBuf(uint8_t reg ,uint8_t *pBuf,uint8_t bytes);	


void NRF_TX_Mode(void);
void NRF_RX_Mode(void);
uint8_t NRF_Rx_Dat(uint8_t *rxbuf);
uint8_t NRF_Tx_Dat(uint8_t *txbuf);
static uint8_t NRF_Check(void);

data_rf_t gSensorNRF = {
	.Init = &NRF_Init,
	.TXMode = &NRF_TX_Mode,
	.RXMode = &NRF_RX_Mode,
	.RXData = &NRF_Rx_Dat,
	.TXData = &NRF_Tx_Dat,
	.TX_BUF={0},
	.RX_BUF={0},
	.TX_ADDRESS = {0xCD,0xEF,0xFF,0xCF,0x0E},
	.RX_ADDRESS = {0xCD,0xEF,0xFF,0xCF,0x0E},
	.Channel		= 30
};

/* nRF初始化  这里不使用中断 */
uint8_t NRF_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/*SCK,MISO,MOSI引脚，GPIOA^5,GPIOA^6,GPIOA^7 */
	GPIO_QuickInit(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_AF_PP);
  /*CE引脚 CSN引脚*/
	GPIO_QuickInit(GPIOB,GPIO_Pin_1,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_2,GPIO_Mode_Out_PP);

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
	gSensorNRF.Channel = 30;//默认30频道
	if(NRF_Check()!=SUCCESS)
	{
		return ERROR;
	}
	else
	{
		DelayMs(50);
		NRF_SETUP();
		NRF_RX_Mode();	
		return SUCCESS;
	}
}

static uint8_t SPI_NRF_RW(uint8_t dat)
{  	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, dat);		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}


static uint8_t SPI_NRF_WriteReg(uint8_t reg,uint8_t dat)
{
	uint8_t status;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg);
	SPI_NRF_RW(dat);   
	NRF_CSN_HIGH();	
	return(status);
}

static uint8_t SPI_NRF_ReadReg(uint8_t reg)
{
 	uint8_t reg_val;
 	NRF_CSN_LOW();
	SPI_NRF_RW(reg); 
	reg_val = SPI_NRF_RW(NOP);
	NRF_CSN_HIGH();		 
	return reg_val;
}	

static uint8_t SPI_NRF_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status, byte_cnt;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg);
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		pBuf[byte_cnt] = SPI_NRF_RW(NOP);
	NRF_CSN_HIGH();
	return status;
}

static uint8_t SPI_NRF_WriteBuf(uint8_t reg ,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_cnt;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg);
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		SPI_NRF_RW(*pBuf++);
	NRF_CSN_HIGH();
	return (status);
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
/* 进入接收模式 */
void NRF_RX_Mode(void)
{
	NRF_CE_LOW();
	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG,0x0f);	//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	NRF_CE_HIGH();
}
/* 进入发送模式 */
void NRF_TX_Mode(void)
{  
	NRF_CE_LOW();
	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG,0x0e);	//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式
	NRF_CE_HIGH();
	vTaskDelay(pdMS_TO_TICKS(1));
}
/* 检查nRF是否接入 */
static uint8_t NRF_Check(void)
{
	uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2};
	uint8_t buf1[5];
	uint8_t i;
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);
	SPI_NRF_ReadBuf(TX_ADDR,buf1,5);
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0xC2)
			break;
	}
	
	if(i == 5)
		return SUCCESS ;	//MCU与NRF成功连接
	else
		return ERROR ;		//MCU与NRF不正常连接
}

uint8_t NRF_Tx_Dat(uint8_t *txbuf)
{
	xSemaphoreTake(flag.spi_sem, 0);
	NRF_TX_Mode();
	NRF_CE_LOW();
	SPI_NRF_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	NRF_CE_HIGH();
	uint8_t sta = SPI_NRF_ReadReg(STATUS);
	while(!(sta & TX_DS))//等待发送完成
		sta = SPI_NRF_ReadReg(STATUS);
	SPI_NRF_WriteReg(FLUSH_TX,NOP);
	SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS,sta);//写1清除发送完成位
	NRF_RX_Mode();
	xSemaphoreGive(flag.spi_sem);
	return TX_DS;
} 

uint8_t NRF_Rx_Dat(uint8_t *rxbuf)
{
	xSemaphoreTake(flag.spi_sem, 0);
	uint8_t sta = SPI_NRF_ReadReg(STATUS);
	if(sta & RX_DR)
	{
		NRF_CE_LOW();
		SPI_NRF_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS,sta);
		NRF_CE_HIGH();
		xSemaphoreGive(flag.spi_sem);
		return RX_DR;
	}
	xSemaphoreGive(flag.spi_sem);
	return ERROR;
}

