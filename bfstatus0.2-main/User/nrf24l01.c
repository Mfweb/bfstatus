/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @author  Mfweb
  * @version V1.0
  * @date    2016.8.02
  * @brief
  * @note    BiFang Status Mini nRF24L01����
  ******************************************************************************
  */
#include "nrf24l01.h"
#include "stm32fx_delay.h"
#include <stdio.h>
#include "led.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xCD,0xEF,0xFF,0xCF,0x0E};
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0xCD,0xEF,0xFF,0xCF,0x0E};
uint8_t RX_DAT[16];
/* nRF��ʼ��  ���ﲻʹ���ж� */
uint8_t NRF_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/*SCK,MISO,MOSI���ţ�GPIOA^5,GPIOA^6,GPIOA^7 */
	GPIO_QuickInit(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_AF_PP);
  /*CE���� CSN����*/
	GPIO_QuickInit(GPIOA,GPIO_Pin_4,GPIO_Mode_Out_PP);
	GPIO_QuickInit(GPIOB,GPIO_Pin_10,GPIO_Mode_Out_PP);

  /* ����csn���ţ�NRF�������״̬ */
  NRF_CSN_HIGH(); 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;												//��ģʽ
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;										//���ݴ�С8λ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;													//ʱ�Ӽ��ԣ�����ʱΪ��
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;												//��1��������Ч��������Ϊ����ʱ��
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;														//NSS�ź����������
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //8��Ƶ��9MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;									//��λ��ǰ
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
	DelayMs(500);
	
	if(NRF_Check()!=SUCCESS)
	{
		return ERROR;
	}
	else
	{
		DelayMs(50);
		NRF_RX_Mode();		
		return SUCCESS;
	}
}

uint8_t SPI_NRF_RW(uint8_t dat)
{  	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, dat);		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}


uint8_t SPI_NRF_WriteReg(uint8_t reg,uint8_t dat)
{
	uint8_t status;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg);
	SPI_NRF_RW(dat);   
	NRF_CSN_HIGH();	
	return(status);
}

uint8_t SPI_NRF_ReadReg(uint8_t reg)
{
 	uint8_t reg_val;
 	NRF_CSN_LOW();
	SPI_NRF_RW(reg); 
	reg_val = SPI_NRF_RW(NOP);
	NRF_CSN_HIGH();		 
	return reg_val;
}	

uint8_t SPI_NRF_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status, byte_cnt;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg);
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		pBuf[byte_cnt] = SPI_NRF_RW(NOP);
	NRF_CSN_HIGH();
	return status;
}

uint8_t SPI_NRF_WriteBuf(uint8_t reg ,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_cnt;
	NRF_CSN_LOW();
	status = SPI_NRF_RW(reg);
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		SPI_NRF_RW(*pBuf++);
	NRF_CSN_HIGH();
	return (status);
}
/* �������ģʽ */
void NRF_RX_Mode(void)
{
	NRF_CE_LOW();	
	SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
 
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);
	SPI_NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);     
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,2mbps,���������濪��
	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_CE_HIGH();
}
/* ���뷢��ģʽ */
void NRF_TX_Mode(void)
{  
	NRF_CE_LOW();		
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2mbps,���������濪��  
	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_CE_HIGH();
	DelayMs(1);
}
/* ���nRF�Ƿ���� */
uint8_t NRF_Check(void)
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
	
	if(i==5)
		return SUCCESS ;	//MCU��NRF�ɹ����� 
	else
		return ERROR ;		//MCU��NRF����������
}

void DelayNoSched(uint16_t ms)
{
	uint16_t i,j;
	for(i=0;i<ms;i++)
		for(j=0;j<9600;j++);
}

uint8_t NRF_Tx_Dat(uint8_t *txbuf) 
{
	NRF_TX_Mode();
	NRF_CE_LOW();
	SPI_NRF_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	NRF_CE_HIGH();
	DelayMs(1);
	SPI_NRF_WriteReg(FLUSH_TX,NOP);
	NRF_RX_Mode();
	return TX_DS;
} 


uint8_t NRF_Rx_Dat(uint8_t *rxbuf)
{
	
	uint8_t sta = SPI_NRF_ReadReg(STATUS);
	if(sta & RX_DR)
	{
		NRF_CE_LOW();
		SPI_NRF_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS,sta);
		NRF_CE_HIGH();
		sta = 0;
		return RX_DR;
	}
	return ERROR;
}
