#include "nrf24l01.h"
#include "stm32fx_delay.h"
u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xCD,0xEF,0xFF,0xCF,0x0E};   //���ص�ַ
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0xCD,0xEF,0xFF,0xCF,0x0E};  //Ҫ����˭����

/**
  * @brief  SPI�� I/O����
  * @param  ��
  * @retval ��
  */
void SPI_NRF_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

 /*������ӦIO�˿ڵ�ʱ��*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
                         |RCC_APB2Periph_GPIOB
												 |RCC_APB2Periph_AFIO
												 ,ENABLE);

	/*ʹ��SPI1ʱ��*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/*���� SPI_NRF_SPI�� SCK,MISO,MOSI���ţ�GPIOA^5,GPIOA^6,GPIOA^7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù���
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  /*����SPI_NRF_SPI��CE����,��SPI_NRF_SPI�� CSN ����*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*����SPI_NRF_SPI��IRQ����*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //��������
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
//	/*�ж�����*/
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//  /* �����ж�Դ */
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//	
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0); 
//  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�½����ж�
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure); 

//	EXTI_ClearITPendingBit(EXTI_Line0);
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
	
}

/**
  * @brief   ������NRF��/дһ�ֽ�����
  * @param   д�������
  *		@arg dat 
  * @retval  ��ȡ�õ�����
  */
u8 SPI_NRF_RW(u8 dat)
{  	
   /* �� SPI���ͻ������ǿ�ʱ�ȴ� */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
   /* ͨ�� SPI2����һ�ֽ����� */
  SPI_I2S_SendData(SPI1, dat);		
   /* ��SPI���ջ�����Ϊ��ʱ�ȴ� */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/**
  * @brief   ������NRF�ض��ļĴ���д������
  * @param   
  *		@arg reg:NRF������+�Ĵ�����ַ
  *		@arg dat:��Ҫ��Ĵ���д�������
  * @retval  NRF��status�Ĵ�����״̬
  */
u8 SPI_NRF_WriteReg(u8 reg,u8 dat)
{
	u8 status;
	/*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();
	/*��������Ĵ����� */
	status = SPI_NRF_RW(reg);
	 /*��Ĵ���д������*/
	SPI_NRF_RW(dat); 
	/*CSN���ߣ����*/	   
	NRF_CSN_HIGH();	
	/*����״̬�Ĵ�����ֵ*/
	return(status);
}

/**
  * @brief   ���ڴ�NRF�ض��ļĴ�����������
  * @param   
  *		@arg reg:NRF������+�Ĵ�����ַ
  * @retval  �Ĵ����е�����
  */
u8 SPI_NRF_ReadReg(u8 reg)
{
 	u8 reg_val;
	/*�õ�CSN��ʹ��SPI����*/
 	NRF_CSN_LOW();
  	 /*���ͼĴ�����*/
	SPI_NRF_RW(reg); 
	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI_NRF_RW(NOP);
   	/*CSN���ߣ����*/
	NRF_CSN_HIGH();		 
	return reg_val;
}	

/**
  * @brief   ������NRF�ļĴ�����д��һ������
  * @param   
  *		@arg reg : NRF������+�Ĵ�����ַ
  *		@arg pBuf�����ڴ洢���������ļĴ������ݵ����飬�ⲿ����
  * 	@arg bytes: pBuf�����ݳ���
  * @retval  NRF��status�Ĵ�����״̬
  */
u8 SPI_NRF_ReadBuf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status, byte_cnt;
	/*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();
	/*���ͼĴ�����*/		
	status = SPI_NRF_RW(reg); 
	/*��ȡ����������*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)		  
		pBuf[byte_cnt] = SPI_NRF_RW(NOP); //��NRF24L01��ȡ����  
	/*CSN���ߣ����*/
	NRF_CSN_HIGH();	
	return status;		//���ؼĴ���״ֵ̬
}

/**
  * @brief   ������NRF�ļĴ�����д��һ������
  * @param   
  *		@arg reg : NRF������+�Ĵ�����ַ
  *		@arg pBuf���洢�˽�Ҫд��д�Ĵ������ݵ����飬�ⲿ����
  * 	@arg bytes: pBuf�����ݳ���
  * @retval  NRF��status�Ĵ�����״̬
  */
u8 SPI_NRF_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes)
{
	u8 status,byte_cnt;
	/*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();			
	/*���ͼĴ�����*/	
	status = SPI_NRF_RW(reg); 
	/*�򻺳���д������*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		SPI_NRF_RW(*pBuf++);	//д���ݵ������� 	 
	/*CSN���ߣ����*/
	NRF_CSN_HIGH();			
	return (status);	//����NRF24L01��״̬ 		
}

/**
  * @brief  ���ò��������ģʽ
  * @param  ��
  * @retval ��
  */
void NRF_RX_Mode(void)
{
	NRF_CE_LOW();	
	SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
 
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);      //����RFͨ��Ƶ��    
	SPI_NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��      
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,250kbps,���������濪��
	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	NRF_CE_HIGH();
}

/**
  * @brief  ���÷���ģʽ
  * @param  ��
  * @retval ��
  */
void NRF_TX_Mode(void)
{  
	NRF_CE_LOW();		
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);       //����RFͨ��ΪCHANAL
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,250kbps,���������濪��  
	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_CE_HIGH();
	DelayMs(1);
}


/**
  * @brief  ��Ҫ����NRF��MCU�Ƿ���������
  * @param  ��
  * @retval SUCCESS/ERROR ��������/����ʧ��
  */
u8 NRF_Check(void)
{
	u8 buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2};
	u8 buf1[5];
	u8 i; 
	/*д��5���ֽڵĵ�ַ.  */  
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);
	/*����д��ĵ�ַ */
	SPI_NRF_ReadBuf(TX_ADDR,buf1,5); 
	/*�Ƚ�*/               
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

u8 NRF_Tx_Dat(u8 *txbuf) 
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


u8 NRF_Rx_Dat(u8 *rxbuf)
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
