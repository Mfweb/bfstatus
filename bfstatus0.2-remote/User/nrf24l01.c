#include "nrf24l01.h"
#include "stm32fx_delay.h"
u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xCD,0xEF,0xFF,0xCF,0x0E};   //本地地址
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0xCD,0xEF,0xFF,0xCF,0x0E};  //要发从谁接收

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
	
}

/**
  * @brief   用于向NRF读/写一字节数据
  * @param   写入的数据
  *		@arg dat 
  * @retval  读取得的数据
  */
u8 SPI_NRF_RW(u8 dat)
{  	
   /* 当 SPI发送缓冲器非空时等待 */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
   /* 通过 SPI2发送一字节数据 */
  SPI_I2S_SendData(SPI1, dat);		
   /* 当SPI接收缓冲器为空时等待 */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/**
  * @brief   用于向NRF特定的寄存器写入数据
  * @param   
  *		@arg reg:NRF的命令+寄存器地址
  *		@arg dat:将要向寄存器写入的数据
  * @retval  NRF的status寄存器的状态
  */
u8 SPI_NRF_WriteReg(u8 reg,u8 dat)
{
	u8 status;
	/*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();
	/*发送命令及寄存器号 */
	status = SPI_NRF_RW(reg);
	 /*向寄存器写入数据*/
	SPI_NRF_RW(dat); 
	/*CSN拉高，完成*/	   
	NRF_CSN_HIGH();	
	/*返回状态寄存器的值*/
	return(status);
}

/**
  * @brief   用于从NRF特定的寄存器读出数据
  * @param   
  *		@arg reg:NRF的命令+寄存器地址
  * @retval  寄存器中的数据
  */
u8 SPI_NRF_ReadReg(u8 reg)
{
 	u8 reg_val;
	/*置低CSN，使能SPI传输*/
 	NRF_CSN_LOW();
  	 /*发送寄存器号*/
	SPI_NRF_RW(reg); 
	 /*读取寄存器的值 */
	reg_val = SPI_NRF_RW(NOP);
   	/*CSN拉高，完成*/
	NRF_CSN_HIGH();		 
	return reg_val;
}	

/**
  * @brief   用于向NRF的寄存器中写入一串数据
  * @param   
  *		@arg reg : NRF的命令+寄存器地址
  *		@arg pBuf：用于存储将被读出的寄存器数据的数组，外部定义
  * 	@arg bytes: pBuf的数据长度
  * @retval  NRF的status寄存器的状态
  */
u8 SPI_NRF_ReadBuf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status, byte_cnt;
	/*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();
	/*发送寄存器号*/		
	status = SPI_NRF_RW(reg); 
	/*读取缓冲区数据*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)		  
		pBuf[byte_cnt] = SPI_NRF_RW(NOP); //从NRF24L01读取数据  
	/*CSN拉高，完成*/
	NRF_CSN_HIGH();	
	return status;		//返回寄存器状态值
}

/**
  * @brief   用于向NRF的寄存器中写入一串数据
  * @param   
  *		@arg reg : NRF的命令+寄存器地址
  *		@arg pBuf：存储了将要写入写寄存器数据的数组，外部定义
  * 	@arg bytes: pBuf的数据长度
  * @retval  NRF的status寄存器的状态
  */
u8 SPI_NRF_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes)
{
	u8 status,byte_cnt;
	/*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();			
	/*发送寄存器号*/	
	status = SPI_NRF_RW(reg); 
	/*向缓冲区写入数据*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		SPI_NRF_RW(*pBuf++);	//写数据到缓冲区 	 
	/*CSN拉高，完成*/
	NRF_CSN_HIGH();			
	return (status);	//返回NRF24L01的状态 		
}

/**
  * @brief  配置并进入接收模式
  * @param  无
  * @retval 无
  */
void NRF_RX_Mode(void)
{
	NRF_CE_LOW();	
	SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
 
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);      //设置RF通信频率    
	SPI_NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度      
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,250kbps,低噪声增益开启
	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
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
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);       //设置RF通道为CHANAL
	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,250kbps,低噪声增益开启  
	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断
	NRF_CE_HIGH();
	DelayMs(1);
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
