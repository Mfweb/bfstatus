#include "stm32f10x_i2c.h"
#include "stm32fx_delay.h"

/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 
	RCC_APB2PeriphClockCmd(RCC_GPIO_I2C, ENABLE);  //使能GPIOB时钟
  
	GPIO_InitStructure.GPIO_Pin =  SCL_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);
	DelayMs(5);
	I2C_Stop();
	DelayMs(5);
}
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
uint16_t I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
uint16_t I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
	  I2C_delay();
      return FALSE;
	}
	SCL_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(unsigned char SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 

uint16_t Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
	while(I2C_Start() == FALSE);
	//if(!I2C_Start())return FALSE;
	I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
	if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
	I2C_SendByte(REG_Address );   //设置低起始地址
	I2C_WaitAck();
	
	I2C_SendByte(REG_data);
	I2C_WaitAck();   
	
	I2C_Stop(); 
	//DelayMs(5);
	return TRUE;
}

void I2C_Write(u8 addr_, u8 reg_, int32_t len, u8 *buf)
{
	int32_t i;
	while(I2C_Start() == FALSE);
	//I2C_Start();
	I2C_SendByte(addr_);
	I2C_WaitAck();
	I2C_SendByte(reg_);
	I2C_WaitAck();
	for (i=0; i<len; i++)
	{
		I2C_SendByte(*(buf+i));
		I2C_WaitAck();
	}
	I2C_Stop();
	//DelayMs(5);
}
//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char *data)
{  
	unsigned char REG_data;     
	while(I2C_Start() == FALSE);	
//	if(!I2C_Start())
//		return FALSE;
	I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
	if(!I2C_WaitAck())
	{
		I2C_Stop();
		return FALSE;
	}
	I2C_SendByte((u8) REG_Address);   //设置低起始地址      
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);
	I2C_WaitAck();

	REG_data= I2C_RadeByte();
	I2C_NoAck();
	I2C_Stop();
	*data = REG_data;
	return TRUE;
}	

void I2C_Read(u8 addr_, u8 reg_, int32_t len, u8 *buf)
{
	int32_t i;
	 // 起始信号
	while(I2C_Start() == FALSE);
	//I2C_Start();
	// 发送设备地址
	I2C_SendByte(addr_);   
	I2C_WaitAck();
	//发送存储单元地址
	I2C_SendByte(reg_);                   
	I2C_WaitAck();
	// 起始信号
	I2C_Start();
	//发送设备地址+读信号
	I2C_SendByte(addr_+1);     
	I2C_WaitAck();
	for (i=0; i<len; i++)                   //连续读取6个地址数据，存储中BUF
	{
		*(buf+i) = I2C_RadeByte();          //BUF[0]存储数据
		if (i == len-1)		I2C_NoAck();                   //最后一个数据需要回NOACK
		else		I2C_Ack();                     //回应ACK
	}
	I2C_Stop();                           //停止信号
}
