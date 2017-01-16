#include "stm32f10x.h"
#include "i2c_gpio.h"
#include "stm32fx_delay.h"

#define GPIO_PORT_I2C	GPIOB
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB
#define I2C_SCL_PIN		GPIO_Pin_11
#define I2C_SDA_PIN		GPIO_Pin_10

#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()  GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */

#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()  GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */

#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */

static void i2c_CfgGpio(void);

static void i2c_Delay(void)
{
	uint8_t i;
	/*　
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
	*/
	for (i = 0; i < 10; i++);
}

void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

void i2c_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
			I2C_SDA_1();
		else
			I2C_SDA_0();
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
			 I2C_SDA_1(); // 释放总线
		_ucByte <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}

uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
			value++;
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
		re = 1;
	else
		re = 0;
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

static void i2c_CfgGpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */

	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	/* 开漏输出 */
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	i2c_Stop();
}

uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	i2c_CfgGpio();/* 配置GPIO */
	DelayMs(10);
	i2c_Start();/* 发送启动信号 */
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_Address | I2C_WR);
	ucAck = i2c_WaitAck();/* 检测设备的ACK应答 */
	i2c_Stop();/* 发送停止信号 */
	return ucAck;
}

uint8_t WriteByte(uint8_t h_addr,uint8_t _usAddress,uint8_t WriteBuf)
{
	i2c_Stop();
	DelayMs(10);
	i2c_Start();
	i2c_SendByte(h_addr|I2C_WR);
	if (i2c_WaitAck() == 0)goto cmd_fail;
	i2c_SendByte((uint8_t)_usAddress);
	if (i2c_WaitAck() != 0)goto cmd_fail;
	i2c_SendByte(WriteBuf);
	if (i2c_WaitAck() != 0)goto cmd_fail;
	i2c_Stop();
	return 1;
cmd_fail:
	i2c_Stop();
	return 0;
}
uint8_t WriteBytes(uint8_t h_addr,uint8_t _usAddress,uint8_t *_pWriteBuf, uint16_t _usSize)
{
	uint16_t i;
	for(i=0;i<_usSize;i++)
	{
		if(i==0)
		{
			i2c_Start();
			i2c_SendByte(h_addr|I2C_WR);
			if (i2c_WaitAck() == 0)goto cmd_fail;
			i2c_SendByte((uint8_t)_usAddress);
			if (i2c_WaitAck() != 0)goto cmd_fail;
		}
		i2c_SendByte(_pWriteBuf[i]);
		if (i2c_WaitAck() != 0)goto cmd_fail;
		
	}
	i2c_Stop();
	return 1;
cmd_fail:
	i2c_Stop();
	return 0;
}

