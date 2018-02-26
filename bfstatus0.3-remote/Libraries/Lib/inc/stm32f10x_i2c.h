#ifndef __STM32F10x_I2C_H__
#define __STM32F10x_I2C_H__
#include "stm32f10x.h"
#define GPIO_I2C	     GPIOB
#define SCL_PIN        GPIO_Pin_10
#define SDA_PIN        GPIO_Pin_11
#define RCC_GPIO_I2C	 RCC_APB2Periph_GPIOB

#define FALSE 0
#define TRUE 1


#define SCL_H         GPIO_I2C->BSRR = SCL_PIN /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIO_I2C->BRR  = SCL_PIN /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIO_I2C->BSRR = SDA_PIN /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIO_I2C->BRR  = SDA_PIN /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      GPIO_I2C->IDR  & SCL_PIN /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIO_I2C->IDR  & SDA_PIN /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */



void I2C_INIT(void);
void I2C_delay(void);
void delay5ms(void);
uint16_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void); 
void I2C_NoAck(void);
uint16_t I2C_WaitAck(void);
void I2C_SendByte(unsigned char SendByte);
unsigned char I2C_RadeByte(void);
uint16_t Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char *data);
void I2C_Read(u8 addr_, u8 reg_, int32_t len, u8 *buf);
void I2C_Write(u8 addr_, u8 reg_, int32_t len, u8 *buf);
#endif
