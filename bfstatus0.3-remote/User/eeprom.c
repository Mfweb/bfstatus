#include "eeprom.h"
#include "oled.h"
#include "yg.h"
#include "stdio.h"

#define I2C_PageSize           8
#define EEPROM_ADDRESS				 0xA0

void I2C_EE_WaitEepromStandbyState(void)      
{
	while(!I2C_WaitAck())
	{
		I2C_Start();
		I2C_SendByte(EEPROM_ADDRESS);
	}
	I2C_Stop();
}

void I2C_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite)
{
  while(I2C_Start() == FALSE);
	
  I2C_SendByte(EEPROM_ADDRESS);
	I2C_WaitAck(); 
	
	I2C_SendByte(WriteAddr);
	I2C_WaitAck();
	
  while(NumByteToWrite--)  
  {
		I2C_SendByte(*pBuffer);
		I2C_WaitAck();
    pBuffer++;
  }
	I2C_Stop();
}

void I2C_EE_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
  while(I2C_Start() == FALSE);
	
	I2C_SendByte(EEPROM_ADDRESS);
  I2C_WaitAck();
	
	I2C_SendByte(ReadAddr); 
	I2C_WaitAck();
	
  while(I2C_Start() == FALSE);
	
  I2C_SendByte(EEPROM_ADDRESS+1);   
  I2C_WaitAck();
	
	
	for (uint16_t i = 0; i < NumByteToRead; i++)
	{
		*(pBuffer+i) = I2C_RadeByte();
		if (i == NumByteToRead - 1)
			I2C_NoAck();
		else
			I2C_Ack();
	}
	I2C_Stop();
}


void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

  Addr = WriteAddr % I2C_PageSize;
  count = I2C_PageSize - Addr;
  NumOfPage =  NumByteToWrite / I2C_PageSize;
  NumOfSingle = NumByteToWrite % I2C_PageSize;
 
  /* If WriteAddr is I2C_PageSize aligned  */
  if(Addr == 0) 
  {
    /* If NumByteToWrite < I2C_PageSize */
    if(NumOfPage == 0) 
    {
      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      I2C_EE_WaitEepromStandbyState();
    }
    /* If NumByteToWrite > I2C_PageSize */
    else  
    {
      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize); 
    	I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_PageSize;
        pBuffer += I2C_PageSize;
      }

      if(NumOfSingle!=0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        I2C_EE_WaitEepromStandbyState();
      }
    }
  }
  /* If WriteAddr is not I2C_PageSize aligned  */
  else 
  {
    /* If NumByteToWrite < I2C_PageSize */
    if(NumOfPage== 0) 
    {
      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      I2C_EE_WaitEepromStandbyState();
    }
    /* If NumByteToWrite > I2C_PageSize */
    else
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / I2C_PageSize;
      NumOfSingle = NumByteToWrite % I2C_PageSize;	
      
      if(count != 0)
      {  
        I2C_EE_PageWrite(pBuffer, WriteAddr, count);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr += count;
        pBuffer += count;
      } 
      
      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_PageSize;
        pBuffer += I2C_PageSize;  
      }
      if(NumOfSingle != 0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle); 
        I2C_EE_WaitEepromStandbyState();
      }
    }
  }  
}

uint8_t read_ee(void)
{
	uint8_t i,buff[24]={0xff};
	I2C_EE_BufferRead(buff,0,24);
	for(i=0;i<24;i++)
	{
		if(buff[i] != 0xff)break;
	}
	if(i == 24)return 0;
	YG_DATA.max.pitch		 = (int16_t)buff[0] | ((int16_t)buff[1]<<8);
	YG_DATA.max.roll		 = (int16_t)buff[2] | ((int16_t)buff[3]<<8);
	YG_DATA.max.yaw			 = (int16_t)buff[4] | ((int16_t)buff[5]<<8);
	YG_DATA.max.throttle = (int16_t)buff[6] | ((int16_t)buff[7]<<8);
	
	YG_DATA.min.pitch		 = (int16_t)buff[8] | ((int16_t)buff[9]<<8);
	YG_DATA.min.roll		 = (int16_t)buff[10] | ((int16_t)buff[11]<<8);
	YG_DATA.min.yaw			 = (int16_t)buff[12] | ((int16_t)buff[13]<<8);
	YG_DATA.min.throttle = (int16_t)buff[14] | ((int16_t)buff[15]<<8);
	
	YG_DATA.sta.pitch		 = (int16_t)buff[16] | ((int16_t)buff[17]<<8);
	YG_DATA.sta.roll		 = (int16_t)buff[18] | ((int16_t)buff[19]<<8);
	YG_DATA.sta.yaw			 = (int16_t)buff[20] | ((int16_t)buff[21]<<8);
	YG_DATA.sta.throttle = (int16_t)buff[22] | ((int16_t)buff[23]<<8);
	for(i=0;i<24;i++)
		printf("0x%02x ",buff[i]);
	printf("\r\n");
	return 1;
}


void save_ee(void)
{
	uint8_t buff[24],i;
	buff[0] = (uint8_t)YG_DATA.max.pitch;
	buff[1] = (uint8_t)(YG_DATA.max.pitch>>8);
	buff[2] = (uint8_t)YG_DATA.max.roll;
	buff[3] = (uint8_t)(YG_DATA.max.roll>>8);
	buff[4] = (uint8_t)YG_DATA.max.yaw;
	buff[5] = (uint8_t)(YG_DATA.max.yaw>>8);
	buff[6] = (uint8_t)YG_DATA.max.throttle;
	buff[7] = (uint8_t)(YG_DATA.max.throttle>>8);
	
	buff[8] = (uint8_t)YG_DATA.min.pitch;
	buff[9] = (uint8_t)(YG_DATA.min.pitch>>8);
	buff[10] = (uint8_t)YG_DATA.min.roll;
	buff[11] = (uint8_t)(YG_DATA.min.roll>>8);
	buff[12] = (uint8_t)YG_DATA.min.yaw;
	buff[13] = (uint8_t)(YG_DATA.min.yaw>>8);
	buff[14] = (uint8_t)YG_DATA.min.throttle;
	buff[15] = (uint8_t)(YG_DATA.min.throttle>>8);
	
	buff[16] = (uint8_t)YG_DATA.sta.pitch;
	buff[17] = (uint8_t)(YG_DATA.sta.pitch>>8);
	buff[18] = (uint8_t)YG_DATA.sta.roll;
	buff[19] = (uint8_t)(YG_DATA.sta.roll>>8);
	buff[20] = (uint8_t)YG_DATA.sta.yaw;
	buff[21] = (uint8_t)(YG_DATA.sta.yaw>>8);
	buff[22] = (uint8_t)YG_DATA.sta.throttle;
	buff[23] = (uint8_t)(YG_DATA.sta.throttle>>8);
//	if(!write_bytes(0xA0,0,buff,24))
//	{
//		oled_printf(0,0,"save error");
//		while(1);
//	}
	I2C_EE_BufferWrite(buff,0,24);
	for(i=0;i<24;i++)
	{
		printf("0x%02x ",buff[i]);
	}
	printf("\r\n");
}

