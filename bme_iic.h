#ifndef __IIC_H
#define __IIC_H

#include "stm32f10x.h"

#define IIC1_SDA_OUT_HIGH()		GPIO_SetBits(GPIOD, GPIO_Pin_12)
#define IIC1_SDA_OUT_LOW()		GPIO_ResetBits(GPIOD,	GPIO_Pin_12)
#define IIC1_SCL_OUT_HIGH()		GPIO_SetBits(GPIOD, GPIO_Pin_14)
#define IIC1_SCL_OUT_LOW()		GPIO_ResetBits(GPIOD,	GPIO_Pin_14)
#define IIC1_SDA_IN()			GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12)

#define IIC2_SDA_OUT_HIGH()		GPIO_SetBits(GPIOD, GPIO_Pin_13)
#define IIC2_SDA_OUT_LOW()		GPIO_ResetBits(GPIOD,	GPIO_Pin_13)
#define IIC2_SCL_OUT_HIGH()		GPIO_SetBits(GPIOD, GPIO_Pin_11)
#define IIC2_SCL_OUT_LOW()		GPIO_ResetBits(GPIOD,	GPIO_Pin_11)
#define IIC2_SDA_IN()			GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)

#define I2C_DELAY_TIME		10
	   
typedef struct{

	void (* IIC_SCL_OUT)(uint8_t cmd);
	void (* IIC_SDA_OUT)(uint8_t cmd);
	uint8_t (* READ_SDA_IN)(void);
	
}I2C_Bit_Ops;

extern I2C_Bit_Ops IIC1_Bit;
extern I2C_Bit_Ops IIC2_Bit;

void IIC_Start(I2C_Bit_Ops *I2C);				//发送IIC开始信号
void IIC_Stop(I2C_Bit_Ops *I2C);	  			//发送IIC停止信号
void IIC_Send_Byte(I2C_Bit_Ops *I2C, u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(I2C_Bit_Ops *I2C, unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(I2C_Bit_Ops *I2C);

////IIC等待ACK信号
void IIC_Ack(I2C_Bit_Ops *I2C);					//IIC发送ACK信号
void IIC_NAck(I2C_Bit_Ops *I2C);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,	u8 addr,	u8 data);
u8 IIC_Read_One_Byte(u8 daddr,	u8 addr);	  
void IIC_GPIO_Config(void);

#endif


