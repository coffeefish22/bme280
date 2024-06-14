#include "bme_iic.h"
#include "globalval.h"
#include "myconfig.h"
#include "bme280.h"
#include "myconfig.h"
#include <rtthread.h>
#include "globalsys.h"

void I2C_Register(void);

I2C_Bit_Ops IIC1_Bit;		//����һ�� ��I2C1 ����
I2C_Bit_Ops IIC2_Bit;		//����һ�� ��I2C2 ����

void IIC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* ʹ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);		//ʱ��ʹ�� 	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14; 	//��IIC PB10--SCL , PB11--SDA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		   		//��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;		   		//IO���ٶ�Ϊ10MHz
	GPIO_Init(GPIOD, &GPIO_InitStructure);					 		   		//������ʼ��
 	GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_14);  
	
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13; 	//��IIC PB6--SCL , PB7--SDA
	GPIO_Init(GPIOD, &GPIO_InitStructure);					 		   	//������ʼ��
 	GPIO_SetBits(GPIOD, GPIO_Pin_11 | GPIO_Pin_13);  


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //��©���   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22Mʱ���ٶ�   
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
#ifdef HUMI_CONTROL_FAN	
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
#else
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);
#endif

	I2C_Register();
}

void IIC_Start(I2C_Bit_Ops *I2C)
{
	I2C->IIC_SCL_OUT(0);
	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SDA_OUT(1);	 
	delay_us(I2C_DELAY_TIME);
	I2C->IIC_SCL_OUT(1);
	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SDA_OUT(0); 				 
	delay_us(I2C_DELAY_TIME);	
	I2C->IIC_SCL_OUT(0); 				
	delay_us(I2C_DELAY_TIME);	
	
	I2C->IIC_SDA_OUT(1);
	delay_us(I2C_DELAY_TIME);
}	 


//����IICֹͣ�ź�
void IIC_Stop(I2C_Bit_Ops *I2C)
{

	I2C->IIC_SCL_OUT(0);
	I2C->IIC_SDA_OUT(0);	//STOP:when CLK is high DATA change form low to high

 	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SCL_OUT(1);
	
	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SDA_OUT(1); //����I2C���߽����ź�
	
	delay_us(I2C_DELAY_TIME);							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(I2C_Bit_Ops *I2C)
{
	u8 ucErrTime=0;
	     
	I2C->IIC_SDA_OUT(1);

	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SCL_OUT(1);
	
	delay_us(I2C_DELAY_TIME);	 
	
	while(I2C->READ_SDA_IN())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop(I2C);
			return 1;
		}
	}

	I2C->IIC_SCL_OUT(0); //ʱ�����0 	
	delay_us(I2C_DELAY_TIME);
	I2C->IIC_SDA_OUT(1);
	delay_us(I2C_DELAY_TIME);
	return 0;  
} 

//����ACKӦ��
void IIC_Ack(I2C_Bit_Ops *I2C)
{
	
	I2C->IIC_SDA_OUT(0);
	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SCL_OUT(1);
	
	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SCL_OUT(0);
	
	delay_us(I2C_DELAY_TIME);
	
	I2C->IIC_SDA_OUT(1);
	
	delay_us(I2C_DELAY_TIME);
}

//������ACKӦ��		    
void IIC_NAck(I2C_Bit_Ops *I2C)
{

	I2C->IIC_SDA_OUT(1);
	delay_us(I2C_DELAY_TIME);
	I2C->IIC_SCL_OUT(1);
	delay_us(I2C_DELAY_TIME);
	I2C->IIC_SCL_OUT(0);
}	

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(I2C_Bit_Ops *I2C, u8 txd)
{                        
	u8 t;   
	
	I2C->IIC_SCL_OUT(0); //����ʱ�ӿ�ʼ���ݴ���

	for(t = 0; t < 8; t++)
	{
		I2C->IIC_SDA_OUT((txd & 0x80) >> 7);
		
		txd <<= 1; 	  
		delay_us(I2C_DELAY_TIME);   
		
		I2C->IIC_SCL_OUT(1);
		delay_us(I2C_DELAY_TIME); 
		
		I2C->IIC_SCL_OUT(0);
		delay_us(I2C_DELAY_TIME);
	}

	I2C->IIC_SDA_OUT(1);	
	delay_us(I2C_DELAY_TIME);
} 	

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(I2C_Bit_Ops *I2C, unsigned char ack)
{
	unsigned char i = 0;
	unsigned char receive = 0;
	
   for(i = 0; i < 8; i++ )
	{
		I2C->IIC_SCL_OUT(1);
		delay_us(I2C_DELAY_TIME);
		
    receive <<= 1;
    if(I2C->READ_SDA_IN())
			receive++;  
		
		
		I2C->IIC_SCL_OUT(0);
		delay_us(I2C_DELAY_TIME); 
  }		

	
	if (!ack)
			IIC_NAck(I2C);//����nACK
	else
			IIC_Ack(I2C); //����ACK   
	return receive;
}


/*******************************/
void I2C1_SCL_OUT(uint8_t cmd)
{
	if(cmd == 1)
	{
		IIC1_SCL_OUT_HIGH();
	}else if(cmd == 0)
	{
		IIC1_SCL_OUT_LOW();
	}
	
}

void I2C1_SDA_OUT(uint8_t cmd)
{
	if(cmd == 1)
	{
		IIC1_SDA_OUT_HIGH();
	}else if(cmd == 0)
	{
		IIC1_SDA_OUT_LOW();
	}
	
}

uint8_t I2C1_SDA_In(void)
{
	return IIC1_SDA_IN();
	
}


/*****************************/
void I2C2_SCL_OUT(uint8_t cmd)
{
	if(cmd == 1)
	{
		IIC2_SCL_OUT_HIGH();
	}else if(cmd == 0)
	{
		IIC2_SCL_OUT_LOW();
	}
	
}

void I2C2_SDA_OUT(uint8_t cmd)
{
	if(cmd == 1)
	{
		IIC2_SDA_OUT_HIGH();
	}else if(cmd == 0)
	{
		IIC2_SDA_OUT_LOW();
	}
	
}

uint8_t I2C2_SDA_In(void)
{
	return IIC2_SDA_IN();
	
}

void I2C_Register(void)
{

	IIC1_Bit.IIC_SCL_OUT = I2C1_SCL_OUT;
	IIC1_Bit.IIC_SDA_OUT = I2C1_SDA_OUT;
	IIC1_Bit.READ_SDA_IN = I2C1_SDA_In;
	
	IIC2_Bit.IIC_SCL_OUT = I2C2_SCL_OUT;
	IIC2_Bit.IIC_SDA_OUT = I2C2_SDA_OUT;
	IIC2_Bit.READ_SDA_IN = I2C2_SDA_In;

}
