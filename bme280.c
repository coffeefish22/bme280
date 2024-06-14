#include <rtthread.h>
#include "stm32f10x.h"
#include "myconfig.h"
#include "bme280.h"
#include "myconfig.h"
#include <rtthread.h>
#include "globalsys.h"
#include "globalval.h"
#ifdef USE_BME280_TASK
				  
struct BME280_Sensor_Struct BME280_Sensor1;
struct BME280_Sensor_Struct BME280_Sensor2;
#define MAX_BME280_ERROR 30
volatile int is_reseting=0;
char reset_flag=0;
extern float bak_temp,bak_humi;

/*
 *	������ : IIC_ReadOneByte
 *	����	 : I2C����һ���ֽ�����
 *	����	 ���Ĵ�����ַ
 *	����ֵ ��������������
 */
u8 IIC_ReadOneByte(I2C_Bit_Ops *I2C, u8 dev_addr, u8 reg_addr)
{				  
	u8 temp=0;
	
  IIC_Start(I2C);  	
	IIC_Send_Byte(I2C, (dev_addr << 1));   //����������ַ��д����
	IIC_Wait_Ack(I2C); 
	
  IIC_Send_Byte(I2C, reg_addr);   			//���ͼĴ�����ַ
	IIC_Wait_Ack(I2C);
	
	IIC_Start(I2C);  	 	   
	IIC_Send_Byte(I2C,  (dev_addr << 1) | 0x01);  //�������ģʽ		
	IIC_Wait_Ack(I2C);	 
	temp=IIC_Read_Byte(I2C, 0);		   
	
	IIC_Stop(I2C);//����һ��ֹͣ����	 
	
	return temp;
}


/*
 *	������ : IIC_Write_OneBytes
 *	����	 : I2Cд1���ֽڵ�����
 *	����	 ��1.I2C�豸�ĵ�ַ ; 2.�Ĵ�����ַ ;  4.��д������
 *	����ֵ ����
 */
void IIC_Write_OneBytes(I2C_Bit_Ops *I2C, u8 dev_addr, u8 reg_addr, u8 reg_data)
{
	u8   ucAddr;	
	ucAddr = reg_addr;
	IIC_Start(I2C);
	IIC_Send_Byte(I2C, dev_addr << 1);  
	IIC_Wait_Ack(I2C);
	
  IIC_Send_Byte(I2C, ucAddr % 256);   //���͵͵�ַ
	IIC_Wait_Ack(I2C); 	
	
	IIC_Send_Byte(I2C, reg_data);     //�����ֽ�							   
	IIC_Wait_Ack(I2C);  
	
  IIC_Stop(I2C);
}

/*
 *	������ : IIC_Read_NBytes
 *	����	 : I2C����N���ֽڵ�����
 *	����	 ��1.I2C�豸�ĵ�ַ ; 2.�Ĵ�����ַ �� 3.�������ݵ����� ��4.�洢�������ݵ��׵�ַ
 *	����ֵ ����
 */
void IIC_Read_NBytes(I2C_Bit_Ops *I2C, u8 dev_addr, u8 reg_addr, u8 cnt, u8 *reg_data)
{  	
	u8 t;
	
	for(t = 0; t < cnt ; t++)
	{
		*reg_data = IIC_ReadOneByte(I2C, dev_addr, reg_addr + t); 
		reg_data++;
	} 
}




/*
 *	������ : IIC_Bme280_Init
 *	����	 : BME280 I2C��ʼ�� ���Ҷ�������ֵ
 *	����	 ����
 *	����ֵ ����
 */
void Bme280_InitParam(struct BME280_Sensor_Struct *BME280)
{
	uint8_t	t_cmd;
	uint8_t buf_Bme280[25]={0};
	uint8_t bme280[7]={0};
	uint8_t buf_H1 = 0;

	/* ��ȡ�¶�ѹ������ֵ */
	IIC_Read_NBytes(BME280->I2C, BME280_DEVICE_ADDR, 0x88 , 24, buf_Bme280);
	
	/* ��ȡʪ�Ȳ���ֵ */
	IIC_Read_NBytes(BME280->I2C, BME280_DEVICE_ADDR, 0xE1, 7, bme280);
	
	/* ��ȡʪ�Ȳ���ֵ */
	IIC_Read_NBytes(BME280->I2C, BME280_DEVICE_ADDR, 0xA1, 1, &buf_H1);

	/* �����¶Ȳ���ֵ */	
	BME280->Register.dig_T1 = buf_Bme280[1] << 8 | buf_Bme280[0];
	BME280->Register.dig_T2 = buf_Bme280[3] << 8 | buf_Bme280[2];
	BME280->Register.dig_T3 = buf_Bme280[5] << 8 | buf_Bme280[4];

	/*����ѹ������ֵ*/
	BME280->Register.dig_P1 = buf_Bme280[7] << 8 | buf_Bme280[6];
	BME280->Register.dig_P2 = buf_Bme280[9] << 8 | buf_Bme280[8];
	BME280->Register.dig_P3 = buf_Bme280[11] << 8 | buf_Bme280[10];
	BME280->Register.dig_P4 = buf_Bme280[13] << 8 | buf_Bme280[12];
	BME280->Register.dig_P5 = buf_Bme280[15] << 8 | buf_Bme280[14];
	BME280->Register.dig_P6 = buf_Bme280[17] << 8 | buf_Bme280[16];
	BME280->Register.dig_P7 = buf_Bme280[19] << 8 | buf_Bme280[18];
	BME280->Register.dig_P8 = buf_Bme280[21] << 8 | buf_Bme280[20];
	BME280->Register.dig_P9 = buf_Bme280[23] << 8 | buf_Bme280[22];
	
	/*����ʪ�Ȳ���ֵ*/
	BME280->Register.dig_H1 = buf_H1;
	BME280->Register.dig_H2 = bme280[1] << 8 | bme280[0];
	BME280->Register.dig_H3 = bme280[2];
	BME280->Register.dig_H4 = (int16_t)(bme280[3]) << 4 | (bme280[4] & 0x0F);
	BME280->Register.dig_H5 = (bme280[4] & 0x0F) | (int16_t)(bme280[3]) >> 4;
	BME280->Register.dig_H6 = bme280[6];
			
	t_cmd = 0xB6;
	/*��λ*/
	IIC_Write_OneBytes(BME280->I2C, BME280_DEVICE_ADDR, 0xE0, t_cmd);
	
	t_cmd = 0x00;
	/*����config�Ĵ���*/	
	IIC_Write_OneBytes(BME280->I2C, BME280_DEVICE_ADDR, 0xF5, t_cmd);	
	
	t_cmd = 0x05;	
	/*����ʪ�ȼĴ���*/		
	IIC_Write_OneBytes(BME280->I2C, BME280_DEVICE_ADDR, 0xF2, t_cmd);
	
	t_cmd = 0xA5;
	/*�����¶ȼ�ѹ���Ĵ���*/	
	IIC_Write_OneBytes(BME280->I2C, BME280_DEVICE_ADDR, 0xF4, t_cmd);
	
}




/*
 *	������ : IIC_Bme280_Read_Data
 *	����	 : I2C����N���ֽڵ�����
 *	����	 ����
 *	����ֵ ��
 */
int IIC_Bme280_Read_Data(struct BME280_Sensor_Struct *BME280)
{
	
	I2C_Bit_Ops	*I2C;
	uint8_t Buf_Bme280[8]={0};
	
	I2C = BME280->I2C;
	
	/*foce mode ����*/	
	IIC_Write_OneBytes(I2C, BME280_DEVICE_ADDR, 0xF4, 0x25);
	
	rt_thread_delay(100);

	/*��ȡ�¶�ѹ��ʪ��8���ֽ�����*/	
	IIC_Read_NBytes(I2C, BME280_DEVICE_ADDR, 0xF7, 0x08, Buf_Bme280);
	
	BME280->adc_T = (int32_t)((Buf_Bme280[3] << 16 | Buf_Bme280[4] << 8 | Buf_Bme280[5]) >> 4);
	BME280->adc_H = (int32_t)((Buf_Bme280[6] << 8 | Buf_Bme280[7]));
	BME280->adc_P = (int32_t)((Buf_Bme280[0] << 16 | Buf_Bme280[1] << 8 | Buf_Bme280[2]) >> 4);
	
	
	IIC_Read_NBytes(I2C, BME280_DEVICE_ADDR, 0xD0, 1, &(BME280->Register.id));
	
	if(BME280->Register.id == 0x60)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
 *	������ : Bme280_temperature
 *	����	 : BME280 �¶�ֵ����
 *	����	 ��ͨ��I2C�������¶�ֵ
 *	����ֵ ���������¶�ֵ
 */
int32_t Bme280_temperature(struct BME280_Sensor_Struct *BME280)
{
	int32_t var1,var2,T;


	var1 = ((((BME280->adc_T >> 3) - ((int32_t)BME280->Register.dig_T1 << 1))) * ((int32_t)BME280->Register.dig_T2)) >> 11;
	var2 = (((((BME280->adc_T >> 4) - ((int32_t)BME280->Register.dig_T1)) * ((BME280->adc_T>>4) - ((int32_t)BME280->Register.dig_T1))) >> 12) *((int32_t)BME280->Register.dig_T3)) >> 14;
	BME280->t_fine = var1 + var2;
	T = (BME280->t_fine * 5 + 128) >> 8;
	return T;
}

/*
 *	������ : Bme280_pressure
 *	����	 : BME280 ѹ��ֵ����
 *	����	 ��ͨ��I2C������ѹ��
 *	����ֵ ��������ѹ��ֵ
 */
uint32_t Bme280_pressure(struct BME280_Sensor_Struct *BME280)
{
	int64_t var1,var2;
	int64_t P;
	
	var1 = ((int64_t)BME280->t_fine) - 128000;
	var2 = var1*var1*(int64_t)BME280->Register.dig_P6;
	var2 = var1 * var1 * (int64_t)BME280->Register.dig_P6;
	var2 = var2 + ((var1 * (int64_t)BME280->Register.dig_P5) << 17);
	var2 = var2 + (((int64_t)BME280->Register.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)BME280->Register.dig_P3) >> 8) + ((var1 * (int64_t)BME280->Register.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)BME280->Register.dig_P1) >> 33;
	
	if(var1 == 0)
	{
		return 0;
	}
	
	P = 1048576 - BME280->adc_P;
	P = (((P << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)BME280->Register.dig_P9) * (P >> 13) * (P >> 13)) >> 25;
	var2 = (((int64_t)BME280->Register.dig_P8) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)BME280->Register.dig_P7) << 4);
	return (uint32_t)P;	
}

/*
 *	������ : Bme280_compensate_H
 *	����	 : BME280 ʪ��ֵ����
 *	����	 ��ͨ��I2C������ʪ��ֵ
 *	����ֵ ��������ʪ��ֵ
 */
uint32_t Bme280_compensate_H(struct BME280_Sensor_Struct *BME280)
{
	int32_t v_x1_u32r;


	v_x1_u32r = (BME280->t_fine -((int32_t)76800));
	v_x1_u32r = (((((BME280->adc_H << 14)-(((int32_t)BME280->Register.dig_H4) << 20)-(((int32_t)BME280->Register.dig_H5) * v_x1_u32r)) +
	((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)BME280->Register.dig_H6)) >> 10) * (((v_x1_u32r *
	((int32_t)BME280->Register.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)BME280->Register.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r-(((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)BME280->Register.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r >> 12);
}

/*
 *	������ : Get_Bme280Value
 *	����	 : BME280 �¶ȣ�ʪ�ȣ�ѹ������
 *	����	 ����
 *	����ֵ ����
 */
int Get_Bme280Value(struct BME280_Sensor_Struct *BME280)
{
	
	/*��ȡ�¶�ѹ��ʪ������*/	
	if(IIC_Bme280_Read_Data(BME280) == 1)
	{
		BME280->Temperature = Bme280_temperature(BME280)*10;   //100
		BME280->Pressure  = Bme280_pressure(BME280) / 256;  //256000
		BME280->Humidity = Bme280_compensate_H(BME280);	    //1024
		BME280->Status = 1;
		
	}else {
		
		BME280->Temperature = 0;
		BME280->Pressure = 0;
		BME280->Humidity = 0;
		BME280->Status = 0;
	}
	return BME280->Status;
}


void Init_BME280_Device(void)
{
	static int init_Flag;
	
	if(init_Flag == 0)
	{
		init_Flag = 1;
		BME280_Sensor1.I2C = &IIC1_Bit;
		BME280_Sensor2.I2C = &IIC2_Bit;
	}

}
int bme280_abs(int a)
{
	if(a<0)
		return -a;
	else
		return a;
}
#ifdef BME1
int bak_bme280_temp=-80000;
int bak_bme280_humi=-50;
void BME280_init1()
{
	IIC_GPIO_Config();
	rt_thread_delay(2000);
	Init_BME280_Device();
	Bme280_InitParam(&BME280_Sensor1);
}
void BME280_task_entry_1(void *param)
{
	int count =0;
	int Bme280_state=0;
	int bak_Bme280_state=0;
	int temp_k=0;
	uint8_t buf_Bme280[25]={0};
	IIC_GPIO_Config();
	rt_thread_delay(2000);
	Init_BME280_Device();
	Bme280_InitParam(&BME280_Sensor1);
	while(1)
	{
		if(is_reseting==0)
		{
			Bme280_state=Get_Bme280Value(&BME280_Sensor1);
			if(Bme280_state)
			{
				if(bak_Bme280_state==0)
				{
					Bme280_InitParam(&BME280_Sensor1);
				}
				else
				{
					/* ��ȡ�¶�ѹ������ֵ */
					IIC_Read_NBytes(BME280_Sensor1.I2C, BME280_DEVICE_ADDR, 0x88 , 24, buf_Bme280);
					BME280_Sensor1.Register.dig_T1 = buf_Bme280[1] << 8 | buf_Bme280[0];
					BME280_Sensor1.Register.dig_T2 = buf_Bme280[3] << 8 | buf_Bme280[2];
					BME280_Sensor1.Register.dig_T3 = buf_Bme280[5] << 8 | buf_Bme280[4];
				}
				count=0;
				if(bak_bme280_temp<-50000||bme280_abs(bak_bme280_temp-BME280_Sensor1.Temperature)<10000)
				{
					if(BME280_Sensor1.Temperature>-50000 &&BME280_Sensor1.Temperature<70000)
					{
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)>>16);
						modifyro(OUTSIDE_TMP,BME280_Sensor1.Temperature+temp_k,0);
						bak_bme280_temp=BME280_Sensor1.Temperature;
					}
					else
					{
						count++;	
					}
					
				}
				else
				{
					count++;
				}
				if(bak_bme280_humi<0||bme280_abs(bak_bme280_humi-BME280_Sensor1.Humidity)<15000)
				{
					if(BME280_Sensor1.Humidity>=0 &&BME280_Sensor1.Humidity<=100000)
					{
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)&0xffff);
						temp_k+=BME280_Sensor1.Humidity;
						if(temp_k<0)
							temp_k=0;
						else if(temp_k>100000)
							temp_k=100000;
						modifyro(OUTSIDE_HUM,temp_k,0);
						bak_bme280_humi=BME280_Sensor1.Humidity;
					}
				}
				else
					count++;
	//			modifyro(OUTSIDE_HUM,BME280_Sensor1.Humidity,0);
	//			rt_kprintf("Tempe1=%d\n",BME280_Sensor1.Temperature);
				Clr_Sht75_1_StaFlag();
			}
			else
			{
				count++;
				if(count>MAX_BME280_ERROR)
				{
					count=0;
	//				modifyro(OUTSIDE_TMP,0,0);
	//				modifyro(OUTSIDE_HUM,0,0);
					Set_Sht75_1_StaFlag();
	
					rt_kprintf("set 13\n");
				}
			}
			bak_Bme280_state=Bme280_state;
		}
		rt_thread_delay(400);
	}
}
void BME280_reset_task_entry_1(void *param);

void BME280_task_init1()
{
	rt_thread_t thread;//thread_reset;
	
    /* create led1 thread */
    thread = rt_thread_create("BME280",
                              BME280_task_entry_1, RT_NULL,
                              1024,
                              THREAD_SHT75_BME280_PRIORITY, 50);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    /* create led1 thread */
    thread= rt_thread_create("BME_re",
                              BME280_reset_task_entry_1, RT_NULL,
                             256,
                              THREAD_SHT75_RESET_PRIORITY, 10);

	
    if (thread != RT_NULL)
        rt_thread_startup(thread);



}
#endif
#ifdef BME2
void BME280_task_entry_2(void *param)
{
	int count =0;
	int Bme280_state=0;
	int bak_Bme280_state=0;
	int temp_k=0;
	uint8_t buf_Bme280[25]={0};
	IIC_GPIO_Config();
	rt_thread_delay(2000);
	Init_BME280_Device();
	Bme280_InitParam(&BME280_Sensor2);
	while(1)
	{
		if(is_reseting==0)
		{
			Bme280_state=Get_Bme280Value(&BME280_Sensor2);
			if(Bme280_state)
			{
				if(bak_Bme280_state==0)
				{
					Bme280_InitParam(&BME280_Sensor2);
				}
				else
				{
					/* ��ȡ�¶�ѹ������ֵ */
					IIC_Read_NBytes(BME280_Sensor2.I2C, BME280_DEVICE_ADDR, 0x88 , 24, buf_Bme280);
					BME280_Sensor2.Register.dig_T1 = buf_Bme280[1] << 8 | buf_Bme280[0];
					BME280_Sensor2.Register.dig_T2 = buf_Bme280[3] << 8 | buf_Bme280[2];
					BME280_Sensor2.Register.dig_T3 = buf_Bme280[5] << 8 | buf_Bme280[4];
				}
				count=0;
				if(BME280_Sensor2.Temperature>-50000 &&BME280_Sensor2.Temperature<70000)
				{
					temp_k=(short)(getrw(ROAD_TEMP_HUMI_K)>>16);
					modifyro(UNDER_COVER_TMP,BME280_Sensor2.Temperature+temp_k,0);
				}
				else
				{
					reset_flag=1;
				}
				if(BME280_Sensor2.Humidity>=0 &&BME280_Sensor2.Humidity<=100000)
				{
					temp_k=(short)(getrw(ROAD_TEMP_HUMI_K)&0xffff);
					temp_k+=BME280_Sensor2.Humidity;
					if(temp_k<0)
						temp_k=0;
					else if(temp_k>100000)
						temp_k=100000;
					modifyro(UNDER_COVER_HUM,temp_k,0);
				}
	//			rt_kprintf("Tempe2=%d\n",BME280_Sensor2.Temperature);
				Clr_Sht75_2_StaFlag();
			}
			else
			{
				count++;
				if(count>MAX_BME280_ERROR)
				{
					count=0;
					modifyro(UNDER_COVER_TMP,0,0);
					modifyro(UNDER_COVER_HUM,0,0);
					Set_Sht75_2_StaFlag();
				}
			}
			bak_Bme280_state=Bme280_state;
		}
		rt_thread_delay(400);
	}
}


void BME280_task_init2()
{
	rt_thread_t thread;//thread_reset;
	
    /* create led1 thread */
    thread = rt_thread_create("2BME280",
                              BME280_task_entry_2, RT_NULL,
                              1024,
                              THREAD_SHT75_BME280_PRIORITY2, 50);
    if (thread != RT_NULL)
        rt_thread_startup(thread);


}
#endif
#define BME280_EN_PORT 	GPIOC
#define BME280_EN_PIN	GPIO_Pin_9
#define BME280_EN_RCC	RCC_APB2Periph_GPIOC
#ifdef HUMI_CONTROL_FAN
#define Set_BME280_Enable() GPIO_SetBits(BME280_EN_PORT,BME280_EN_PIN)  
#define Set_BME280_Disable() GPIO_ResetBits(BME280_EN_PORT,BME280_EN_PIN)  	
#else
#define Set_BME280_Enable() GPIO_ResetBits(BME280_EN_PORT,BME280_EN_PIN)  
#define Set_BME280_Disable() GPIO_SetBits(BME280_EN_PORT,BME280_EN_PIN)  	
#endif
void BME280_reset(void)
{
	#define BME280_RESET_TIM	2000
	{
		Set_BME280_Disable();
		rt_thread_delay(BME280_RESET_TIM);
		Set_BME280_Enable();
		rt_thread_delay(BME280_RESET_TIM);
		Bme280_InitParam(&BME280_Sensor1);
		Bme280_InitParam(&BME280_Sensor2);
	}

}

void BME280_reset_task_entry_1(void *param)
{
	#define SHT75_RESET_SCAN_TIM 100
	while(1)
	{
		if(reset_flag)
		{
			reset_flag =0;
			is_reseting=1;
			BME280_reset();
			rt_thread_delay(500);
			bak_temp=-80000;  //��λ�󱸷�ֵ���ǲ���֮ǰ��
			bak_humi=-80;
			is_reseting=0;
		}	
		rt_thread_delay(SHT75_RESET_SCAN_TIM);
	}
}

#endif

