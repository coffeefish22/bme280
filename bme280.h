#ifndef __BME280_H
#define __BME280_H	 


#include "string.h"
#include "BME_iic.h"

//stm32 rt_thread 2.0
typedef struct {
	uint8_t id;
	uint8_t dig_H1; 
	uint8_t dig_H3;
	uint8_t dig_H6;

	uint16_t dig_T1;
	uint16_t dig_P1;
	uint16_t dig_H2;
	uint16_t dig_H4;
	uint16_t dig_H5;
	uint16_t temp;
	
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	int16_t dig_T2;
	int16_t dig_T3;
	
	
}BME280_RegisterDef;


struct BME280_Sensor_Struct
{
	
	I2C_Bit_Ops	*I2C;
	
	BME280_RegisterDef Register;
	
	int adc_H;     		//寄存器里的湿度值                  
	int adc_T;				//寄存器里的温度值 
	int adc_P;				//寄存器里的压力值
	int t_fine;
	
	int Pressure;
	int Humidity;
	int Temperature;
	
	int16_t Status;

};

#define BME280_DEVICE_ADDR	0x76		//SDO 引脚接3.3V  111011x; SDO 接3.3V x等于1，接GND x等于0
#define BME280_SIGNAL1			0x01
#define BME280_SIGNAL2			0x01

void BME280_Sensor1_Thread(void const *arg);
void BME280_Sensor2_Thread(void const *arg);
extern int bak_bme280_temp;
extern int bak_bme280_humi;
#define MAX_BME280_ERROR 30
extern struct BME280_Sensor_Struct BME280_Sensor1;
extern struct BME280_Sensor_Struct BME280_Sensor2;
extern volatile int is_reseting;
extern char reset_flag;
int Get_Bme280Value(struct BME280_Sensor_Struct *BME280);
void Bme280_InitParam(struct BME280_Sensor_Struct *BME280);
void IIC_Read_NBytes(I2C_Bit_Ops *I2C, u8 dev_addr, u8 reg_addr, u8 cnt, u8 *reg_data);
int bme280_abs(int a);
void BME280_reset(void);
void BME280_init1(void);

#endif

