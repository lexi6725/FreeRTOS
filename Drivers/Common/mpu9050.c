#include "FreeRTOS.h"
#include <string.h>
#include <math.h>
#include "stm3210e_bit3.h"
#include "event_groups.h"
#include "main.h"
#include "mpu9050.h"

extern EventGroupHandle_t xEventGruop;

void MPU9050_IO_Init(void)
{
	GPIO_InitTypeDef  gpioinitstruct = {0};  

	gpioinitstruct.Pin = GPIO_PIN_1;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING;
	gpioinitstruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioinitstruct);
	
	/* Enable and set Eval EXTI8(PA8) Interrupt to the highest priority */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void MPU9050_Init(void)
{
	uint8_t regValue ;
	
	I2C1_IO_Init();

	MPU9050_IO_Init();

	regValue = 0x80;
	I2C1_IO_Write(SlaveAddress, AUX_VDDIO, 1, &regValue);
	
	regValue = 0x00;
	I2C1_IO_Write(SlaveAddress, PWR_MGMT_1, 1, &regValue);

	regValue = 0x20;
	I2C1_IO_Write(SlaveAddress, SMPLRT_DIV, 1, &regValue);

	regValue = 0x06;
	I2C1_IO_Write(SlaveAddress, CONFIG, 1, &regValue);

	regValue = 0x18;
	I2C1_IO_Write(SlaveAddress, GYRO_CONFIG, 1, &regValue);

	regValue = 0x01;
	I2C1_IO_Write(SlaveAddress, ACCEL_CONFIG, 1, &regValue);

	regValue = 0x02;
	I2C1_IO_Write(SlaveAddress, INT_PIN_CFG, 1, &regValue);
	
	regValue = 0x01;
	I2C1_IO_Write(SlaveAddress, INT_ENABLE, 1, &regValue);
}

void MPU9050_ISR(void)
{
	BaseType_t xResult, xHigherPriorityTaskWoken;

	xResult = xEventGroupSetBitsFromISR(xEventGruop, MPU_DATA_READY, &xHigherPriorityTaskWoken);
	
	if (xResult != pdFAIL)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void MPU9050_Data_Cal(mpu9050_data_t *mpu_data, mpu9050_t *mpu_orig)
{
	float R;
	static triaxial_t pre_gyro;

	R = sqrt((double)mpu_orig->accel.x*(double)mpu_orig->accel.x+
			(double)mpu_orig->accel.y*(double)mpu_orig->accel.y+
			(double)mpu_orig->accel.z*(double)mpu_orig->accel.z);

	mpu_data->accel.x = acos((double)mpu_orig->accel.x/R)*180.0/3.1415926;
	mpu_data->accel.y = acos((double)mpu_orig->accel.y/R)*180.0/3.1415926;
	//mpu_data->accel.z = acos((double)mpu_orig->accel.z/R)*180.0/3.1415926;

	if (mpu_orig->gyro.x != 0 && mpu_orig->gyro.y != 0 && mpu_orig->gyro.z != 0)
	{
		mpu_data->gyro.x = (float)(mpu_orig->gyro.x-pre_gyro.x)/33.3334;
		mpu_data->gyro.y = (float)(mpu_orig->gyro.y-pre_gyro.y)/33.3334;
		mpu_data->gyro.z = (float)(mpu_orig->gyro.z-pre_gyro.z)/33.3334;
		memcpy((uint8_t *)&pre_gyro, (uint8_t *)&mpu_orig->gyro, sizeof(pre_gyro));
	}
}

uint8_t MPU9050_Read(mpu9050_t *mpu9050)
{
	uint8_t buff[6];
	
	I2C1_IO_Read(SlaveAddress, ACCEL_XOUT_H, 6, buff);
	mpu9050->accel.x = (buff[0]<<8)+buff[1];
	mpu9050->accel.y = (buff[2]<<8)+buff[3];
	mpu9050->accel.z = (buff[4]<<8)+buff[5];
	I2C1_IO_Read(SlaveAddress, GYRO_XOUT_H, 6, buff);
	mpu9050->gyro.x = (buff[0]<<8)+buff[1];
	mpu9050->gyro.y = (buff[2]<<8)+buff[3];
	mpu9050->gyro.z = (buff[4]<<8)+buff[5];

	return 0;
}
