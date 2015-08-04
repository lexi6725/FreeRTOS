#include "FreeRTOS.h"
#include "stm3210e_bit3.h"
#include "event_groups.h"
#include "main.h"
#include "mpu9050.h"

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
	
	regValue = 0x00;
	I2C1_IO_Write(SlaveAddress, &regValue, PWR_MGMT_1, 1);

	regValue = 0x07;
	I2C1_IO_Write(SlaveAddress, &regValue, SMPLRT_DIV, 1);

	regValue = 0x06;
	I2C1_IO_Write(SlaveAddress, &regValue, CONFIG, 1);

	regValue = 0x18;
	I2C1_IO_Write(SlaveAddress, &regValue, GYRO_CONFIG, 1);

	regValue = 0x01;
	I2C1_IO_Write(SlaveAddress, &regValue, ACCEL_CONFIG, 1);
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


uint8_t MPU9050_Read(mpu9050_t *mpu9050)
{
	BaseType_t uxBits;
	const TickType_t xTickToWait = 100;		// Time Out one second.

	uxBits = xEventGroupWaitBits(xEventGruop, HMC_DATA_READY, pdTRUE, pdFALSE, xTickToWait);

	if (uxBits & MPU_DATA_READY)
	{
		I2C1_IO_Read(SlaveAddress, (uint8_t *)&mpu9050->accel, ACCEL_XOUT_H, sizeof(triaxial_t));
		I2C1_IO_Read(SlaveAddress, (uint8_t *)&mpu9050->gyro, GYRO_XOUT_H, sizeof(triaxial_t));
		
		return pdTRUE;
	}

	return pdFALSE;
}
