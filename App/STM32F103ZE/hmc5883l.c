#include <stdlib.h>
#include <math.h>
#include "hmc5883l.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "main.h"

extern EventGroupHandle_t xEventGruop;

void HMC5883L_Init(void)
{
	HMC5883L_Conf_Reg	hmc5883l_conf_regs;

	HMC5883L_IO_Init();
	
	hmc5883l_conf_regs.REG_A = 0x74;
	hmc5883l_conf_regs.REG_B = 0x20;
	hmc5883l_conf_regs.REG_M = 0x00;
	I2C1_IO_Write(HMC5883L_SlaveAddr, 0x00, sizeof(hmc5883l_conf_regs), (uint8_t *)&hmc5883l_conf_regs.REG_A);
}

uint8_t HMC5883L_IsReady(uint32_t Trials)
{
	I2C1_IO_Init();

	return (I2C1_IO_IsDeviceReady(HMC5883L_SlaveAddr, Trials) == HAL_OK);
}

uint8_t HMC5883L_ReadStatus(void)
{
	uint8_t	tmp = 0;

	I2C1_IO_Read(HMC5883L_SlaveAddr, 0x09, 1, &tmp);

	return tmp;
}

uint8_t HMC5883L_ReadDst(uint8_t DstAddr)
{
	uint8_t tmp = 0;

	I2C1_IO_Read(HMC5883L_SlaveAddr, DstAddr, 1, &tmp);

	return tmp;
}

void HMC5883L_ISR(void)
{
	BaseType_t xResult, xHigherPriorityTaskWoken;

	xResult = xEventGroupSetBitsFromISR(xEventGruop, HMC_DATA_READY, &xHigherPriorityTaskWoken);
	
	if (xResult != pdFAIL)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HMC_Data_Cal(HMC_Data_t *hmc_data)
{
	float tmp, r;

	r = sqrt((double)hmc_data->direct.x*(double)hmc_data->direct.x
			+(double)hmc_data->direct.y*(double)hmc_data->direct.y
			+(double)hmc_data->direct.z*(double)hmc_data->direct.z);

	tmp = (double)hmc_data->direct.x/r;
	tmp = acos(tmp);
	hmc_data->angle.x = tmp*180.0/3.1415926;
	
	tmp = (double)hmc_data->direct.y/r;
	tmp = acos(tmp);
	hmc_data->angle.y = tmp*180.0/3.1415926;
	
	tmp = (double)hmc_data->direct.z/r;
	tmp = acos(tmp);
	hmc_data->angle.z = tmp*180.0/3.1415926;
}

uint8_t HMC5883L_ReadAngle(HMC_Data_t *hmc_data)
{
	uint8_t outputData[6];
	BaseType_t uxBits;
	const TickType_t xTickToWait = 100;		// Time Out one second.

	uxBits = xEventGroupWaitBits(xEventGruop, HMC_DATA_READY, pdTRUE, pdFALSE, xTickToWait);

	if (uxBits & HMC_DATA_READY)
	{
		I2C1_IO_Read(HMC5883L_SlaveAddr, 0x03, 6, outputData);

		hmc_data->direct.x = outputData[0] << 8 | outputData[1];
		hmc_data->direct.y = outputData[2] << 8 | outputData[3];
		hmc_data->direct.z = outputData[4] << 8 | outputData[5];

		HMC_Data_Cal(hmc_data);
		
		return pdTRUE;
	}

	return pdFALSE;
}