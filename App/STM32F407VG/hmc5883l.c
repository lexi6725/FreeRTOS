#include <stdlib.h>
#include "hmc5883l.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

void HMC5883L_Init(uint16_t DeviceAddr)
{
	HMC5883L_Conf_Reg	hmc5883l_conf_regs;

	hmc5883l_conf_regs.REG_A = 0x74;
	hmc5883l_conf_regs.REG_B = 0x20;
	hmc5883l_conf_regs.REG_M = 0x00;
	HMC5883L_IO_Write(0x3C, (uint8_t *)&hmc5883l_conf_regs.REG_A,0x00,sizeof(hmc5883l_conf_regs));
}

uint8_t HMC5883L_IsReady(uint16_t DeviceAddr, uint32_t Trials)
{
	HMC5883L_IO_Init();

	return (HMC5883L_IO_IsDeviceReady(DeviceAddr, Trials) == HAL_OK);
}

uint8_t HMC5883L_ReadStatus(uint16_t DeviceAddr)
{
	uint8_t	tmp = 0;

	HMC5883L_IO_Read(DeviceAddr, &tmp, 0x09, 1);

	return (uint8_t)(tmp);
}

uint8_t HMC5883L_ReadDst(uint16_t DeviceAddr, uint8_t DstAddr)
{
	uint8_t tmp = 0;

	HMC5883L_IO_Read(DeviceAddr, &tmp, DstAddr, 1);

	return tmp;
}

void HMC5883L_ReadAngle(uint16_t DeviceAddr, HMC_Measure *xValue)
{
	int16_t x, y, z, retValue;
	uint8_t outputData[6];
	double angle;

	HMC5883L_IO_Read(DeviceAddr, outputData, 0x03, 6);

	xValue->x = outputData[0] << 8 | outputData[1];
	xValue->y = outputData[2] << 8 | outputData[3];
	xValue->z = outputData[4] << 8 | outputData[5];	
}