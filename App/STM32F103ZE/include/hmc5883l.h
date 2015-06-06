#ifndef __HMC5883L_H_
#define	__HMC5883L_H_
#include "stm32f1xx_hal_def.h"

typedef struct
{
	uint8_t REG_A;
	uint8_t REG_B;
	uint8_t REG_M;
} HMC5883L_Conf_Reg;

typedef struct
{
	int16_t	x;
	int16_t	y;
	int16_t	z;
	int16_t	angle;
}HMC_Measure;

extern void HMC5883L_Init(uint16_t DeviceAddr);
extern uint8_t HMC5883L_IsReady(uint16_t DeviceAddr, uint32_t Trials);
extern uint8_t HMC5883L_ReadStatus(uint16_t DeviceAddr);
extern uint8_t HMC5883L_ReadDst(uint16_t DeviceAddr, uint8_t DstAddr);
extern void HMC5883L_ReadAngle(uint16_t DeviceAddr, HMC_Measure *xValue);
#endif