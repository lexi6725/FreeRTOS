#ifndef __HMC5883L_H_
#define	__HMC5883L_H_
#include "stm32f1xx_hal_def.h"

#define HMC5883L_SlaveAddr	0x3C

typedef struct
{
	uint8_t REG_A;
	uint8_t REG_B;
	uint8_t REG_M;
} HMC5883L_Conf_Reg;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} direct_t;

typedef struct
{
	float x;
	float y;
	float z;
} angle_t;

typedef struct
{
	direct_t direct;
	angle_t angle;
}HMC_Data_t;

void HMC5883L_Init(void);
uint8_t HMC5883L_IsReady(uint32_t Trials);
uint8_t HMC5883L_ReadStatus(void);
uint8_t HMC5883L_ReadDst(uint8_t DstAddr);
uint8_t HMC5883L_ReadAngle(HMC_Data_t *xValue);
#endif