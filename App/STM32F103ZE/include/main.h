/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Inc/main.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-December-2014
  * @brief   This file contains all the functions prototypes for the main.c
  *          file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm3210e_bit3.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_Channel		DMA1_Channel7
#define USARTx_DMA_RX_Channel		DMA1_Channel6
#define USARTx_DMA_TX_IRQn			DMA1_Channel7_IRQn
#define USARTx_DMA_RX_IRQn			DMA1_Channel6_IRQn
#define USARTx_DMA_TX_IRQHandler	DMA1_Channel7_IRQHandler
#define	USARTx_DMA_RX_IRQHandler	DMA1_Channel6_IRQHandler

extern EventGroupHandle_t xEventGruop;
#define nRF_State_TX_OK		(0x01<<0)		// 1. 发送数据成功
#define nRF_State_TX_MAX	(0x01<<1)		// 1. 发送数据最大次数
#define nRF_State_RX_OK		(0x01<<2)		// 1. 接收数据成功
#define HMC_DATA_READY		(0x01<<3)
#define MPU_DATA_READY		(0x01<<4)

#ifdef __cplusplus
}
#endif


#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
