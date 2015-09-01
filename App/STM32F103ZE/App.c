/* Standard includes. */
#include <stdio.h>
#include <math.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "main.h"

/* Library includes. */
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"

/* Demo app includes. */
#include "stm3210e_bit3.h"
#include "flash.h"
#include "serial.h"
#include "hmc5883l.h"
#include "pwm.h"
#include "serial.h"
#include "nrf24l01.h"
#include "mpu9050.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainRF_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )

// EventGroup
EventGroupHandle_t xEventGruop = NULL;

static portTASK_FUNCTION_PROTO( vHMC5883LTask, pvParameters );
static portTASK_FUNCTION_PROTO( vMPU9050Task, pvParameters );
static portTASK_FUNCTION( vnRFTask, pvParameters );
static portTASK_FUNCTION( vDMPTask, pvParameters );

void StartApp(void)
{
	if ((xEventGruop = xEventGroupCreate()) != NULL)
	{
		vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
		//xTaskCreate( vHMC5883LTask, "HMC5883L", configMINIMAL_STACK_SIZE, NULL, 1, ( TaskHandle_t * ) NULL );
		//xTaskCreate( vMPU9050Task, "MPU", configMINIMAL_STACK_SIZE, NULL, 1, ( TaskHandle_t * ) NULL );
		xTaskCreate( vnRFTask, "nRF", configMINIMAL_STACK_SIZE*2, NULL, mainRF_TASK_PRIORITY, ( TaskHandle_t * ) NULL );
		xTaskCreate( vDMPTask, "DMP", configMINIMAL_STACK_SIZE*5, NULL, 1, ( TaskHandle_t * ) NULL );
		
		/* Start the scheduler. */
		vTaskStartScheduler();
	}
}

static portTASK_FUNCTION( vHMC5883LTask, pvParameters )
{
	TickType_t xRate, xLastTime;
	uint8_t isConnect;
	HMC_Data_t hmc_data;

	/* The parameters are not used. */
	( void ) pvParameters;
	
	xRate = 500;
	xRate /= portTICK_PERIOD_MS;
	
	/* We need to initialise xLastFlashTime prior to the first call to 
	vTaskDelayUntil(). */
	xLastTime = xTaskGetTickCount();

	isConnect = 10;

	for(;;)
	{
		if (isConnect < 10)
		{
			if (HMC5883L_ReadAngle(&hmc_data))
			{
				printf("\nx: %d\ty: %d\tz: %d\t", hmc_data.direct.x, hmc_data.direct.y, hmc_data.direct.z);
				printf("angle_x: %.2f\tangle_y: %.2f\tangle_z: %.2f\n", hmc_data.angle.x, hmc_data.angle.y, hmc_data.angle.z);
				BSP_LED_Toggle(LED2);
			}
			else
				isConnect++;
		}
		else
		{
			vTaskDelayUntil(&xLastTime, 500);
			if (HMC5883L_IsReady(10))
			{
				HMC5883L_Init();
				isConnect = 0;
				printf("HMC5883L Init ... \n");
			}
			else
				printf("HMC5883L not Connect ...\n");
		}
	}
}


static portTASK_FUNCTION( vMPU9050Task, pvParameters )
{
	mpu9050_t mpu9050;
	BaseType_t uxBits;
	const TickType_t xTickToWait = 100;		// Time Out one second.

	/* The parameters are not used. */
	( void ) pvParameters;
	MPU9050_Init();

	for(;;)
	{
	
		uxBits = xEventGroupWaitBits(xEventGruop, MPU_DATA_READY, pdTRUE, pdFALSE, xTickToWait);
		BSP_LED_Off(LED2);
		if (uxBits & MPU_DATA_READY)
		{
			MPU9050_Read(&mpu9050);
			BSP_LED_On(LED2);
			//printf("accel:\tx= %d\ty= %d\tz= %d\n", mpu9050.accel.x, mpu9050.accel.y, mpu9050.accel.z);
			//printf("gyro:\tx= %d\ty= %d\tz= %d\n", mpu9050.gyro.x, mpu9050.gyro.y, mpu9050.gyro.z);
		}
	}
}



static portTASK_FUNCTION( vDMPTask, pvParameters )
{
extern struct hal_s hal;
    uint32_t sensor_timestamp;

	/* The parameters are not used. */
	( void ) pvParameters;

	for(;;)
	{
		vTaskDelay(1);
		if (hal.new_gyro && hal.dmp_on) 
		{
			BSP_LED_Toggle(LED2);
	        short gyro[3], accel[3], sensors;
	        unsigned char more;
	        long quat[4];
	        /* This function gets new data from the FIFO when the DMP is in
	         * use. The FIFO can contain any combination of gyro, accel,
	         * quaternion, and gesture data. The sensors parameter tells the
	         * caller which data fields were actually populated with new data.
	         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
	         * the FIFO isn't being filled with accel data.
	         * The driver parses the gesture data to determine if a gesture
	         * event has occurred; on an event, the application will be notified
	         * via a callback (assuming that a callback function was properly
	         * registered). The more parameter is non-zero if there are
	         * leftover packets in the FIFO.
	         */
	        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
	            &more);
	        if (!more)
	            hal.new_gyro = 0;
	        /* Gyro and accel data are written to the FIFO by the DMP in chip
	         * frame and hardware units. This behavior is convenient because it
	         * keeps the gyro and accel outputs of dmp_read_fifo and
	         * mpu_read_fifo consistent.
	         */
	        if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
	            send_packet(PACKET_TYPE_GYRO, gyro);
	        if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
	            send_packet(PACKET_TYPE_ACCEL, accel);
	        /* Unlike gyro and accel, quaternions are written to the FIFO in
	         * the body frame, q30. The orientation is set by the scalar passed
	         * to dmp_set_orientation during initialization.
	         */
	        if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
	            send_packet(PACKET_TYPE_QUAT, quat);
	    } 
	    else if (hal.new_gyro) 
	    {
	        short gyro[3], accel[3];
	        unsigned char sensors, more;
	        /* This function gets new data from the FIFO. The FIFO can contain
	         * gyro, accel, both, or neither. The sensors parameter tells the
	         * caller which data fields were actually populated with new data.
	         * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
	         * being filled with accel data. The more parameter is non-zero if
	         * there are leftover packets in the FIFO.
	         */
	        mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
	        if (!more)
	            hal.new_gyro = 0;
	        if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
	            send_packet(PACKET_TYPE_GYRO, gyro);
	        if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
	            send_packet(PACKET_TYPE_ACCEL, accel);
	    }
	}
}

uint8_t CommProcess(nRF_Tx_DataType *nRF_Buf)
{

	switch(nRF_Buf->datatype)
	{
		case DataType_Key:
			return PWM_Ctr_Dir((PWM_Ctr_Type *)nRF_Buf);
		default:
			return HAL_ERROR;		
	}

	return HAL_ERROR;
}

static portTASK_FUNCTION( vnRFTask, pvParameters )
{
	nRF_Tx_DataType nRF_Buf;
	mpu9050_t mpu9050;
	mpu9050_data_t mpu9050_data;
	HMC_Data_t hmc_data;
	/* The parameters are not used. */
	( void ) pvParameters;

	nRF_RX_Mode();
	
	for(;;)
	{	
		if (nRF_Start_Rx((uint8_t *)&nRF_Buf, 32) == nRF_RX_OK)
		{
			BSP_LED_Toggle(LED2);
			if (nRF_Buf.datatype == 0x02)
			{
				memcpy((uint8_t *)&mpu9050, (uint8_t *)&nRF_Buf.data[0], sizeof(mpu9050));
				MPU9050_Data_Cal(&mpu9050_data, &mpu9050);
				printf("x= %d  y= %d  z= %d  x= %.2f  y= %.2f  ", mpu9050.accel.x, mpu9050.accel.y, mpu9050.accel.z, mpu9050_data.accel.x, mpu9050_data.accel.y/*, mpu9050_data.accel.z*/);
				printf("x= %.2f  y= %.2f  z= %.2f  ", mpu9050_data.gyro.x, mpu9050_data.gyro.y, mpu9050_data.gyro.z);
				printf("x= %d  y= %d  z= %d\n", mpu9050.gyro.x, mpu9050.gyro.y, mpu9050.gyro.z);
			}
			else if (nRF_Buf.datatype == 0x04)
			{
				memcpy((uint8_t *)&hmc_data.direct.x, (uint8_t)&nRF_Buf.data[0], sizeof(direct_t));
				HMC_Data_Cal(&hmc_data);
				printf("x= %d y= %d z= %d x= %.2f y= %.2f z= %.2f\n", hmc_data.direct.x, hmc_data.direct.y, hmc_data.direct.z, hmc_data.angle.x, hmc_data.angle.y, hmc_data.angle.z);
			}
			/*if (CommProcess(&nRF_Buf) == HAL_OK)
			{
				nRF_Buf.datatype |= 0x08;
				if (nRF_Start_Tx() == nRF_TX_OK)
				{
					BSP_LED_Toggle(LED2);
				}
			}*/
		}
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

