/*
 * LSM.h
 *
 *  Created on: Sep 24, 2025
 *      Author: hari0
 */

#ifndef INC_LSM_H_
#define INC_LSM_H_

#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "task.h"
//#include "timers.h"
#include "queue.h"
#include "semphr.h"
//#include "event_groups.h"

typedef struct {
	SPI_HandleTypeDef 	*spiHandle;
	GPIO_TypeDef	  	*csAccPinBank;
	GPIO_TypeDef		*csGyrPinBank;
	uint16_t			*csAccPin;
	uint16_t			*csGyrPin;

	int8_t accRxBuf[7]; 
	int8_t accTxBuf[7]; 
	int8_t gyrRxBuf[7]; 
	int8_t gyrTxBuf[7]; 

	//acc data xyz ms-2

	float accConversion;
	float acc_mps2[3];
	float gyrConversion;
	float gyr_mps2[3];

	uint8_t readingAcc;
	uint8_t readingGyr;

	float temp_C;
} LSM;

//init

uint8_t LSM_Initialise(LSM *imu,
					SPI_HandleTypeDef *spiHandle,
					GPIO_TypeDef *csAccPinBank,
					uint16_t csAccPin);
//data aqq

//HAL_StatusTypeDef LSM_ReadTemp(LSM *dev);
uint8_t LSM_ReadAccelDMA(LSM *imu);
void LSM_ReadAccelDMA_Complete(LSM *imu);
uint8_t LSM_ReadGyroDMA(LSM *imu);
void LSM_ReadGyroDMA_Complete(LSM *imu);

#endif /* INC_LSM_H_ */
