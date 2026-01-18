#include "LSM.h"

uint8_t flagg = 0;

uint8_t LSM_Initialise(LSM *imu,
					SPI_HandleTypeDef *spiHandle,
					GPIO_TypeDef *csIMUPinBank,
					uint16_t csIMUPin){

	imu->spiHandle		= spiHandle;
	imu->csIMUPinBank	= csIMUPinBank;
	imu->csIMUPin		= csIMUPin;

	imu->readingAcc = 0;
	imu->readingGyr = 0;

	//Init procedure 
	
	HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_SET);
	HAL_Delay(50);

	return(100);

}

uint8_t LSM_ReadAccelDMA(LSM *imu){
	//printf("triggdrere \r\n");

	//txBuf[0] = BMI_ACC_DATA | 0x80;

	imu->accTxBuf[0] = 0x28 | 0x80;   // 0x28 with read bit set
	imu->accTxBuf[1] = 0x00;
	imu->accTxBuf[2] = 0x00;
	imu->accTxBuf[3] = 0x00;
	imu->accTxBuf[4] = 0x00;
	imu->accTxBuf[5] = 0x00;
	imu->accTxBuf[6] = 0x00;


		HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_RESET);


		if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->accTxBuf, imu->accRxBuf, 7) == HAL_OK){
			return 1;
		}
		else{
			HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_SET);
			//printf("NOTok");
			return 0;
		}

}

void LSM_ReadAccelDMA_Complete(LSM *imu) {
	//printf("triggd \r\n");
	HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_SET);

	float accX = (float) ((imu->accRxBuf[2] << 8) | imu->accRxBuf[1]);
	float accY = (float) ((imu->accRxBuf[4] << 8) | imu->accRxBuf[3]);
	float accZ = (float) ((imu->accRxBuf[6] << 8) | imu->accRxBuf[5]);

	imu->acc_mps2[0] = accX;
	imu->acc_mps2[1] = accY; //imu->accConversion * accY;
	imu->acc_mps2[2] = accZ; //imu->accConversion * accZ;


}

uint8_t LSM_ReadGyroDMA(LSM *imu){

	//printf("A\r\n");

	//txBuf[0] = BMI_ACC_DATA | 0x80;
	imu->gyrTxBuf[0] = 0x22 | 0x80;   // 0x22 with read bit set
	imu->gyrTxBuf[1] = 0x00;
	imu->gyrTxBuf[2] = 0x00;
	imu->gyrTxBuf[3] = 0x00;
	imu->gyrTxBuf[4] = 0x00;
	imu->gyrTxBuf[5] = 0x00;
	imu->gyrTxBuf[6] = 0x00;

	HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_RESET);

		if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->gyrTxBuf, imu->gyrRxBuf, 7) == HAL_OK){
			//printf("B\r\n");
			return 1;
		}
		else{
			HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_SET);
			//printf("C\r\n");

			return 0;
		}

}

void LSM_ReadGyroDMA_Complete(LSM *imu) {
	//printf("D\r\n");
	HAL_GPIO_WritePin(imu->csIMUPinBank, imu->csIMUPin, GPIO_PIN_SET);

	float gyrX = (float) ((imu->gyrRxBuf[2] << 8) | imu->gyrRxBuf[1]);
	float gyrY = (float) ((imu->gyrRxBuf[4] << 8) | imu->gyrRxBuf[3]);
	float gyrZ = (float) ((imu->gyrRxBuf[6] << 8) | imu->gyrRxBuf[5]);

	imu->gyr_mps2[0] = gyrX;
	imu->gyr_mps2[1] = gyrY; 
	imu->gyr_mps2[2] = gyrZ; 

}
