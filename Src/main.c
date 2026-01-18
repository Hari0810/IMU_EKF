/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FIRFilter.h"
#include "EKF.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
SemaphoreHandle_t dmaSemaphore;
FIRFilter lpfAccX,lpfAccY,lpfAccZ, lpfGyrX, lpfGyrY, lpfGyrZ;


/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskAccel */
osThreadId_t myTaskAccelHandle;
const osThreadAttr_t myTaskAccel_attributes = {
  .name = "myTaskAccel",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskGyro */
osThreadId_t myTaskGyroHandle;
const osThreadAttr_t myTaskGyro_attributes = {
  .name = "myTaskGyro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
LSM imu;
EKF ekf;
uint8_t flag = 0;
float EKF_PREDICT_PERIOD = 8.33f;
float EKF_UPDATE_PERIOD = 83.3f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartTaskAccel(void *argument);
void StartTaskGyro(void *argument);

/* USER CODE BEGIN PFP */

/*Code to redirect printf contents to USB*/

int _write(int file, char *ptr, int len)
{
  CDC_Transmit_FS((uint8_t *)ptr, len);
  return len;
}

/*SPI Polling transactions to manually read/write register values*/

 uint8_t LSM_ReadRegister(uint8_t addr, uint8_t *data){

 	uint8_t txBuf[2] = {(addr| 0x80), 0x00};
 	uint8_t rxBuf[2];

 	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
 	uint8_t status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
 	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

 	*data = rxBuf[1];

 	return status;
 }

 uint8_t LSM_WriteRegister(uint8_t addr, uint8_t data){

 	uint8_t txBuf[2] = {addr, data};

 	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
 	uint8_t status = (HAL_SPI_Transmit(&hspi1, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
 	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

 	return status;
 }


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* DMA Functions */
/* The IMU has two separate interrupt pins, */
/*hence accelerometer and gyro data can be treated separately if desired*/

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
 	if (GPIO_Pin == ACC_INT_Pin) {

 		LSM_ReadAccelDMA(&imu);
     imu.readingAcc = 1;
 	}

   if (GPIO_Pin == GYR_INT_Pin) {
 		LSM_ReadGyroDMA(&imu);
     imu.readingGyr = 1;
 	}
 }

 void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
 	if ((hspi->Instance == SPI1)&&imu.readingAcc) {
 		LSM_ReadAccelDMA_Complete(&imu);
 		// float AccX = FIRFilter_Update(&lpfAccX, imu.acc_mps2[0]);
 		// float AccY = FIRFilter_Update(&lpfAccY, imu.acc_mps2[1]);
 		// float AccZ = FIRFilter_Update(&lpfAccZ, imu.acc_mps2[2]);

 		//printf("Acc: %.8f, %.8f \r\n", AccX, lpfAccX.out);

     imu.readingAcc = 0;
 	}

   if ((hspi->Instance == SPI1)&&imu.readingGyr) {
 		LSM_ReadGyroDMA_Complete(&imu);
// 		float GyrX = FIRFilter_Update(&lpfGyrX, imu.gyr_mps2[0]);
//		float GyrY = FIRFilter_Update(&lpfGyrY, imu.gyr_mps2[1]);
//		float GyrZ = FIRFilter_Update(&lpfGyrZ, imu.gyr_mps2[2]);

		//printf("Gyr: %.8f, %.8f \r\n", GyrX, imu.gyr_mps2[0]);
     imu.readingGyr = 0;
 	}

 }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  LSM_Initialise(&imu, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
  FIRFilter_Init(&lpfAccX);
  FIRFilter_Init(&lpfAccY);
  FIRFilter_Init(&lpfAccZ);


  LSM_WriteRegister(0x12, 0x01);  // IMU Reset
  HAL_Delay(100);

  LSM_WriteRegister(0x12, 0x44);  // Enable auto-increment for bulk register reads

  //OD = INT1, 0E = INT2, 0b01 = ACC, 0b10 = GYR
  LSM_WriteRegister(0x62, 0x01);
  LSM_WriteRegister(0x10, 0x06);  // Accel at 120 Hz
  LSM_WriteRegister(0x11, 0x06);  // Gyro at 120 Hz

  LSM_WriteRegister(0x0D, 0x01);  // INT1 for Accel
  LSM_WriteRegister(0x0E, 0x02);  // INT2 for Gyro


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskAccel */
  myTaskAccelHandle = osThreadNew(StartTaskAccel, NULL, &myTaskAccel_attributes);

  /* creation of myTaskGyro */
  myTaskGyroHandle = osThreadNew(StartTaskGyro, NULL, &myTaskGyro_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ACC_INT_Pin GYR_INT_Pin */
  GPIO_InitStruct.Pin = ACC_INT_Pin|GYR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  printf("PHI: %.4f THETA %.4f \r\n", ekf.phi_r,ekf.theta_r);
	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskAccel */
/**
* @brief Function implementing the myTaskAccel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskAccel */
void StartTaskAccel(void *argument)
{
  /* USER CODE BEGIN StartTaskAccel */
  /* Infinite loop */
  for(;;)
  {
	  //printf("Acc: %d %d %d\r\n", imu.acc_mps2[0],imu.acc_mps2[1],imu.acc_mps2[2]);
	  EKF_Update(&ekf, lpfAccX.out, lpfAccY.out, lpfAccZ.out);
	  //printf("PHI: %.4f THETA %.4f \r\n", ekf.phi_r,ekf.theta_r);
	  osDelay(EKF_UPDATE_PERIOD);
  }
  /* USER CODE END StartTaskAccel */
}

/* USER CODE BEGIN Header_StartTaskGyro */
/**
* @brief Function implementing the myTaskGyro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGyro */
void StartTaskGyro(void *argument)
{
  /* USER CODE BEGIN StartTaskGyro */
  /* Infinite loop */
  for(;;)
  {
	  //printf("Gyr: %d %d %d\r\n", imu.gyr_mps2[0],imu.gyr_mps2[1],imu.gyr_mps2[2]);
	  EKF_Predict(&ekf, lpfGyrX.out, lpfGyrY.out, lpfGyrZ.out, 0.0001f * EKF_PREDICT_PERIOD);
	  osDelay(EKF_PREDICT_PERIOD);
  }
  /* USER CODE END StartTaskGyro */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
