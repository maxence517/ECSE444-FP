
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "stm32l475e_iot01_qspi.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const int nsignals = 2;
const int nsamples = 32000;
const int buf_size = 100;
const int ntransfers = nsamples / buf_size;

const uint32_t S0_START_ADDR = 0;
const uint32_t S1_START_ADDR = nsamples * sizeof(float);
const uint32_t S0W_START_ADDR = nsamples * sizeof(float) + S1_START_ADDR;
const uint32_t S1W_START_ADDR = nsamples * sizeof(float) + S0W_START_ADDR;


const uint32_t DATA_WIDTH = (uint32_t)buf_size*sizeof(float);

const float f[] = {261.63, 392.0};
const uint32_t sampling_freq = 16000;
const int scale_coeff = 50;

float ica_fltr[2][2];
float a[4] = {2, 4, 5, 3};
float mean[2];

arm_matrix_instance_f32 matrix_a = {
	.numRows = 2,
	.numCols = 2,
	.pData = a
};

int tim3_flag = 0;

//Signals local memory Buffers
float signal[nsignals][buf_size];
float signal_w[nsignals][buf_size];
float signal_x[nsignals][buf_size];
float test_signal[buf_size];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DAC1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
  while (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 30000));
  return ch;
}
int fgetc(FILE *f) {
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 30000));
  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DFSDM1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
	BSP_QSPI_Init();
	if(BSP_QSPI_Erase_Chip()!=QSPI_OK) printf("Error erasing chip");
	if(BSP_QSPI_Erase_Chip()==QSPI_OK) printf("chip erased successfully");
	
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
	

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 32;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config
    */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config
    */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DFSDM1 init function */
static void MX_DFSDM1_Init(void)
{

  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = DISABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 128;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = DISABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 128;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel1.Init.OutputClock.Divider = 32;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = -1152;
  hdfsdm1_channel1.Init.RightBitShift = 0x0D;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel2.Init.OutputClock.Divider = 32;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = -1152;
  hdfsdm1_channel2.Init.RightBitShift = 0x0D;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{
	
	  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void fast_ica(int niter, float epsilon)
{	
	int idx, iter;
	uint32_t offset;
	float tmp[3], cov[2][2], tr, det, sqrt, eigvals[2], ev[2][2][2], eigvec[2][2], eigval[2][2], w[2][2], eig_inv[2][2], inv_tmp, dw[4],matrix_w[2][2], weight[2], old_weight[2], matrix_dw[2][2], vector_dw[2][2], basis_set[2][2];

	// Compute the mean
  mean[0] = 0.0f;
	mean[1] = 0.0f;
  for(int i = 0; i < nsamples; i++)
  {
		
    idx = i % buf_size;

    if(idx == 0)
    {
      // read to buffer from flash
      offset = (i / buf_size) * buf_size * sizeof(float);
      BSP_QSPI_Read((uint8_t *)signal[0], S0_START_ADDR + offset, DATA_WIDTH);
      BSP_QSPI_Read((uint8_t *)signal[1], S1_START_ADDR + offset, DATA_WIDTH);
    }

		// CMA of signal 0 & 1
    mean[0] += (signal[0][idx] - mean[0]) / (i + 1);
    mean[1] += (signal[1][idx] - mean[1]) / (i + 1);
  }
	
  for(int i = 0; i < nsamples; i++)
  {
		
    idx = i % buf_size;

    if(idx == 0)
    {
      // read to buffer from flash
      offset = (i / buf_size) * buf_size * sizeof(float);
      BSP_QSPI_Read((uint8_t *)signal[0], S0_START_ADDR + offset, DATA_WIDTH);
      BSP_QSPI_Read((uint8_t *)signal[1], S1_START_ADDR + offset, DATA_WIDTH);
    }

		// center values
    signal[0][idx] -= mean[0];
    signal[1][idx] -= mean[1];
		
    tmp[0] += (signal[0][idx] * signal[0][idx]); // sum(x_i * x_i)
    tmp[1] += (signal[1][idx] * signal[1][idx]); // sum(y_i * y_i)
    tmp[2] += (signal[0][idx] * signal[1][idx]); // sum(x_i * y_i)
  }
	
	// cov_mat
	// [ tmp0 tmp2 ]
	// [ tmp2 tmp1 ]
	// Obtain Variance and Covariance Matrice
  tmp[0] /= (nsamples - 1); // Why minus 1
  tmp[1] /= (nsamples - 1);
  tmp[2] /= (nsamples - 1);
	
	
	cov[0][0] = tmp[0];
	cov[0][1] = tmp[2];
	cov[1][0] = tmp[2];
	cov[1][1] = tmp[1];
	
	// Find eigenvector and eigen value of the Covariance 
	
	// trace of 2x2 matrix
	tr = cov[0][0] + cov[1][1]; 
	// determinant of 2x2 matrix
	det = (cov[0][0] * cov[1][1]) - (cov[0][1] * cov[1][0]); 
	
	arm_sqrt_f32((tr * tr) - (4.0f * det), &sqrt);
	eigvals[0] = (tr + sqrt) / 2.0f;
	eigvals[1] = (tr - sqrt) / 2.0f;
	
	//EV1
	ev[0][0][0] = cov[0][0] - eigvals[0];
	ev[0][0][1] = cov[0][1];
	ev[0][1][0] = cov[1][0];
	ev[0][1][1] = cov[1][1] - eigvals[1];	
	//EV2
	ev[1][0][0] = cov[0][0] - eigvals[1];
	ev[1][0][1] = cov[0][1];
	ev[1][1][0] = cov[1][0];
	ev[1][1][1] = cov[1][1] - eigvals[1];
	
	// [ cov[0][0]-eigval1	cov[0][0]-eigval2 ]
	// [ cov[1][0]-0				cov[1][0]-0				]
	eigvec[0][0] = ev[0][0][0];	// cov[0][0] - eigvals[0]
	eigvec[0][1] = ev[1][0][0];	// cov[0][0] - eigvals[1];
	eigvec[1][0] = ev[0][1][0];	// cov[1][0]
	eigvec[1][1] = ev[1][1][0];	// cov[1][0]
	
	//Diagonal Matrice of Eigen Values
	// [ eigval2	0				]
	// [ 0				eigval1 ]
	eigval[0][0] = eigvals[1];
	eigval[0][1] = 0.0f;
	eigval[1][0] = 0.0f;
	eigval[1][1] = eigvals[0];
	
	// Divide by norm : eigvec = eigvec / np.linalg.norm(eigvec, axis=0)
	// ***ERROR CORRECTED
	// assume eigvec is
	// [ a b ]
	// [ c d ]
	// np.linalg.norm(eigvec, axis=0) returns
	// [sqrt(a^2+c^2), sqrt(b^2+d^2)]
	//
	// eigvec / np.linalg.norm(eigvec, axis=0) returns
	// [ a/sqrt(a^2+c^2)	b/sqrt(b^2+d^2) ]
	// [ c/sqrt(a^2+c^2)	c/sqrt(b^2+d^2) ]
	
	//-------------OLD CODE-------------//
	//arm_sqrt_f32((eigvec[0][0] * eigvec[0][0]) + (eigvec[0][1] * eigvec[0][1]), &sqrt);
	//eigvec[0][0] /= sqrt;
	//eigvec[0][1] /= sqrt;
	//arm_sqrt_f32((eigvec[1][0] * eigvec[1][0]) + (eigvec[1][1] * eigvec[1][1]), &sqrt);
	//eigvec[1][0] /= sqrt;
	//eigvec[1][1] /= sqrt;	
	//----------------------------------//
	arm_sqrt_f32((eigvec[0][0] * eigvec[0][0]) + (eigvec[1][0] * eigvec[1][0]), &sqrt);
	eigvec[0][0] /= sqrt;
	eigvec[1][0] /= sqrt;
	arm_sqrt_f32((eigvec[0][1] * eigvec[0][1]) + (eigvec[1][1] * eigvec[1][1]), &sqrt);
	eigvec[0][1] /= sqrt;
	eigvec[1][1] /= sqrt;
	
	//De-Whitening
	// np.sqrt(eig_val)
	// since eig_val is in form of
	// [ a 0 ]
	// [ 0 b ]
	// where a = eigval2 and b = eigval1
	arm_sqrt_f32(eigval[0][0],  &eig_inv[0][0]);
	arm_sqrt_f32(eigval[0][1],  &eig_inv[0][1]);	// is always 0
	arm_sqrt_f32(eigval[1][0],  &eig_inv[1][0]);	// is always 0
	arm_sqrt_f32(eigval[1][1],  &eig_inv[1][1]);
	
	// dewhitening_mat = eig_vec * np.sqrt(eig_val)
	// [ vec00 vec01 ]	[	inv00 inv01 ]
	// [ vec10 vec11 ]	[ inv10 inv11 ]
	// where inv01 = 0 and inv10 = 0
	//-------------OLD CODE-------------//
	//matrix_dw[0][0] = eig_inv[0][0] * eigvec[0][0] + eig_inv[0][1] * eigvec[1][0];
	//matrix_dw[0][1] = eig_inv[0][0] * eigvec[0][1] + eig_inv[0][1] * eigvec[1][1];
	//matrix_dw[1][0] = eig_inv[1][0] * eigvec[0][0] + eig_inv[1][1] * eigvec[1][0];
	//matrix_dw[1][1] = eig_inv[1][0] * eigvec[0][1] + eig_inv[1][1] * eigvec[1][1];
	//-------------OLD CODE-------------//
	// ***CORRECTED
	matrix_dw[0][0] = eigvec[0][0] * eig_inv[0][0] + eigvec[0][1] * eig_inv[1][0];	// eigvec[0][0] * eig_inv[0][0] <- inv10 = 0
	matrix_dw[0][1] = eigvec[0][0] * eig_inv[0][1] + eigvec[0][1] * eig_inv[1][1];	// eigvec[0][1] * eig_inv[1][1] <- inv01 = 0
	matrix_dw[1][0] = eigvec[1][0] * eig_inv[0][0] + eigvec[1][1] * eig_inv[1][0];	// eigvec[1][0] * eig_inv[0][0] <- inv10 = 0
	matrix_dw[1][1] = eigvec[1][0] * eig_inv[0][1] + eigvec[1][1] * eig_inv[1][1];	// eigvec[1][1] * eig_inv[1][1] <- inv01 = 0
	
	//Whitening
	
	// eig_inv[0][0] * eig_inv[1][1] is enough since the second term is 0
	det = eig_inv[0][0] * eig_inv[1][1] - eig_inv[0][1] * eig_inv[1][0];
	
	// np.linalg.inv(np.sqrt(eig_val))
	inv_tmp = eig_inv[0][0];
	eig_inv[0][0] = (1.0 / det) * eig_inv[1][1];
	eig_inv[0][1] *= (-1.0 / det); //** Technically not necessary because of diagonal matrice **//	is 0
	eig_inv[1][0] *= (-1.0 / det); //** Technically not necessary because of diagonal matrice **//	is 0
	eig_inv[1][1] = (1.0 / det) * inv_tmp;

	// whitening_mat
	// [ inv00 inv01 ]	[ vec00 vec10 ] 
	// [ inv10 inv11 ]	[ vec01 vec11 ]
	matrix_w[0][0] = eig_inv[0][0] * eigvec[0][0] + eig_inv[0][1] * eigvec[0][1];	// eig_inv[0][0] * eigvec[0][0] + 0
	matrix_w[0][1] = eig_inv[0][0] * eigvec[1][0] + eig_inv[0][1] * eigvec[1][1];	// eig_inv[0][0] * eigvec[1][0] + 0
	matrix_w[1][0] = eig_inv[1][0] * eigvec[0][0] + eig_inv[1][1] * eigvec[0][1];	// 0 + eig_inv[1][1] * eigvec[0][1]
	matrix_w[1][1] = eig_inv[1][0] * eigvec[1][0] + eig_inv[1][1] * eigvec[1][1]; // 0 + eig_inv[1][1] * eigvec[1][1]
	
	// Apply whitening to center matrice 
	for(int i = 0; i < nsamples; i++)
  {
		
    idx = i % buf_size;

    if(idx == 0)
    {
      // read to buffer from flash
      offset = (i / buf_size) * buf_size * sizeof(float);
      BSP_QSPI_Read((uint8_t *)signal[0], S0_START_ADDR + offset, DATA_WIDTH);
      BSP_QSPI_Read((uint8_t *)signal[1], S1_START_ADDR + offset, DATA_WIDTH);
    }

		// center values
    signal[0][idx] -= mean[0];
    signal[1][idx] -= mean[1];
		
		// Whitening Center Signals
		// white_mat
		signal_w[0][idx] = matrix_w[0][0] * signal[0][idx] + matrix_w[0][1] * signal[1][idx];
    signal_w[1][idx] = matrix_w[1][0] * signal[0][idx] + matrix_w[1][1] * signal[1][idx];
		
		//Write the new 2 x 32 000 to the QSPI
		if(idx == buf_size -1){
			offset = ((i / (buf_size-1)) - 1) * buf_size * sizeof(float);
			BSP_QSPI_Write((uint8_t *)signal_w[0], S0W_START_ADDR + offset, DATA_WIDTH);
			BSP_QSPI_Write((uint8_t *)signal_w[1], S1W_START_ADDR + offset, DATA_WIDTH);
		}
  }
	
	// values working
	// Initialize random weight
	weight[0] = 5;
	weight[1] = 10;
	// Normalize the weight
	weight[0] = 5/15;
	weight[1] = 10/15;
	
	// History of matrices
	old_weight[0] = 0;
	old_weight[1] = 0;
	
	iter = 0;
	while(iter < niter)
	{
		//Test for convergence
		float norm1, norm2;
		arm_sqrt_f32((weight[0]- old_weight[0]) * (weight[0]- old_weight[0])  + (weight[1] - old_weight[1])*(weight[1] - old_weight[1]),  &norm1);
		arm_sqrt_f32((weight[0]+ old_weight[0]) * (weight[0]+ old_weight[0])  + (weight[1] + old_weight[1])*(weight[1] + old_weight[1]),  &norm2);
		
		if( norm1 < epsilon || norm2 < epsilon){
			printf("Converge\n");
			break;
		}
		
		// update old weight
		old_weight[0] = weight[0];
		old_weight[1] = weight[1];
		
		// update weight
		for(int i = 0; i < nsamples; i++){
		
			idx = i % buf_size;

			if(idx == 0)
			{
				// read Whiten Center signal to buffer from flash
				offset = (i / buf_size) * buf_size * sizeof(float);
				BSP_QSPI_Read((uint8_t *)signal_w[0], S0W_START_ADDR + offset, DATA_WIDTH);
				BSP_QSPI_Read((uint8_t *)signal_w[1], S1W_START_ADDR + offset, DATA_WIDTH);
			}
		
			// Equivalent of : weight = (white_mat * np.power(white_mat.T * weight, 3)) / num_sample - 3 * weight
			// np.power(white_mat.T * weight, 3)
			// [num_sample x 2]	[2 x 1] -> white_mat.T * weight is a [num_sample x 1] matrix
			signal_x[0][idx] = pow(signal_w[0][idx] * weight[0] + signal_w[1][idx] * weight[1], 3);
			
			// white_mat * np.power(white_mat.T * weight, 3) -> [2x1] matrix
			signal_x[1][0] += signal_w[0][idx] * signal_x[0][idx];
			signal_x[1][1] += signal_w[1][idx] * signal_x[0][idx];
			
			if(idx == buf_size -1){
				weight[0] += signal_x[1][0]; // / (buf_size -1) - weight[0]*3 ;
				weight[1] += signal_x[1][1]; // / (buf_size -1) - weight[1]*3 ;
			}
		}
		weight[0] = weight[0]/nsamples - old_weight[0] * 3;
		weight[1] = weight[1]/nsamples - old_weight[1] * 3;
		
		// Normalize Weight
		weight[0] = weight[0]/pow((pow(weight[0],2) + pow(weight[1],2)), 1/2);
		weight[1] = weight[1]/pow((pow(weight[0],2) + pow(weight[1],2)), 1/2);
	}
	
	// basis_set[:, 0] = weight
	// assume weight is
	// [ a ]
	// [ b ]
	// basis_set[:, 0] = weight returns
	// [ a 0 ]
	// [ b 0 ]
	//------------------OLD CODE------------------//
	//basis_set[0][0] = weight[0];
	//basis_set[0][1] = weight[1];
	//basis_set[:, 1] = np.matrix([[0,-1], [1,0]]) * weight 
	//basis_set[1][0] = - weight[1];
	//basis_set[1][1] = weight[0];
	//------------------OLD CODE------------------//
	basis_set[0][0] = weight[0];
	basis_set[1][0] = weight[1];
	
	basis_set[0][1] = -weight[1];
	basis_set[1][1] = weight[0];
	
	// dewhitening_mat (2x2) * basis_set (2x2)
	vector_dw[0][0] = matrix_dw[0][0] * basis_set[0][0] + matrix_dw[0][1] * basis_set[1][0];
	vector_dw[0][1] = matrix_dw[0][0] * basis_set[0][1] + matrix_dw[0][1] * basis_set[1][1];
	vector_dw[1][0] = matrix_dw[1][0] * basis_set[0][0] + matrix_dw[1][1] * basis_set[1][0];
	vector_dw[1][1] = matrix_dw[1][0] * basis_set[0][1] + matrix_dw[1][1] * basis_set[1][1];
	
	// basis_set.T * whitening_mat
	ica_fltr[0][0] = basis_set[0][0] * matrix_w[0][0] + basis_set[1][0] * matrix_w[1][0];
	ica_fltr[0][1] = basis_set[0][0] * matrix_w[0][1] + basis_set[1][0] * matrix_w[1][1];
	ica_fltr[1][0] = basis_set[0][1] * matrix_w[0][0] + basis_set[1][1] * matrix_w[1][0];
	ica_fltr[1][1] = basis_set[0][1] * matrix_w[0][1] + basis_set[1][1] * matrix_w[1][1];
	
}

/* USER CODE END 4 */
// Apply filter by multiplying the ica filter with the Signal and write the result in the buffer.
void filter(void){
	int idx;
	uint32_t offset;
	
	for(int i = 0; i < nsamples; i++)
  {
    idx = i % buf_size;

    if(idx == 0)
    {
      // Read to buffer from flash
      offset = (i / buf_size) * buf_size * sizeof(float);
      BSP_QSPI_Read((uint8_t *)signal[0], S0_START_ADDR + offset, DATA_WIDTH);
      BSP_QSPI_Read((uint8_t *)signal[1], S1_START_ADDR + offset, DATA_WIDTH);
    }
		
		// Apply FastICA filter and add mean that was substracted 
		//signal_w[0][idx] = ica_fltr[0][0] * (signal[0][idx] + mean[0]) + ica_fltr[0][1] * (signal[1][idx] + mean[1]);
    //signal_w[1][idx] = ica_fltr[1][0] * (signal[0][idx] + mean[0]) + ica_fltr[1][1] * (signal[1][idx] + mean[1]);
		signal_w[0][idx] = ica_fltr[0][0] * signal[0][idx] + ica_fltr[0][1] * signal[1][idx];
    signal_w[1][idx] = ica_fltr[1][0] * signal[0][idx] + ica_fltr[1][1] * signal[1][idx];
		
		// Write the resulting filter to QSPI overwritting signal_w
		if(idx == buf_size -1){
			offset = ((i / (buf_size-1)) - 1) * buf_size * sizeof(float);
			BSP_QSPI_Write((uint8_t *)signal_w[0], S0W_START_ADDR + offset, DATA_WIDTH);
			BSP_QSPI_Write((uint8_t *)signal_w[1], S1W_START_ADDR + offset, DATA_WIDTH);
		}
  }
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	int i, flash_flag, idx, stage;
	stage = 0;
	uint32_t offset;
	float32_t s[2], x[2];
	arm_matrix_instance_f32 matrix_s, matrix_x;

	matrix_s.numRows = 2;
	matrix_s.numCols = 1;
	matrix_s.pData = s;

	matrix_x.numRows = 2;
  matrix_x.numCols = 1;
	matrix_x.pData = x;

	i = 0;
	flash_flag = 0;
	
	//Set Stage light to off
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET  );
	
	printf("----------------- Stage 0 : Initialization DONE ----------------------\n");
  /* Infinite loop */
  for(;;)
  {	
		
		//STAGE CONTROL
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == HAL_OK){
			stage += 1;
			if(stage == 6) stage = 1;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		}
		
		
		// STAGE 1 : On press generate the mix matrix
		// TODO : REVIEW INPUT
		if(stage == 1){ 
			printf("----------------- Stage 1 : GENERATE MIX WAVES ----------------------\n");

			for(int i = 0; i < 32000; i++){
				if(tim3_flag)
				{

					tim3_flag = 0;
					idx = i % buf_size;

					if(flash_flag && idx == 0)
					{
						// read to buffer from flash
						offset = (i / buf_size) * buf_size * sizeof(float);
						BSP_QSPI_Read((uint8_t *)signal[0], S0_START_ADDR + offset, DATA_WIDTH);
						BSP_QSPI_Read((uint8_t *)signal[1], S1_START_ADDR + offset, DATA_WIDTH);
					}
					else if(!flash_flag)
					{
						// calc values
						s[0] = arm_sin_f32((2 * PI * f[0] * i) / sampling_freq) + 1;
						s[1] = arm_sin_f32((2 * PI * f[1] * i) / sampling_freq) + 1;
						s[0] *= scale_coeff;
						s[1] *= scale_coeff;
						
						arm_mat_mult_f32(&matrix_a, &matrix_s, &matrix_x);

						signal[0][idx] = x[0]; 
						signal[1][idx] = x[1];

						if(idx == buf_size-1)
						{
							// write from buffer to flash
							offset = ((i / (buf_size-1)) - 1) * buf_size * sizeof(float);
							BSP_QSPI_Write((uint8_t *)signal[0], S0_START_ADDR + offset, DATA_WIDTH);
							BSP_QSPI_Write((uint8_t *)signal[1], S1_START_ADDR + offset, DATA_WIDTH);
						}
					}

					//printf("%f\t%f\t%d\n", signal[0][idx], signal[1][idx], i);
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, signal[0][idx]);
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, signal[1][idx]);

					if(++i == nsamples)
					{
						flash_flag = 1; // start reading vals from flash
						i = 0;
					}
				}
			}
			// Set stage light to off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET  );
		}
		
		// STAGE 2 : Generate the Weight Matrix using fast ica
		if(stage == 2){
			printf("----------------- Stage 2 : FAST ICA ----------------------\n");
			fast_ica(1000, 0.0001);
			
			// Set stage light to off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET  );
		}
		
		// STAGE 3 : Use Weight Matrix to get the original sine waves (Reuse the Memory)
		if(stage == 3){
			printf("----------------- Stage 3 : FILTER MIX MATRIX ----------------------\n");
			
			filter();
			
			// Set stage light to off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET  );
		}
		//TODO
		// STAGE 4 : Compare the two result
		if(stage == 4){
			printf("----------------- Stage 4 : COMPARISON ----------------------\n");
		
			for(int i = 0; i < 32000; i++){
				
					idx = i % buf_size;
				
					//Generate Similar initial wave
					s[0] = arm_sin_f32((2 * PI * f[0] * i) / sampling_freq) + 1;
					s[1] = arm_sin_f32((2 * PI * f[1] * i) / sampling_freq) + 1;
					s[0] *= scale_coeff;
					s[1] *= scale_coeff;
				
					// read to buffer from flash
					if(flash_flag && idx == 0){
						offset = (i / buf_size) * buf_size * sizeof(float);
						BSP_QSPI_Read((uint8_t *)signal_w[0], S0W_START_ADDR  + offset, DATA_WIDTH);
						BSP_QSPI_Read((uint8_t *)signal_w[1], S1W_START_ADDR  + offset, DATA_WIDTH);
					}
					
					// Output Difference of magnitude
					printf("%f\t%f\t%d\n", signal_w[0][idx]-s[0], signal_w[1][idx]-s[1], i);
				
			}
			// Set stage light to off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET  );
		}
		
		if(stage == 5){
			printf("----------------- Stage 5 : DONE ----------------------\n");
			
			// Set stage light to off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET  );
		}
		
		HAL_Delay(1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	tim3_flag = 1;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
