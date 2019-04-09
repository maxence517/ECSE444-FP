 
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
int tim3_flag = 0;
int nbSaving=0; //number of savings 
float f1 = 261.63;
float f2 = 392;
float sampling_frequency = 16000;


//Buffer for writing/reading to QSPI

float32_t r_buff1[400];
float32_t r_buff2[400];
float32_t r_buff[800];

float32_t rbuff1;
float32_t rbuff2;
float32_t a[4] = {2, 4, 5, 3};	// steps for initializing the matrix
arm_matrix_instance_f32 matrix_a = {2, 2, a};


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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DFSDM1_Init();
  MX_DAC1_Init();
  BSP_QSPI_Init();
		
		
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	//if(BSP_QSPI_Erase_Chip() != QSPI_OK){ printf("Error - Chip Clean \n");}else{
		//printf("OK - Chip Clean \n");
	//}

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

	int i = 0;
	float s1, s2;
	
	//Creating Matrix
	float32_t x[2];
  arm_matrix_instance_f32 matrix_x = {2, 1, x};	
	
	// Saving flag
	int saving = 1; 
	
	
  for(;;)
  {
 
		if(tim3_flag == 1) {
			tim3_flag = 0;
			
			/**** Writing Block ****/
			if(i<32000 && saving == 1) {
				s1 = arm_sin_f32((2 * PI * f1 * i) / sampling_frequency);
				s2 = arm_sin_f32((2 * PI * f2 * i) / sampling_frequency);
				s1 = (s1+1)*50;
				s2 = (s2+1)*50;
			
				float s[2] = {s1, s2};
				arm_matrix_instance_f32 matrix_s = {2, 1, s};
				arm_mat_mult_f32(&matrix_a, &matrix_s, &matrix_x);
				
					// ---------------
					//	x1[i] = x[0];
					//	x2[i] = x[1];
					// ---------------
				
			//	x1 = x[0];
			//	x2 = x[1];
					if(BSP_QSPI_Write((uint8_t*)&x[0], i*8, 4) != QSPI_OK){
		
						printf("Write Error \n");
					}
					if(BSP_QSPI_Write((uint8_t*)&x[1], i*8+4, 4) != QSPI_OK){
		
						printf("Write Error \n");
					}
				
			}
			
			/**** Reading Block ****/
			if( saving == 1){
				
		
				if(BSP_QSPI_Read((uint8_t*)&rbuff1, i*8, 4) != QSPI_OK){
	
						printf("Read Error \n");
				}
				if(BSP_QSPI_Read((uint8_t*)&rbuff2, i*8+4, 4) != QSPI_OK){
	
						printf("Read Error \n");
				}
				
				uint8_t flags = 0;
				
				// Test for Matching Buffer
				if(x[0]==rbuff1&&x[1]==rbuff2){
					printf("Perfect match\n");
					
				}else{
					printf("Original=%f, Our=%f\n",x[0],rbuff1);
					printf("Original=%f, Our=%f\n",x[1],rbuff2);
				}
				
			
			}
			//send data to DAC
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, rbuff1);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, rbuff2);
			
			//Stop Saving data after the 32000 samples
			if(i==31999) {
				saving = 0;
				i=-1;
				
			}
			
			i++;

		}
  }
	
  /* USER CODE END 5 */ 
}


//

void fastICA(){
	int i=0;
	float mean1,mean2;
	int buffLength; //number of samples we process each time 
	int maxIterations=1000;
	
	
	buffLength=400;
	for (int j=0;j<buffLength;j++){
		
				if(BSP_QSPI_Read((uint8_t*)&rbuff1, i*8, 4) != QSPI_OK){
	
						printf("Read Error \n");
				}
				
				r_buff[j]=rbuff1;   //r_buff1&r_buff2 stores x[0],x[1] respectively
				
				mean1+=rbuff1/buffLength; //calculate the mean of x[0]
				
				if(BSP_QSPI_Read((uint8_t*)&rbuff2, i*8+4, 4) != QSPI_OK){
	
						printf("Read Error \n");
				}
				r_buff[j+400]=rbuff2;
				 
				mean2+=rbuff2/buffLength; //calculate the mean of x[1]
		i++;
	}
	//centering the matrix
		for (int j=0;j<buffLength;j++){
				r_buff[j]=r_buff1[j]-mean1;
				r_buff[j+400]=r_buff2[j]-mean2;
		
	}
		
	//find covariance matrix cov_mat
	arm_matrix_instance_f32 centeredMat={2,buffLength,r_buff};
	float covMat0[4];
	float centeredMatT[800];
	float covMat[4];
	float evMat1[4];
	float evMat2[4];
	float eigvec[4];
	arm_matrix_instance_f32 covarianceMat0={2,2,covMat0};
	arm_matrix_instance_f32 centeredMatTrans={buffLength,2,centeredMatT};
	arm_matrix_instance_f32 covarianceMat={2,2,covMat};
	arm_mat_trans_f32	(&centeredMat,&centeredMatTrans);
	
	arm_mat_mult_f32(&centeredMat, &centeredMatTrans, &covarianceMat0);
	//scale this covariance function by  (buffLength-1), cov_mat=center_mat*center_mat.T/(num_sample-1)
	arm_mat_scale_f32(&covarianceMat0,1/(buffLength-1),&covarianceMat);
	
	//find eigenvalues and eigenvector of the covariance matrix
	float trace=covMat[0]+covMat[3];  //covMat[0]=a, covMat[1]=b,covMat[2]=c,covMat[3]=d
	float det=covMat[0]*covMat[3]-covMat[1]*covMat[2]; //det=a*d-b*c
	
	float eigval1=(trace+sqrt(trace*trace-4*det))/2;  //first eigen value
	float eigval2=(trace-sqrt(trace*trace-4*det))/2;  //second eigen value
	
	//ev1=mat-eigval1*i  (i is the identity matrix)
	//ev2=mat-eigval2*i
	evMat1[0]=covMat[0]-eigval1;
	evMat1[1]=covMat[1];
	evMat1[2]=covMat[2];
	evMat1[3]=covMat[3]-eigval1;
	
	evMat2[0]=covMat[0]-eigval2;
	evMat2[1]=covMat[1];
	evMat2[2]=covMat[2];
	evMat2[3]=covMat[3]-eigval2;
	
	eigvec[0]=evMat1[0];
	eigvec[1]=evMat2[0];
	eigvec[2]=evMat1[2];
	eigvec[3]=evMat2[2];
	
	//normalize the eigenvec
	float lengthVec1=sqrt(eigvec[0]*eigvec[0]+eigvec[2]*eigvec[2]);
	float lengthVec2=sqrt(eigvec[1]*eigvec[1]+eigvec[3]*eigvec[3]);
	eigvec[0]=eigvec[0]/lengthVec1;
	eigvec[2]=eigvec[2]/lengthVec1;
	
	eigvec[1]=eigvec[1]/lengthVec2;
	eigvec[3]=eigvec[3]/lengthVec2;
	
	float eigvecTrans[4]={eigvec[0],eigvec[2],eigvec[1],eigvec[3]}; //get transpose of eigvec
	float eigValue[4]={eigval1,0,eigval2,0};
	//whiten input matrix
	float eigValueSqrt[4]={sqrt(eigval1),0,sqrt(eigval2),0};
	float eigValueSqrtInver[4];
	float whitening[4];
	float whiten[4];
	arm_matrix_instance_f32 eValueSqrtMat={2,2,eigValueSqrt};
	arm_matrix_instance_f32 eigvecTransMat={2,2,eigvecTrans};
	arm_matrix_instance_f32 eigValueSqrtInverMat={2,2,eigValueSqrtInver};
	arm_matrix_instance_f32 whiteningMat={2,2,whitening};
	arm_matrix_instance_f32 whitenMat={2,2,whiten};
	
	arm_mat_inverse_f32(&eValueSqrtMat,&eigValueSqrtInverMat);
	arm_mat_mult_f32(&eigValueSqrtInverMat,&eigvecTransMat,&whiteningMat); 
	arm_mat_mult_f32(&whiteningMat,&centeredMat,&whitenMat);
	
	// initialize a random vector
	
	int iter=0;
	while (iter<maxIterations){
		
		//test for convergence
		
		//update weight 
		
		
		//normalize weight
		
		
		
		
	}
	
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
	tim3_flag = 1;	// need to set flag
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
