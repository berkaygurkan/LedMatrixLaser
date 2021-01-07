/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void delay_us (uint16_t delay);
void CheckLed (void);
void CheckColumn (void);
void CheckRow (void);
void AverageColumn(void);
void ShowScreen(void);
void DisplayScreen(uint16_t display[]);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */



  
 
  uint16_t go[8] = {0x00,0xF0,0x90,0x87,0xB5,0x95,0xF7,0x00};
							 
          
             
            
             
       
									
										
										
										
										
										
										





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Pin Numaralari Tanimlamalari
uint32_t rows[8] ={GPIO_PIN_15,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7,GPIO_PIN_8,GPIO_PIN_9};       // PA12 PA15 | PB2-PB8
uint32_t cols[8] ={GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7};				// PA0-PA7


// ADC Dizileri
uint32_t adc_col[8]={0};																																														// ADC Ham Veri Dizisi
uint32_t adc_col_tresh[8]={0};																																											// Baslangic Voltaj Dizisi
int32_t diff[8]={0};																																																// Fark Dizisi


// Lazer Pointer Degiskenleri
uint8_t pointed_column=8;																																														// Lazerle isaretlenen Sütün Degeri - [8 - lazer yok]
uint8_t pointed_row=8;																																															// Lazerle isaretlenen Satir Degeri - [8 - lazer yok]			

uint16_t screen[8]={0};																																															// Ekran Deger Dizisi

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
//  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


	CheckLed();																																																						// Baslangic Ekrani ve Ledler
	AverageColumn();																																																			// Ortalama Voltaj Degerleri Okunuyor





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			

		CheckColumn();																																																			// Lazer Pointer Ile Isaretlenen Satir Degeri Hesaplama Fonksiyonu
		CheckRow();																																																					// Lazer Pointer Ile Isaretlenen Sutun Degeri Hesaplama Fonksiyonu
				
		HAL_GPIO_DeInit(GPIOA,GPIO_PIN_15);
		HAL_GPIO_DeInit(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
														|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
		

		if(pointed_row <8 && pointed_column <8)
		{
			screen[pointed_row]= screen[pointed_row] | ( 1u<<pointed_column);
		}
		
		
		
		//ShowScreen();																																																				// Ekrana Isaretlenen Deger Yazdiriliyor.
		DisplayScreen(go);
			
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
	|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
	|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void micros(uint16_t delay)
{
 __HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<delay);

}



void delay_us (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<<delay);
}

void CheckLed (void)
{

	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);


	HAL_Delay(500);
	
	
	// Ledler Kapali
	GPIOA->BSRR=(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)<<16u;				// Reset
	GPIOA->BSRR=(GPIO_PIN_15);																																												// Set
	GPIOB->BSRR=(GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);												// Set
	
	
	
	
	
	
	
	
	
	// Problem Var mi Diye Sebebiyle Tek Tek Açiliyor.
	for(uint8_t j=0; j<8;j++)
{
	if(j<1)
	{
			GPIOA->BSRR = rows[j] << 16u;											// Satir LOW 
	}
	else
	{
			GPIOB->BSRR = rows[j]<<16u;												// Satir LOW
	}

	for(uint8_t i=0; i<8;i++)
	{
			// Sütun Aç		
		GPIOA->BSRR = cols[i];
		HAL_Delay(10);
		// Sütun Kapa		
		GPIOA->BSRR = cols[i]<<16u;
	}
	
	if(j<1)
	{
			GPIOA->BSRR = rows[j];											
	}
	else
	{
			GPIOB->BSRR = rows[j];
	}


}
}


void CheckColumn (void)
{
	
	
	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_15);
	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	HAL_GPIO_DeInit(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
	

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
	|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
	|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	
	 GPIOB->BSRR=(GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9)<<16u;												// Reset
	 GPIOA->BSRR=(GPIO_PIN_15)<<16u;																																													// Reset		
// GPIO Pinleri Kapatiliyor
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
		MX_ADC1_Init();
		HAL_ADC_Start_DMA(&hadc1,adc_col,8);
		HAL_Delay(5);
		HAL_ADC_Stop(&hadc1);
		HAL_ADC_DeInit(&hadc1);
		
		
		for(uint8_t i=0; i<8; i++)
		{
		 diff[i]=adc_col[i]-adc_col_tresh[i];
		}
		
		pointed_column = 8;
		
		for (uint8_t j=0; j<8; j++)
		{
			if (diff[j] >100)
			{
			 pointed_column = j;
			 break;
			}
		}

}










void AverageColumn(void)
{
	
	MX_ADC1_Init();
	
	for (uint16_t k=0; k<500; k++)
	{
		
	 GPIOB->BSRR=(GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9)<<16u;												// Reset
	 GPIOA->BSRR=(GPIO_PIN_15)<<16u;																																													// Reset		
// GPIO Pinleri Kapatiliyor
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
		MX_ADC1_Init();
		HAL_ADC_Start_DMA(&hadc1,adc_col,8);
		HAL_Delay(5);
			for (uint8_t j=0; j<8; j++)
		{
			adc_col_tresh[j] += adc_col[j];
		}
		HAL_ADC_Stop(&hadc1);
		HAL_ADC_DeInit(&hadc1);
		MX_GPIO_Init();
	}


		for (uint8_t j=0; j<8; j++)
		{
			adc_col_tresh[j] = adc_col_tresh[j]/500;
		}

	

		


}

void CheckRow(void)
{
	
	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_15);
	HAL_GPIO_DeInit(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
	
	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	
	// Pinler Output Olarak Konfigüre Ediliyor.
	GPIO_InitTypeDef GPIO_InitStruct = {0};
 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

 
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	

	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	
		
		
		
		
	// Tersine Voltaj Uygulaniyor
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_SET);
	
	delay_us(100);
	
	
	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_15);
	HAL_GPIO_DeInit(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
	
	GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	

	
	uint16_t val[8] = {1000,1000,1000,1000,1000,1000,1000,1000};
	
	
	for( uint16_t cnt=0; cnt<1000; cnt++ )
	{
    for( uint8_t r=0; r<8; r++ )
		{
			if (r<1)
			{
				if( HAL_GPIO_ReadPin(GPIOA,rows[r]) == GPIO_PIN_RESET && val[r] == 1000 )
				{
					val[r] = cnt;
				}
			}
			else
			{
				if( HAL_GPIO_ReadPin(GPIOB,rows[r]) == GPIO_PIN_RESET && val[r] == 1000 )
				{
					val[r] = cnt;
				}	
			}
      
    }
	}
		pointed_row=8;
	for (uint8_t j=0; j<8; j++)
		{
			if (val[j]<100)
			{
			 pointed_row = j;
			 break;
			}
		}

}








void ShowScreen(void)
{
	
				

	
	GPIO_InitTypeDef GPIO_InitStruct = {0};						
	GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	

		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
										|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4
										|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_SET);

	
	for( uint8_t i=0; i<5; i++)	
	{
    for( uint8_t row=0; row<8; row++ )
			{   
				if(row<1)
				{
					GPIO_InitTypeDef GPIO_InitStruct = {0};	
					GPIO_InitStruct.Pin = rows[row];
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					GPIOA->BSRR = rows[row]<<16u;
				  
	
				}
				else if(row>=1 && row<8)
				{
					GPIO_InitTypeDef GPIO_InitStruct = {0};	
					GPIO_InitStruct.Pin = rows[row];
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					GPIOB->BSRR = rows[row]<<16u;
				}
				
				
				
				GPIOA->ODR = screen[row];
				
							
				if(row>0)
				{
					GPIOA->BSRR = GPIO_PIN_15;
				}
				
			for(uint16_t k=0; k<100;k++)
				{
					delay_us(800);
				}
				
			
				
				if(row<1)
				{
						GPIOA->BSRR = rows[row];
				}
				else
				{
						GPIOB->BSRR = rows[row];
				}
    }
  }
	
	
	

}


void DisplayScreen(uint16_t display[])
{
	
				

	
	GPIO_InitTypeDef GPIO_InitStruct = {0};						
	GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	

		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
										|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4
										|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_SET);

	
	for( uint8_t i=0; i<5; i++)	
	{
    for( uint8_t row=0; row<8; row++ )
			{   
				if(row<1)
				{
					GPIO_InitTypeDef GPIO_InitStruct = {0};	
					GPIO_InitStruct.Pin = rows[row];
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					GPIOA->BSRR = rows[row]<<16u;
				  
	
				}
				else if(row>=1 && row<8)
				{
					GPIO_InitTypeDef GPIO_InitStruct = {0};	
					GPIO_InitStruct.Pin = rows[row];
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					GPIOB->BSRR = rows[row]<<16u;
				}
				
				
				
				GPIOA->ODR = display[row];
				
							
				if(row>0)
				{
					GPIOA->BSRR = GPIO_PIN_15;
				}
				
			for(uint16_t k=0; k<100;k++)
				{
					delay_us(800);
				}
				
			
				
				if(row<1)
				{
						GPIOA->BSRR = rows[row];
				}
				else
				{
						GPIOB->BSRR = rows[row];
				}
    }
  }
	
	
	

}

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
