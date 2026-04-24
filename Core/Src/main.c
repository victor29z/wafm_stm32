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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_XSIZE 10000
#define MAX_YSIZE 10000
#define XPOINTS_PER_LINE 2500
#define YPOINTS_PER_LINE 1000

#define POINTS_V 5000
#define DAC_X_MSB 4096  // DAC输出的最大�?�，12-bit分辨率为4095，但为了方便计算和留有余量，使用4096
#define DAC_Y_MSB 4096  // 同上，DAC输出的最大�??

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//ALIGN_32BYTES (uint16_t adc3_buffer[ADC_BUFFER_SIZE]) __attribute__((section(".ARM.__at_0x38000000")));
ALIGN_32BYTES (uint16_t adc_buffer[ADC_BUFFER_SIZE]) __attribute__((section(".ARM.__at_0x38000000")));
volatile uint8_t dma_transfer_complete_flag = 0;
volatile uint8_t dma_transfer_half_complete_flag = 0;


// AFM Scan parameters
static uint16_t scan_x_size = 10000; // integer in nm, 10 um
static uint16_t scan_y_size = 10000; // integer in nm, 10 um
static uint16_t scan_x_offset = 0; // integer in nm
static uint16_t scan_y_offset = 0; // integer in nm
static uint16_t scan_rate = 1; // in Hz
static uint16_t scan_samples = 250;
static uint16_t scan_lines = 250;
static bool scanning = false;
static bool scan_direction_upward = false; // true: upward (from max to 0), false: downward (from 0 to max)
static uint16_t current_scan_line = 0;
static bool frame_completed = false;
static bool half_line_completed = false;
static bool line_completed = false;

static bool scan_x_tr = false; // true: x scanning in retrace direction, false: x scanning in trace direction

// scanning control varibles
static float scan_x_inc = 0.0f; 
static float scan_y_inc = 0.0f; 

uint16_t scan_y_index = 0; // 当前扫描行的索引
ALIGN_32BYTES (uint16_t x_dac_buffer_t[XPOINTS_PER_LINE]) __attribute__((section(".ARM.__at_0x38000000")));
ALIGN_32BYTES (uint16_t x_dac_buffer_r[XPOINTS_PER_LINE]) __attribute__((section(".ARM.__at_0x38000000")));
ALIGN_32BYTES (uint16_t y_dac_buffer[YPOINTS_PER_LINE]) __attribute__((section(".ARM.__at_0x38000000")));
//ALIGN_32BYTES (uint16_t y_dac_buffer_nl[YPOINTS_PER_LINE]) __attribute__((section(".ARM.__at_0x38000000")));  // nl: next line


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

static void Scan_Init(void);
static void Launch_Scan(void);
static void Stop_Scan(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *stream)
{
    /* ??????? USART3 ???? */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SPI5_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  /* Start DAC channels */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  
  /* Set DAC output values: 1/3 and 2/3 of full scale (4095) */
  /* DAC resolution is 12-bit, so full scale = 4095 */
  // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1365);  /* 4095 * 1/3 = 1365 */
  // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2730);  /* 4095 * 2/3 = 2730 */
  
  /* Perform ADC calibration before starting */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  

  
  /* Start TIM3 to trigger ADC1 conversions */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

    /* Start TIM2 to trigger DAC1 CH1 conversions */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start TIM4 to trigger DAC1 CH2 conversions */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    Error_Handler();
  }



  
  /* Initialize scanning parameters */
  Scan_Init();



  /* Debug: Send startup message */
  printf("STM32H730 AFM Scan Control Started\r\n");



  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // if(dma_transfer_complete_flag==1)
    // {
    //   // 处理 buffer1 数据
    //   // ...

    //   uint32_t sum = 0;
    //   float voltage;
      
    //   // 计算1000个点总和
    //   for(int i=0; i<1000; i++) sum += adc_buffer[i + 1000];

      
    //   // 计算平均�????????? �????????? 转换成电压（12位ADC�?????????3.3V参�?�）
    //   uint16_t adc_avg = sum / 1000;
    //   //voltage = (adc_avg * 3.3f) / 4095.0f;
    //   voltage = ((adc_buffer[1000]) * 3.3f) / 4095.0f;
      
    //   // 通过 USART3 打印平均电压
    //   printf("retrace volt: %.3f V\r\n", voltage);

    //   printf("full transfer complete, processing buffer1...\n");
    //   dma_transfer_complete_flag = 0; // 重置标志
    //   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET); // 点亮 LED
    // }

    // if(dma_transfer_half_complete_flag==1)
    // {
    //   // 处理 buffer2 数据
    //   // ...
    //   uint32_t sum = 0;
    //   float voltage;
      
    //   // 计算1000个点总和
    //   for(int i=0; i<1000; i++) sum += adc_buffer[i];

      
    //   // 计算平均�????????? �????????? 转换成电压（12位ADC�?????????3.3V参�?�）
    //   uint16_t adc_avg = sum / 1000;
    //   //voltage = (adc_avg * 3.3f) / 4095.0f;
    //   voltage = ((adc_buffer[0]) * 3.3f) / 4095.0f;
      
    //   // 通过 USART3 打印平均电压
    //   printf("trace volt: %.3f V\r\n", voltage);
    //   printf("half transfer complete, processing buffer2...\n");
    //   dma_transfer_half_complete_flag = 0; // 重置标志
    //   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET); // 熄灭 LED
    // }


    if(line_completed){
      // 处理半行扫描完成的情况，例如更新 DAC 输出以准备下�??行扫�??
      if(scan_direction_upward){
        // 如果当前扫描方向是向上（从最大到0），则更�?? DAC 输出为下�??行的�??
        if(current_scan_line > 0)  // 如果不是第一行，预先计算上一行的Y DAC�??
        {
          uint16_t i;
          for(i=0;i<YPOINTS_PER_LINE;i++)
          {
            y_dac_buffer[i] = (uint16_t)(scan_y_offset * DAC_Y_MSB / MAX_YSIZE + (YPOINTS_PER_LINE * (current_scan_line - 1) + i) * scan_y_inc);
          }
        }
        else
        {
          // 如果是第�??行，预先计算第二行的Y DAC值（�??复循环扫描）
          uint16_t i;
          for(i=0;i<YPOINTS_PER_LINE;i++)
          {
            y_dac_buffer[i] = (uint16_t)(scan_y_offset * DAC_Y_MSB / MAX_YSIZE + (YPOINTS_PER_LINE * (current_scan_line + 1) + i) * scan_y_inc);
          }
        }
        
      }
      else{
        // 如果当前扫描方向是向下（�??0到最大），则更新 DAC 输出为下�??行的�??
        if(current_scan_line < (scan_lines - 1))  // 如果不是�??后一行，预先计算下一行的Y DAC�??
        {
          uint16_t i;
          for(i=0;i<YPOINTS_PER_LINE;i++)
          {
            y_dac_buffer[i] = (uint16_t)(scan_y_offset * DAC_Y_MSB / MAX_YSIZE + (YPOINTS_PER_LINE * (current_scan_line + 1) + i) * scan_y_inc);
          }
        }
        else
        {
            // 如果是最后一行，预先计算倒数第二行的Y DAC值（�??复循环扫描）
            uint16_t i;
            for(i=0;i<YPOINTS_PER_LINE;i++)
            {
              y_dac_buffer[i] = (uint16_t)(scan_y_offset * DAC_Y_MSB / MAX_YSIZE + (YPOINTS_PER_LINE * (current_scan_line + 0) + i) * scan_y_inc);
            }
        }
        
        
      }
      line_completed = false;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET); // 点亮 LED
      printf("line %d completed\n", current_scan_line);

      // 更新扫描行索�??
      if(scan_direction_upward){
        // 如果当前扫描方向是向上（从最大到0），则索引�?�减  
        if(current_scan_line > 0) current_scan_line--;
        else{
          // 已经扫描到最上面�??行，切换扫描方向
          scan_direction_upward = false;
        }

      }
      else{
        // 如果当前扫描方向是向下（�??0到最大），则索引递增
        if(current_scan_line < (scan_lines - 1)) current_scan_line++;
        else{
          // 已经扫描到最下面�??行，切换扫描方向
          scan_direction_upward = true;
        }
      }
    }

    if(half_line_completed)
    {
      // 处理半行扫描完成的情况，例如更新 DAC 输出以准备半行扫�??
      // ...
      half_line_completed = false; // 重置标志
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET); // 熄灭 LED
      //printf("half line %d completed\n", current_scan_line);
    }


    // if(line_completed){
    //   // 处理整行扫描完成
    //   printf("line %d completed\n", current_scan_line);
    //   line_completed = false;
    // }
    /* ADC conversions are handled by TIM3 trigger and DMA interrupt */
    /* No need for polling in main loop */
    
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 110;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 274;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 274;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 274;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI5_CS_Pin */
  GPIO_InitStruct.Pin = SPI5_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI5_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  // 半传输完成回调：切换到另�????????个缓冲区
  dma_transfer_half_complete_flag = 1; // 设置半传输完成标�????????

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

  dma_transfer_complete_flag = 1; // 设置传输完成标志

  
  

  


}


/* line scanning dac buffer filling*/
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  // DAC1通道1转换完成回调
  // 可以在这里更新DAC输出值以实现连续扫描
  // 例如，更新x_dac_buffer中的下一个�??
  // 注意：确保更新�?�度与扫描�?�率匹配，避免过快或过慢

  if(scan_x_tr == false){
    half_line_completed = true;

    // 如果当前是扫描方向（trace），则下�??次更新为回扫方向（retrace）的DAC�??
    scan_x_tr = true;
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)x_dac_buffer_r, XPOINTS_PER_LINE, DAC_ALIGN_12B_R);
  }
  else{
    line_completed = true;

    // retrace finished, prepare for next trace
    scan_x_tr = false;
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)x_dac_buffer_t , XPOINTS_PER_LINE, DAC_ALIGN_12B_R);
    // start trace sampling for the next line
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2000);
    

  }
  
  
  
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)y_dac_buffer , YPOINTS_PER_LINE, DAC_ALIGN_12B_R);



}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t spi_rx_buffer[128];
  uint8_t spi_tx_buffer[128];
  if(GPIO_Pin == GPIO_PIN_1){
    // 处理GPIO_PIN_1的中断事�?
    // ...
    //printf("GPIO_PIN_1 interrupt triggered\n");

    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_SET){
      // PD1上升沿触�?
      scanning = true;
      printf("Scanning started\n");
      Scan_Init(); // 初始化扫描参数
      Launch_Scan(); // 启动扫描  
      

      
      

    }
    else{
      // PD1下降沿触�?
      scanning = false;
      printf("Scanning stopped\n");
      Stop_Scan(); // 停止扫描
      printf("spi trans finished\n");
      
    }
  }
  else if(GPIO_Pin == GPIO_PIN_0){
    // GPIO_PIN_0 : Scan params update trigger
    
    //printf("spi trans pending\n");
    HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi5, (uint8_t*)spi_tx_buffer, (uint8_t*)spi_rx_buffer, 14, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);

    printf("spi params updated!\n");

    //printf("GPIO_PIN_0 interrupt triggered\n");
  }
}

void Scan_Init(void){
  uint16_t i;

  current_scan_line = 0;

  line_completed = false;
  half_line_completed = false;
  scan_x_tr = false;


  scan_x_inc = (float)scan_x_size * DAC_X_MSB / MAX_XSIZE / XPOINTS_PER_LINE; // X增量
  scan_y_inc = (float)scan_y_size * DAC_Y_MSB / MAX_YSIZE / (YPOINTS_PER_LINE * scan_lines); // Y增量

  for(i=0; i<XPOINTS_PER_LINE; i++){
    x_dac_buffer_t[i] = (uint16_t)(scan_x_offset * DAC_X_MSB / MAX_XSIZE + i * scan_x_inc);
    x_dac_buffer_r[i] = (uint16_t)(scan_x_offset * DAC_X_MSB / MAX_XSIZE + (XPOINTS_PER_LINE - 1 - i) * scan_x_inc);
  }


  // fill buffer of first line of Y scanning, the rest will be updated in the DAC callback
  for(i=0; i<YPOINTS_PER_LINE; i++){
    y_dac_buffer[i] = (uint16_t)(scan_y_offset * DAC_Y_MSB / MAX_YSIZE + i * scan_y_inc);
    // if(current_scan_line < scan_lines - 1)  // 如果不是�??后一行，预先计算下一行的Y DAC�??
    //   y_dac_buffer_nl[i] = (uint16_t)(scan_y_offset * DAC_Y_MSB / MAX_YSIZE + (YPOINTS_PER_LINE * (current_scan_line + 1) + i) * scan_y_inc);
  }

}


void Launch_Scan(void){
  // 启动扫描的函数，可以在这里执行任何必要的初始化或状态设置
  // 例如，重置扫描行索引，设置扫描标志等
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)x_dac_buffer_t , XPOINTS_PER_LINE, DAC_ALIGN_12B_R);
  /* Start line sampling right after DAC is started */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)y_dac_buffer ,  YPOINTS_PER_LINE, DAC_ALIGN_12B_R);
  // 可以在这里添加其他需要在扫描开始时执行的操作
}


void Stop_Scan(void){
  // 停止扫描的函数，可以在这里执行任何必要的清理或状态重置
  // 例如，停止DMA传输，重置扫描标志等
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
  // 可以在这里添加其他需要在扫描停止时执行的操作
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
