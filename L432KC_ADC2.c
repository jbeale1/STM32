/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Stats from single ADC readings on Nucleo STM32 L432KC
  * 43 readings per sec using 20k samples (2 ADC reading/sample)  3-Dec-2023 JPB
  * 491 rdg/sec with 2k samples, Peak = (Max-Min) output
  * 840 rdg/sec with 1k samples
  * full buffer rate: 2.132 kHz @ 1000 samples=> 2.13 MHz rate
  * 4.264 kHz @ 500 samples (buffer callback) 0.6984 kHz (main proc. rate) ratio 6:1
  * 4x oversample: @ 500 samples: 1.0661 kHz (buf callback)  0.8557 kHz proc
  * 4x over: @ 1000 samples: 0.5330 kHz (buf callbk)  0.4374 kHz proc
  * 4x over: @ 4k samples:  133.26 Hz (buf callbk)  111.38 Hz proc
  * 8x over: @ 4k samples:  66.63 Hz (buff callbk)  66.63 Hz proc <= SYNC!
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>  // sqrt()
#include <stdio.h> // printf()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPU_RATE 32000000   // CPU counter ticks per second
#define true 1
#define false 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define ADC_BUFFER_SIZE 4000 // was 20k
uint16_t adc_buffer[ADC_BUFFER_SIZE];  // DMA writes ADC data into this buffer
uint16_t tBuf[ADC_BUFFER_SIZE/2];       // working buffer for operation & printout

uint32_t counter = 0;
uint32_t tick=0;      // geiger counter "count"
uint32_t lastTick=0;

int32_t sMax = 0;
int32_t sMin = 65535;
float avgMin = 360.0;  // 64 for no oversamp, 360 @ 4x
float avgMinFilt = 0.01;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int readingCount = 0;            // how many sets of ADC readings done
int lastValue = 0;               // previous ADC sample, for rising-edge detect
int last2Value = 0;              // 2nd previous ADC sample, for rising-edge detect
volatile int buf1Ready = false;  // Ping buffer is ready
volatile int buf2Ready = false;  // Pong buffer is ready

int pThreshold = 100;            // ADC amplitude threshold for finding rising edge

// Read a set of values from STM32 Nucleo L432KC board 12-bit ADC and get statistics
// Typical St.Dev. = 3.1 counts. Single-ended input from 1.5V AA (4095 counts = 3.3V)
// 14800 readings per second (SAMPLES=4k, no ADC oversampling)
// count   sps1      sps2      avg       min       max       st.dev
// 16.000  13675.214 13673.658 28240.963 28186.000 28297.000 15.218  (16x oversample)
// 10852.0 13675.2   13673.5   28252.0   28194.0   28317.0   14.5
// 11824.0 13675.2 13674.4 6620.8 6393.0 6827.0 55.5  KC761 raw signal


// Process a Ping or Pong buffer looking for peaks
void procBuf(int bIndex) {
    int idxStart, idxEnd;

    if (bIndex==1) {
    	idxStart=0;
    	idxEnd = ADC_BUFFER_SIZE/2;
    } else {
    	idxStart=ADC_BUFFER_SIZE/2;
    	idxEnd = ADC_BUFFER_SIZE;
    }

    // ==================================================
    /*
    uint32_t j=0;
    for (int i = idxStart; i < idxEnd; i++) {
      tBuf[j] = adc_buffer[i];        // copy into working buffer tBuf[]
      j++;
    }
    */
    // ==================================================

    // int pThreshold = (132-avgMin);         // ADC thresh for rising edge (no oversamp)
    int pThreshold = (510-avgMin);            // ADC thresh for rising edge (8x oversamp, <1 bitshft)
	sMax = 0;
	sMin = 65535;

    for (int i = idxStart; i < idxEnd; i++) {
    //for (int i = 0; i < ADC_BUFFER_SIZE/2; i++) {
      //uint32_t x = tBuf[i];
      uint32_t x = adc_buffer[i];
      uint32_t xTest = x - avgMin;
      if ((lastValue < pThreshold) && (last2Value < pThreshold) && (xTest > pThreshold)) {
    	  tick++;
      }
      if (x > sMax) sMax = x;
      if (x < sMin) sMin = x;
      last2Value = lastValue;
      lastValue = xTest;			// remember current value for next time
    }
    avgMin = (avgMin * (1.0-avgMinFilt)) + sMin*avgMinFilt;
	counter++;

	/*
	if (sMax > 1000) {  // dump this buffer, for large signals
		printf("sMax = %ld ==============\n",sMax);
	    for (int i = 0; i < ADC_BUFFER_SIZE/2; i++) {
	      uint32_t x = tBuf[i];
	      printf("%ld\n",x);
	    }
		printf("\n");
	}
	*/
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// enable core cycle counter per https://stackoverflow.com/questions/42747128/
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //testADC();


	// Wait for DMA buffer to be filled
	while (!buf1Ready);
	buf1Ready = false;
	procBuf(1); // process first half ("ping") of buffer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	while (!buf2Ready);
	buf2Ready = false;
	procBuf(2); // process second half ("pong") of buffer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

    // 800 * 1860 in 1 second => 1.5 MHz
	if (counter > 132) {               // report CPS each second
		printf("%ld, %ld, %ld, %5.1f\n", tick, sMin, sMax, avgMin); // output CPS value, min/max
		counter = 0;
		lastTick = tick;
		tick=0;
	}

    //HAL_Delay(50);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  buf1Ready = true;  // Ping buffer is ready
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  buf2Ready = true;  // Pong buffer is ready
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

}

// ================================================================
void testADC() {

	  uint32_t tStart, deltaT;
	  uint32_t start_ms, delta_ms;
	  int SAMPLES = 40000;
      long datSum = 0;  // reset our accumulated sum of input values to zero
      int sMax = 0;
      int sMin = 65535;
      long n;            // count of how many readings so far
      double x,mean,delta,m2,variance,stdev;  // to calculate standard deviation

      tStart = DWT->CYCCNT;
      start_ms = HAL_GetTick();

      // oldT = millis();   // record start time in milliseconds

      n = 0;     // have not made any ADC readings yet
      mean = 0; // start off with running mean at zero
      m2 = 0;

      for (int i=0;i<SAMPLES;i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t value = HAL_ADC_GetValue(&hadc1);
        x = value;

        // x = analogRead(analogInPin);
        datSum += x;
        if (x > sMax) sMax = x;
        if (x < sMin) sMin = x;
        n++;
        delta = x - mean;
        mean += delta/n;
        m2 += (delta * (x - mean));
      }
      variance = m2/(n-1);  // (n-1):Sample Variance  (n): Population Variance
      stdev = sqrt(variance);  // Calculate standard deviation

      deltaT =  DWT->CYCCNT - tStart; // cycles
      delta_ms = HAL_GetTick() - start_ms; // milliseconds

      //long durT = millis() - oldT;
      float datAvg = (1.0*datSum)/n;
      readingCount++;

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

      printf("%d, ",readingCount);
      printf("%5.3f, ", (1.0E3*n/delta_ms)); // readings per sec
      printf("%5.3f, ", (1.0*CPU_RATE*n/deltaT)); // readings per sec  32MHz CPU clock
      printf("%5.3f, ", datAvg);
      // printf(" Offset: ");  printf(datAvg - EXPECTED,2);
      printf("%d, ",sMin);
      printf("%d, ",sMax);
      printf("%5.3f",stdev);
      printf("\n");
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

} //  ======================
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
