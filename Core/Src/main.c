/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * Features:
  * - ADC reads internal temperature sensor
  * - LED turns ON if temp > 50°C
  * - LED blinks if temp > 70°C
  * - UART logs temperature every second
  * - FreeRTOS tasks for ADC, LED, UART
  *
  ******************************************************************************
  */
/***********************************************
 * INCLUDES
 ***********************************************/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/***********************************************
 * DEFINES
 ***********************************************/
/* Calibration addresses for STM32F4 internal temp sensor */
/* Values from Data sheet stm32f446mc -> Table 81. Temperature sensor calibration value*/
#define TEMP30_CAL_ADDR  ((uint16_t*) ((uint32_t) 0x1FFF7A2C)) 
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFF7A2E))
#define TEMP30           30.0f
#define TEMP110          110.0f

/* LED GPIO config */
#define LED_GPIO_PORT GPIOA
#define LED_GPIO_PIN  GPIO_PIN_5

/* Temperature thresholds */
#define TEMP_HIGH_THRESHOLD   29.0f  /* Temperature above which LED blinks */
#define TEMP_LOW_THRESHOLD    26.0f  /* Temperature above which LED stays ON */

/***********************************************
 * GLOBAL VARIABLES
 ***********************************************/
/* Handles for ADC and USART */
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

/* FreeRTOS objects */
osThreadId_t gx_adc_task_handle;
osThreadId_t gx_led_task_handle;
osThreadId_t gx_uart_task_handle;
osMutexId_t gx_temp_mutex_handle;

/* Definitions of Attributes for Task */
const osThreadAttr_t gx_adc_task_attributes = {
  .name = "ADC_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
const osThreadAttr_t gx_led_task_attributes = {
  .name = "LED_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
const osThreadAttr_t gx_uart_task_attributes = {
  .name = "UART_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
const osMutexAttr_t gx_mutex_attr = {
  .name = "tempMutex"
};

/* Shared temperature variable */
volatile float gf_temperature = 0.0f;

/***********************************************
 * FUNCTION PROTOTYPES
 ***********************************************/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

/* RTOS Tasks */
void adc_task(void *argument);
void led_task(void *argument);
void uart_task(void *argument);

/* Initialize the mutexes */
void init_mutexes(void);
/* Initialize the tasks */
void init_tasks(void);

/* Convert ADC raw value to °C */
float convert_adc_to_temperature(uint16_t au16_raw_adc);

/***********************************************
 * MAIN
 ***********************************************/
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* Create mutex before starting scheduler */
  init_mutexes();

  /* Init scheduler */
  osKernelInitialize();

  /* Creation of the threads */
  init_tasks();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1) {}
}

/***********************************************
 * HELPER FUNCTIONS
 ***********************************************/

/**
 * @brief  Converts raw ADC value from internal temperature sensor to Celsius temperature.
 *
 * This function reads calibration values stored in system memory at known addresses
 * for two reference temperatures (typically 30°C and 110°C). It uses these calibration
 * points to calculate the temperature with taken adc value (formula taken from 
 * ST AN3964 document, section 2.2.1) 
 *
 * The formula applied is:
 *   Temperature = ((ADC_value - ADC_30C) * (110 - 30)) / (ADC_110C - ADC_30C) + 30
 *
 * where:
 *   - ADC_value   : Raw ADC measurement from temperature sensor
 *   - ADC_30C     : Calibrated ADC value at 30°C (stored in system memory)
 *   - ADC_110C    : Calibrated ADC value at 110°C (stored in system memory)
 *   - 30 and 110  : Reference temperatures
 *
 * @param au16_raw_adc Raw ADC value read from the temperature sensor channel
 * @return float Temperature in degrees Celsius corresponding to the ADC reading
 */
float convert_adc_to_temperature(uint16_t au16_raw_adc) {
    uint16_t lu16_TS_CAL1 = *TEMP30_CAL_ADDR;
    uint16_t lu16_TS_CAL2 = *TEMP110_CAL_ADDR;
    return ((float)(au16_raw_adc - lu16_TS_CAL1) * (TEMP110 - TEMP30)) /
           (lu16_TS_CAL2 - lu16_TS_CAL1) + TEMP30;
}

/***********************************************
 * TASK FUNCTIONS
 ***********************************************/

/**
 * @brief Initialize all the Mutexes
 * 
 * @param [None]
 * @return [None]
 */
void init_mutexes(void) {
  gx_temp_mutex_handle = osMutexNew(&gx_mutex_attr);
  if (NULL == gx_temp_mutex_handle)
  {
    Error_Handler(); /* Mutex creation failed */
  }
}

/**
 * @brief Initialize all the Tasks
 * 
 * @param [None]
 * @return [None]
 */
void init_tasks(void) {
  gx_adc_task_handle = osThreadNew(adc_task, NULL, &gx_adc_task_attributes);
  if (NULL == gx_adc_task_handle)
  {
    Error_Handler();
  }
  gx_led_task_handle = osThreadNew(led_task, NULL, &gx_led_task_attributes);
  if (NULL == gx_led_task_handle)
  {
    Error_Handler();
  }
  gx_uart_task_handle = osThreadNew(uart_task, NULL, &gx_uart_task_attributes);
  if (NULL == gx_uart_task_handle)
  {
    Error_Handler();
  }
}

/* ADC Task: reads internal temp sensor every 1s */
void adc_task(void *argument) {
  for (;;) {
    /* Start the ADC and get the raw value by waiting - Interrupt/DMA can be used here */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t lu16_raw_temp = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float lf_temp = convert_adc_to_temperature(lu16_raw_temp);

    if (osMutexWait(gx_temp_mutex_handle, osWaitForever) == osOK) {
      gf_temperature = lf_temp;
      osMutexRelease(gx_temp_mutex_handle);
    }

    osDelay(1000);
  }
}

/* LED Task: sets or toggles LED based on temperature */
void led_task(void *argument) {
  for (;;) {
    float lf_temp;

    if (osOK == osMutexWait(gx_temp_mutex_handle, osWaitForever))
    {
      lf_temp = gf_temperature;
      osMutexRelease(gx_temp_mutex_handle);
    }

    if (lf_temp > TEMP_HIGH_THRESHOLD) {
      /* Toggle the LED if temperature is above 29 degree Celcius*/
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
      osDelay(100);
    } else if (lf_temp > TEMP_LOW_THRESHOLD) {
      /* Turn on the LED if temperature is above 26 degree Celcius */
      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
      osDelay(1000);
    } else {
      /* Turn Off the LED if temperature is below 26 degree Celcius */
      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
      osDelay(1000);
    }
  }
}

/* UART Task: logs temperature every 1s over uart */
void uart_task(void * argument) {
  char l_msg_buff[64]; /* Buffer for logging the temperature */
  for (;;) {
    float lf_temp;

    osMutexWait(gx_temp_mutex_handle, osWaitForever);
    lf_temp = gf_temperature;
    osMutexRelease(gx_temp_mutex_handle);

    snprintf(l_msg_buff, sizeof(l_msg_buff), "Temp: %.2f C\r\n", lf_temp);
    HAL_UART_Transmit(&huart2, (uint8_t *)l_msg_buff, strlen(l_msg_buff), HAL_MAX_DELAY);

    osDelay(1000);
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
