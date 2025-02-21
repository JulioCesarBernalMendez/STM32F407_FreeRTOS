/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * 
  ******************************************************************************
  */

/**
 * @author  Julio Cesar Bernal Mendez
 * @brief   This is the "Example 10.2 Using a task notification in place of a semaphore,
 *          method 2" from the Mastering the FreeRTOS Real Time Kernel v1.0 book.
 *          
 *          In example 10.1, the ulTaskNotifyTake() xClearOnExit parameter was set to pdTRUE.
 *          Example 10.2 modifies example 10.1 slightly to demonstrate the behavior when the
 *          ulTaskNotifyTake() xClearOnExit parameter is instead set to pdFALSE.
 * 
 *          When xClearOnExit is pdFALSE, calling ulTaskNotifyTake() will only decrement
 *          (reduce by one) the calling task's notification value, instead of clearing it
 *          to zero.
 *          
 *          The notification count is therefore the difference between the number of events
 *          that have occurred, and the number of events that have been processed.
 *          This allows the structure of vHandlerTask() to be simplified in two ways:
 *          1. The number of events waiting to be processed is held in the notification
 *             value, so it doesn't need to be held in a local variable.
 *          2. It is only necessary to process one event between each call to ulTaskNotifyTake().
 * 
 *          For demonstration purposes the TIM3 timeout interrupt service routine
 *          (HAL_TIM_PeriodElapsedCallback()) has also been modified to send more than one task
 *          notification per interrupt, and in so doing, simulate the multiple interrupts
 *          occurring at high frequency.
 * 
 *          The TIM3 timeout is configured to occur every 500 milliseconds,
 *          although in a real application an interrupt is supposed to occur at any time, this is
 *          used only for simplicity.
 *          The handler (TIM3_IRQHandler()) of the TIM3 interrupt simply executes a callback
 *          (HAL_TIM_PeriodElapsedCallback()) which gives the task notification using the
 *          interrupt safe version (vTaskNotifyGiveFromISR()) API and subsequently checks whether
 *          a context switch is required or not by using the portYIELD_FROM_ISR() macro.
 * 
 *          The task to which interrupt processing is deferred (vHandlerTask()) is the task that is
 *          synchronized with the interrupt through the use of the task notification.
 *          This task remains blocked thanks to the ulTaskNotifyTake() xTicksToWait parameter,
 *          once the notification is taken the task enters the ready state again and simply prints out
 *          a string indicating the TIM3 event is being processed.
 * 
 *          NOTES: - the priority of the interrupt cannot be higher than the configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 *                   symbolic constant. In the context of STM32 microcontrollers this can be confusing as
 *                   a higher numeric value means a lower priority. In other words the numeric value of the
 *                   interrupt cannot be lower (higher priority) than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY and
 *                   not the other way around as one may think.
 *                 - The interrupt controller (NVIC) allows the bits that define each interrupt's
 *                   priority to be split between bits that define the interrupt's pre-emption priority
 *                   bits and bits that define the interrupt's sub-priority. For simplicity all bits
 *                   must be defined to be pre-emption priority bits. If this is not the case then
 *                   an assertion will fail causing the program to get stuck inside an infinite loop.
 *                   (refer to "configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue )"
 *                   in port.c)
 *                   
 * @version 0.1
 * @date    2024-12-26
 * 
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* the rate at which the TIM3 generates a timeout interrupt
   note: actual configuration is done in MX_TIM3_Init() */
const TickType_t xInterruptFrequency = pdMS_TO_TICKS( 500 );

/* vHandlerTask() task handler.
   This will be used to send a notification directly to the task
   to which interrupt processed will be deferred */
TaskHandle_t xHandlerTask;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void vHandlerTask( void *pvParameters );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* enable CYCCNT (Cycle Count, needed for SEGGER SystemView) in DWT_CTRL register */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* initialize and configure SEGGER SystemView */
  SEGGER_SYSVIEW_Conf();

  /* start recording SEGGER SystemView events */
  SEGGER_SYSVIEW_Start();
  
  /* Create the 'handler' task, which is the task to which interrupt processing
     is deferred. This is the task that will be synchronized with the interrupt.
     (The handler task must be created with a high priority to ensure it runs
     immediately after the interrupt exits. But since this is the only task
     running for this example, then a priority of 1 is chosen).

     The pxCreatedTask (task handler) parameter is used to pass a task notification
     from within the TIM3 timeout interrupt via vTaskNotifyGiveFromISR() */
  xTaskCreate( vHandlerTask, "Handler", 1000, NULL, 1, &xHandlerTask );

  /* start the TIM3 in interrupt mode */
  HAL_TIM_Base_Start_IT( &htim3 );

  /* start the scheduler */
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* If all is well, main() will not reach here because the scheduler will now
     be running the created tasks.
     If main() does reach here, then there was not enough heap memory to create either
     the idle or timer tasks */
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 12499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_MCK_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text )
{
  uint8_t character;

  /* loop through the string until null character found */
  for ( character = 0; text[ character ] != '\0'; character++ )
  {
    /* transmit current character over UART */
    HAL_UART_Transmit( huart, ( const uint8_t* ) &text[ character ], 1, 5000 );
  }
}

static void vHandlerTask( void *pvParameters )
{
  /* xMaxExpectedBlockTime is set to be a little longer than the maximum expected time 
     between TIM3 timeout events */
  const TickType_t xMaxExpectedBlockTime = xInterruptFrequency + pdMS_TO_TICKS( 10 );

  /* as per most tasks, this task is implemented within an infinite loop */
  for ( ;; )
  {
    /* Wait to receive a notification sent directly to this task from the 
       TIM3 timeout interrupt service routine.
       The xClearCountOnExit parameter is now pdFALSE, so the task's notifification value
       will be decremented by ulTaskNotifyTake() and not cleared to zero */
    if ( ulTaskNotifyTake( pdFALSE, xMaxExpectedBlockTime ) != 0 )
    {
      /* To get here an event must have occurred.
         Process the event (in this case just print out a message) */
      UART2_Print_Text( &huart2, "Handler task - Processing event.\r\n" );
    }
    else
    {
      /* If this part of the function is reached, then a TIM3 timeout interrupt did not arrive
         within the expected time.
         Indicate this by printing out an error message.
         (in a real application it may be necessary to perform some error recovery operations) */
      UART2_Print_Text( &huart2, "Handler task - Error.\r\n" );
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3/TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* variable used only when TIM3 interrupt is triggered */
  BaseType_t xHigherPriorityTaskWoken;

  /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE
     as it will get set to pdTRUE inside the interrupt safe API function
     if a context switch is required */
  xHigherPriorityTaskWoken = pdFALSE;

  if ( htim->Instance == TIM3 )
  {
    /* used for debugging purposes (check TIM3 period via a logic analyzer) */
    HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_12 );

    /* print out a message */
    UART2_Print_Text( &huart2, "ISR Handler - About to give a notification.\r\n" );

    /* Send a notification directly to the task to which interrupt processing is being deferred.
       In this case the notification is sent multiple times.
       The first 'give' will unblock the task, the following 'gives' are to demonstrate
       that the receiving task's notification value is being used to count (latch events),
       allowing the task to process each event in turn */
    vTaskNotifyGiveFromISR( xHandlerTask, &xHigherPriorityTaskWoken );
    vTaskNotifyGiveFromISR( xHandlerTask, &xHigherPriorityTaskWoken );
    vTaskNotifyGiveFromISR( xHandlerTask, &xHigherPriorityTaskWoken );

    /* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR():
       - If xHigherPriorityTaskWoken was set to pdTRUE inside vTaskNotifyGiveFromISR(),
         then calling portYIELD_FROM_ISR() will request a context switch.
       - If xHigherPriorityTaskWoken is still pdFALSE then calling portYIELD_FROM_ISR()
         will have no effect */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
