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
 * @brief   This is the "Example 9.2 Synchronizing tasks"
 *          from the Mastering the FreeRTOS Real Time Kernel v1.0 book.
 *          
 *          This example uses xEventGroupSync() to synchronize three instances of a
 *          single task implementation.
 *          The task parameter is used to pass into each instance the event bit the
 *          task will set when it calls xEventGroupSync().
 * 
 *          The task prints a message before calling xEventGroupSync(),
 *          and again after the call to xEventGroupSync() has returned.
 *          Each message includes a time stamp. This allows the sequence of execution
 *          to be observed in the output produced.
 *          A pseudo random delay (200ms to 699ms) is used to prevent all the tasks reaching
 *          the synchronization point at the same time.
 * 
 *          Even though each task reaches the synchronization point at a different
 *          (pseudo random) time, each task exits the synchronization point at the
 *          same time (which is the time at which the last task reached the
 *          synchronization point).
 *                   
 * @version 0.1
 * @date    2024-12-20
 * 
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* definitions for the event bits in the event group */
#define mainTHIRD_TASK_BIT     ( 1UL << 2UL ) /* event bit 2, set by the third  instance of task vSyncingTask() */
#define mainSECOND_TASK_BIT    ( 1UL << 1UL ) /* event bit 1, set by the second instance of task vSyncingTask() */
#define mainFIRST_TASK_BIT     ( 1UL << 0UL ) /* event bit 0, set by the first  instance of task vSyncingTask() */

#define STRING_SIZE            ( 50 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
EventGroupHandle_t xEventGroup;
char cOutputString[ STRING_SIZE ];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void vSyncingTask( void *pvParameters );
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
  /* USER CODE BEGIN 2 */
  /* enable CYCCNT (Cycle Count, needed for SEGGER SystemView) in DWT_CTRL register */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* initialize and configure SEGGER SystemView */
  SEGGER_SYSVIEW_Conf();

  /* start recording SEGGER SystemView events */
  SEGGER_SYSVIEW_Start();

  /* before an event group can be used, it must first be created */
  xEventGroup = xEventGroupCreate();

  /* Create three instances of the task.
     Each task is given a different name, which is later printed out to give
     a visual indication of which task is executing.
     The event bit to use when the task reaches its synchronization point is
     passed into the task using the task parameter */
  xTaskCreate( vSyncingTask, "Task 1", 1000, ( void * const ) mainFIRST_TASK_BIT, 1, NULL );
  xTaskCreate( vSyncingTask, "Task 2", 1000, ( void * const ) mainSECOND_TASK_BIT, 2, NULL );
  xTaskCreate( vSyncingTask, "Task 3", 1000, ( void * const ) mainTHIRD_TASK_BIT, 3, NULL );

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

static void vSyncingTask( void *pvParameters )
{
  TickType_t       xDelayTime; /* used to get a pseudo random delay (200ms to 699ms) */
  const TickType_t xMaxDelay = pdMS_TO_TICKS( 500UL );
  const TickType_t xMinDelay = pdMS_TO_TICKS( 200UL );
  
  EventBits_t       uxThisTaskSyncBit; /* event bit to set by this task */
  const EventBits_t uxAllSyncBits = ( mainTHIRD_TASK_BIT | mainSECOND_TASK_BIT | mainFIRST_TASK_BIT ); /* event bits to sync */

  /* Three instances of this task are created - each task uses a different event bit
     in the synchronization.
     The event bit to use is passed into each task instance by using the task
     parameter.
     Store it in the uxThisTaskSyncBit variable */
  uxThisTaskSyncBit = ( EventBits_t ) pvParameters;

  for ( ;; )
  {
    /* Simulate this task taking some time to perform an action by delaying for
       a pseudo random time (200ms to 699ms).
       This prevents all three instances of this task reaching the synchronization
       point at the same time, and so allows the example's behavior to be
       observed more easily */
    xDelayTime = ( rand() % xMaxDelay ) + xMinDelay;
    vTaskDelay( xDelayTime );

    /* Print out a message to show this task has reached its synchronization point.
       pcTaskGetTaskName() is an API function that returns the name assigned
       to the task when the task was created */
    ( void ) snprintf( cOutputString, STRING_SIZE, "At time %10lu: %s reached sync point\n\r",
                       xTaskGetTickCount(), pcTaskGetName( NULL ) );
    UART2_Print_Text( &huart2, ( const char * ) cOutputString );

    /* wait for all the task to reach their respective synchronization points */
    xEventGroupSync( xEventGroup,       /* event group used to synchronize    */
                     uxThisTaskSyncBit, /* bit set by this task to indicate it has
                                           reached its synchronization point  */
                     uxAllSyncBits,     /* bits to wait for, one for each task
                                           taking part in the synchronization */
                     portMAX_DELAY      /* wait indefinitely for all three tasks
                                           to reach the synchronization point */
                   );

    /* Print out a message to show this task has passed its synchronization point.
       As an indefinite delay (200ms to 699ms) was used, the following line
       will only be executed after all the three tasks reached their respective
       synchronization point */
    ( void ) snprintf( cOutputString, STRING_SIZE, "At time %10lu: %s  exited sync point\n\r",
                       xTaskGetTickCount(), pcTaskGetName( NULL ) );
    UART2_Print_Text( &huart2, ( const char * ) cOutputString );
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  
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
