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
 * @brief   This is an idea of an example obtained from the Mastering the FreeRTOS Real Time Kernel v1.0.
 *          Listings 5.28, 5.30 and 5.32. Let's call it "Example 5.Y Mailbox Queue"
 *          
 *          This demonstrates how to create a mailbox, how to update (overwrite) a mailbox every 100ms
 *          and how to peek from it (which is not the same as receiving/reading from a mailbox/queue) every 100ms.
 * 
 *          A mailbox can hold a fixed sized data item.
 *          The size of the data item is set when the mailbox (queue) is created.
 *          In this example the mailbox is created to hold an Example_t structure which
 *          includes a timestamp to allow the data held in the mailbox to note the time
 *          at which the mailbox was last updated. This time stamp is used for demonstration
 *          purposes only.
 * 
 *          A mailbox can hold any data the application writer wants, and the data does not
 *          need to include a time stamp
 *
 * @version 0.1
 * @date    2024-10-23
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
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STRING_SIZE    (24)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* create the data type (structure) the mailbox will contain */
typedef struct xExampleStructure
{
    TickType_t xTimeStamp;
    uint32_t   ulValue;
} Example_t;

/* a mailbox is a queue, so its handle is stored in a variable of type QueueHandle_t */
QueueHandle_t xMailbox;

/* declare a buffer that will contain a string to print for each FreeRTOS task */
char pcString[ STRING_SIZE ];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );
static void vUpdateMailboxTask( void *pvParameters );
static void vPeekMailboxTask( void *pvParameters );
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

  /* Create the queue that is going to be used as a mailbox.
     The queue has a length of 1 to allow it to be used with the xQueueOverWrite() API */
  xMailbox = xQueueCreate( 1, sizeof( Example_t ) );

  /* Create the task that will overwrite the contents in the mailbox queue.
     The task is created with a priority of 2, so above the priority of the peeker task */
  ( void ) xTaskCreate( vUpdateMailboxTask, "Update M.box Task", 1000, ( void* ) &pcString, 2, NULL );

  /* Create the task that will peek from the queue.
     The task is created with a priority of 1, so below the priority of the overwriter task */
  ( void ) xTaskCreate( vPeekMailboxTask, "Peek M.box Task", 1000, ( void* ) &pcString, 1, NULL );

  /* start the scheduler so the tasks start executing */
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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

/* a task that overwrites the data to the globally created mailbox */
static void vUpdateMailboxTask( void *pvParameters )
{
  /* declare a pointer that will point to the string to print */
  char *pcStr = ( char * ) pvParameters;

  /* get 100 milliseconds in Ticks used for delay */
  const TickType_t xDelay100ms = pdMS_TO_TICKS( 100 );

  /* declare the variable that will hold the data to overwrite into the mailbox */
  Example_t xData;

  /* declare the 'data' variable used to overwrite the data in the Example_t struct
     to be stored in the mailbox */
  volatile uint32_t ulNewValue = 0;

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* write the new data into the Example_t structure */
    xData.ulValue = ulNewValue;

    /* use the RTOS tick count as the time stamp stored in the Example_t structure */
    xData.xTimeStamp = xTaskGetTickCount();

    /* send the structure to the mailbox, overwritting any data that is already there */
    ( void ) xQueueOverwrite( xMailbox, &xData );

    /* print the data sent to the mailbox */
    ( void ) snprintf( pcStr, STRING_SIZE, "Data Sent = %lu\n\r", ulNewValue );
    UART2_Print_Text( &huart2, ( const char * ) pcStr );

    /* increment the variable so that the data overwritten into the mailbox is different on each
       iteration of this task */
    ulNewValue++;

    /* place this task into Blocked state for a 100ms-period */
    vTaskDelay( pdMS_TO_TICKS( xDelay100ms ) );
  }
}

/* A task that peeks the data from the globally created mailbox */
static void vPeekMailboxTask( void *pvParameters )
{
  /* declare a pointer that will point to the string to print */
  char *pcStr = ( char * ) pvParameters;

  /* declare the variable that will hold the data peeked from the mailbox */
  Example_t  xMailboxPeek;

  /* get 100 milliseconds in Ticks used for delay */
  const TickType_t xDelay100ms = pdMS_TO_TICKS( 100 );

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* Update the Example_t structure pointed to by xMailboxPeek.

       If xQueueReceive() was used here then the mailbox would be left empty,
       and the data could not be read by any other tasks.
       
       Using xQueuePeek() instead of xQueueReceive() ensures the data remains in the mailbox.
       
       A block time is not specified, as this task will execute with a lower priority than
       the overwriter task */
    ( void ) xQueuePeek( xMailbox, &xMailboxPeek, 0 );

    /* print the data peeked from the mailbox */
    ( void ) snprintf( pcStr, STRING_SIZE, "Data Peeked = %lu\n\r", xMailboxPeek.ulValue );
    UART2_Print_Text( &huart2, ( const char * ) pcStr );
    ( void ) snprintf( pcStr, 24, "Timestamp = %lu\n\n\r", xMailboxPeek.xTimeStamp );
    UART2_Print_Text( &huart2, ( const char * ) pcStr );

    /* place this task into Blocked state for a 100ms-period */
    vTaskDelay( pdMS_TO_TICKS( xDelay100ms ) );
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