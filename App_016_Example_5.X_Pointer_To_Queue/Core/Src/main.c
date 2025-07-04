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
 *          Listings 5.13, 5.14 and 5.15. Let's call it "Example 5.X Queue of Pointers"
 *          
 *          This demonstrates how to create a queue that can hold up to 5 pointers (queueing pointers),
 *          instead of copying the data itself from one task to another.
 * 
 *          The vStringSendingTask() allocates a buffer dynamically and writes a string into it, then
 *          this task sends the pointer to the buffer to the queue.
 * 
 *          The vStringReceivingTask() receives the pointer to a buffer (i.e. a string) from the queue,
 *          then prints it out.
 *
 * @version 0.1
 * @date    2024-10-21
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* declare a variable of type QueueHandle_t to hold the handle of the
   queue being created */
QueueHandle_t xQueueOfPointers;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void vStringSendingTask( void *pvParameters );
static void vStringReceivingTask( void *pvParameters );
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

  /* create a queue that can hold a maximum of 5 pointers, in this case
   character pointers (i.e. 5 strings) */
  xQueueOfPointers = xQueueCreate( 5, sizeof ( char * ) );

  /* Create the task that will write to the queue.
     The task is created with priority 1, so below the priority of the receiver task */
  xTaskCreate( vStringSendingTask, "Sending Task", 1000, NULL, 1, NULL );

  /* Create the task that will read from the queue.
     The task is created with priority 2, so above the priority of the sender task */
  xTaskCreate( vStringReceivingTask, "Receiving Task", 1000, NULL, 2, NULL );

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

/* a task that obtains a buffer, writes a string to the buffer, then
   sends the address of the buffer to the globally created queue of pointers */
static void vStringSendingTask( void *pvParameters )
{
  /* declare the variable that will point to the string to write to the queue */
  char *pcStringToSend;

  /* declare the variable that will hold the status of xQueueSendToBack() */
  BaseType_t xStatus;

  /* declare the variable that contains the maximum amount of characters the string can hold */
  const size_t xMaxStringLength = 50;

  /* declare the string counter, used to indicate the string number that was sent */
  uint8_t xStringNumber = 0;

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* obtain (dynamically) a buffer that is at least xMaxStringLength (50) characters big */
    pcStringToSend = ( char * ) pvPortMalloc( xMaxStringLength );

    /* write a string into the buffer */
    ( void ) snprintf( pcStringToSend, xMaxStringLength, "String number = %d\n\r", xStringNumber );

    /* increment the counter so the string is different on each iteration of this task */
    xStringNumber++;

    /* Send the address of the buffer to the queue of pointers that was globally created.d
    
       The first parameter is the queue to which data is being sent.
       The queue was created before the scheduler was started, so
       before this task started to execute.
       
       The second parameter is the address of the data to be sent,
       in this case the address of pcStringToSend.
       
       The third parameter is the Block time - the maximum amount of time
       the task should be kept in the Blocked state to wait for space to become available on
       the queue if it is already full.
       In this case the block time will cause the task to wait indefinitely since portMAX_DELAY
       is used and INCLUDE_vTaskSuspend is set to 1 */
    xStatus = xQueueSendToBack( xQueueOfPointers, &pcStringToSend, portMAX_DELAY );

    if ( xStatus == pdPASS )
    {
      /* data (pointer) was successfully sent to the queue,
         print out the string it points to */
      UART2_Print_Text( &huart2, "Sent:     " );
      UART2_Print_Text( &huart2, ( const char * ) pcStringToSend );
    }
    else
    {
      /* data (pointer) could not be sent to the queue,
         print error message */
      UART2_Print_Text( &huart2, "Write to queue error!\n\r" );
    }
  }
}

/* A task that receives the address of a buffer from the globally created queue.
   The buffer contains a string, which is printed out */
static void vStringReceivingTask( void *pvParameters )
{
  /* declare the variable that will hold the received string from the queue */
  char *pcReceivedString;

  /* declare the variable that will hold the status of xQueueReceive() */
  BaseType_t xStatus;

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* Receive the address from the queue.
    
       The first parameter is the queue from which data is to be received.
       The queue is created before the scheduler is started, and therefore
       before this task runs for the first time.
       
       The second parameter is the buffer into which the received data will
       be placed. In this case the buffer is simply the address of a
       variable that has the required size to hold the received data.
       
       The last parameter is the block time - the maximum amount of time
       that the task should be kept in the Blocked state to wait for data to
       be available if it is already empty.
       
       In this case the block time will cause the task to wait indefinitely since portMAX_DELAY
       is used and INCLUDE_vTaskSuspend is set to 1 */
    xStatus = xQueueReceive( xQueueOfPointers, &pcReceivedString, portMAX_DELAY );

    if ( xStatus == pdPASS )
    {
      /* data (pointer) was successfully received from the queue,
         print out the string it points to */
      UART2_Print_Text( &huart2, "Received: " );
      UART2_Print_Text( &huart2, ( const char * ) pcReceivedString );

      vPortFree( pcReceivedString );
    }
    else
    {
      /* Data was not received from the queue even after waiting indefinitely.
        This must be an error as the sending task is free running and will
        be continuously writing to the queue */
      UART2_Print_Text( &huart2, "Could not receive from the queue\n\r" );
    }
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
