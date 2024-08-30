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
 * 
 * @brief   This is the "Example 5.3 Using a Queue Set" from the Mastering the FreeRTOS
 *          Real Time Kernel v1.0 document.
 *          
 *          This example creates two sending tasks and one receiving task:
 *          - the sending tasks send data to the receiving task on two separate queues.
 *          - the two queues are added to a queue set.
 *          - the receiving task reads from the queue set to determine which of the two
 *            queues contain data.
 * 
 *          The first sending task uses xQueue1 to send a character pointer to the receiving
 *          task every 100 milliseconds.
 *          The second sending task uses xQueue2 to send a character pointer to the receiving
 *          task every 200 milliseconds.
 *          The character pointers point to a string that identifies the sending task.
 * 
 *          The two queues written to by the sending tasks are members of the same queue set.
 *          Each time a task sends to one of the queues, the handle of the queue is sent to
 *          the queue set.
 *          
 *          The receiving task calls xQueueSelectFromSet() to read the queue handles from the
 *          que set. After the receiving task receives a queue handle from the set, it knows
 *          the queue referenced by the received handle contains data, so it read the data
 *          from the queue directly.
 * 
 *          If a call to xQueueSelectFromSet() times out, it returns NULL. In the vReceiverTask()
 *          function, xQueueSelectFromSet() is called with an indefinite block time, so it will
 *          never time out, and can only return a valid queue handle. Therefore, the receiving
 *          task does not need to check to see if xQueueSelectFromSet() returned NULL before using
 *          the returned value.
 * 
 *          It is not 100% the same as it appears in the guide, since it's explained it in a
 *          general way, therefore this example was adapted by me to run on a STM32F407 discovery
 *          board using the FreeRTOS GCC port for this device.
 * 
 * @version 0.1
 * @date    2024-08-29
 * 
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* Declare two variables of type QueueHandle_t.
   Both queues will be added to the same queue set */
static QueueHandle_t xQueue1 = NULL;
static QueueHandle_t xQueue2 = NULL;

/* Declare a variable of type QueueSetHandle_t.
   This is the queue set to which the two queues are added */
static QueueSetHandle_t xQueueSet = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

void vSenderTask1( void *pvParameters );
void vSenderTask2( void *pvParameters );
void vReceiverTask( void *pvParameters );
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
  
  /* Create the two queues, both of which send character pointers.
     The priority of the receiving task is above the priority of the
     sending tasks, so the queues will never have more than one item
     in them at any one time */
  xQueue1 = xQueueCreate( 1, sizeof( char * ) );
  xQueue2 = xQueueCreate( 1, sizeof( char * ) );

  /* Create the queue set.
     Two queues will be added to the set, each of which can contain 1 item,
     so the maximum number of queue handles the queue set will ever
     have to hold at one item is 2 (2 queues multiplied by 1 item per queue) */
  xQueueSet = xQueueCreateSet( 1 * 2 );

  /* add the two queues to the set */
  xQueueAddToSet( xQueue1, xQueueSet );
  xQueueAddToSet( xQueue2, xQueueSet );

  /* create the tasks that send to the queues (their priority is lower than that of
     the receiving task) */
  xTaskCreate( vSenderTask1, "Sender1", 1000, NULL, 1, NULL );
  xTaskCreate( vSenderTask2, "Sender2", 1000, NULL, 1, NULL );

  /* create the task that reads from the queue set to determine which of the
     two queues contain data (higher priority than sending tasks) */
  xTaskCreate( vReceiverTask, "Receiver", 1000, NULL, 2, NULL );

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

void vSenderTask1( void *pvParameters )
{ 
  /* get 100 milliseconds equivalency in tick interrupts */
  const TickType_t xBlockTime = pdMS_TO_TICKS( 100 );

  /* string to be printed out by task 1 */
  const char * const pcMessage = "Message from vSenderTask1\n\r";

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* block for 100ms */
    vTaskDelay( xBlockTime );

    /* Send this task's string to xQueue1. It is not necessary to use a
       block time, even though the queue can only hold one item. This is
       because the priority of the task that reads from the queue is
       higher than the priority of this task; as soon as this task writes
       to the queue it will be pre-empted by the task that reads from the
       queue, so the queue will already be empty again by the time the
       call to xQueueSend() returns. The block time is set to 0 */
    xQueueSend( xQueue1, &pcMessage, 0 );
  }
}

void vSenderTask2( void *pvParameters )
{
  /* get 200 milliseconds equivalency in tick interrupts */
  const TickType_t xBlockTime = pdMS_TO_TICKS( 200 );

  /* string to be printed out by task 2 */
  const char * const pcMessage = "Message from vSenderTask2\n\r";

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* block for 200ms */
    vTaskDelay( xBlockTime );

    /* Send this task's string to xQueue2. It is not necessary to use a
       block time, even though the queue can only hold one item. This is
       because the priority of the task that reads from the queue is
       higher than the priority of this task; as soon as this task writes
       to the queue it will be pre-empted by the task that reads from the
       queue, so the queue will already be empty again by the time the
       call to xQueueSend() returns. The block time is set to 0 */
    xQueueSend( xQueue2, &pcMessage, 0 );
  }
}

void vReceiverTask( void *pvParameters )
{
  /* declare the variable that will hold the queue read from the queue set */
  QueueHandle_t xQueueThatContainsData;

  /* declare the variable that will point to the message read by the queue */
  char *pcReceivedString;

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* Block on the queue set to wait for one of the queues in the set to
       contain data.
       Cast the QueueSetMemberHandle_t value returned from
       xQueueSelectFromSet() to a QueueHandle_t, as it is known all the
       members of the set are queues (the queue set does not contain any
       semaphores) */
    xQueueThatContainsData = ( QueueHandle_t ) xQueueSelectFromSet( xQueueSet, portMAX_DELAY );

    /* An indefinite block time was used when reading from the queue set,
       so xQueueSelectFromSet() will not have returned unless one of the
       queues in the set contained data, and xQueueThatContainsData cannot
       be NULL.
       Read from the queue. It is not necessary to specify a
       block time because it is known the queue contains data.
       The block time is set to 0 */
    xQueueReceive( xQueueThatContainsData, &pcReceivedString, 0 );

    /* print the string received from the queue */
    UART2_Print_Text( &huart2, pcReceivedString );
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
