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
 * @brief   This is the "Example 5.2 Blocking when sending to a queue, and sending structures on a queue"
 *          from the Mastering the FreeRTOS Real Time Kernel v1.0 document.
 *          
 *          This examples demonstrates creating a queue, sending data to the queue from multiple tasks,
 *          and receiving data from the queue.
 *          The queue is created to hold data items of a user defined struct Data_t.
 *          The task that receives from the queue does not specify a block time, whereas the tasks that send
 *          to the queue does.
 *  
 *          Example 5.2 is similar to Example 5.1, but with reversed task prioritites, so the receiving task
 *          has a lower priority than the sending tasks. Also, the created queue holds structures rather
 *          than integers.
 * 
 *          2 instances of the sending task are created, one that writes value xStructsToSend[ 0 ]
 *          to the queue and another that writes the value xStructsToSend[ 1 ] to the same queue.
 * 
 *          Since the sending tasks have higher priority than the receiving task, the queue
 *          will normally be full. This is because, as soon as the receiving task removes an item from
 *          the queue, it is pre-empted by one of the sending tasks which then immediately re-fills the queue.
 *          The sending task then re-enters the Blocked state to wait for space to become available on the
 *          queue again.
 * 
 *          The sending task specifies a block time of 100 milliseconds, so it enters the Blocked state to wait
 *          for space to be available each time the queue becomes full. It (sending task) leaves the Blocked
 *          state when either space is available on the queue, or 100 milliseconds passes without space
 *          becoming available. In this example, the receiving task continuously make space in the queue,
 *          so the 100 milliseconds timeout never expires.
 * 
 *          It is not 100% the same as the guide explains it in a general way, meaning it was adapted by me to run on
 *          a STM32F407 discovery board using the FreeRTOS GCC port for this device.
 * @version 0.1
 * @date    2024-08-28
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
/* Declare a variable of type QueueHandle_t. This is used to store the
   handle to the queue that is accessed by all three tasks */
QueueHandle_t xQueue;

/* string array for printing the received Queue values */
char pcReceivedQueue[ 22 ];

/* define an enumerated type used to identify the source of the data */
typedef enum
{
  eSender1,
  eSender2
} DataSource_t;

/* define the structure type that will be passed on the queue */
typedef struct
{
  DataSource_t eDataSource;
  uint8_t      ucValue;
} Data_t;

/* declare two variables of type Data_t that will be passed on the queue */
static const Data_t xStructsToSend[ 2 ] =
{
  { eSender1, 100 }, /* used by Sender1 */
  { eSender2, 200 }  /* used by Sender2 */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void vSenderTask( void *pvParameters );
static void vReceiverTask( void *pvParameters );
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

  /* the queue is created to hold a maximum of 3 structures of type Data_t */
  xQueue = xQueueCreate( 3, sizeof( Data_t ) );

  /* Create two instances of the task that will write to the queue.
     The task parameter is used to pass the structure that the task will
     write to the queue, so one task will send xStructsToSend[ 0 ] to the queue,
     while the other task will send xStructsToSend[ 1 ] to the queue.
     Both tasks are created at priority 2, which is above the priority
     of the receiver */
  xTaskCreate( vSenderTask, "Sender 1", 1000, ( void * ) &( xStructsToSend[ 0 ] ), 2, NULL );
  xTaskCreate( vSenderTask, "Sender 2", 1000, ( void * ) &( xStructsToSend[ 1 ] ), 2, NULL );

  /* Create the task that will read from the queue.
     The task is created with priority 1, so below the priority of the sender tasks */
  xTaskCreate( vReceiverTask, "Receiver", 1000, NULL, 1, NULL );

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

static void vSenderTask( void *pvParameters )
{
  /* declare the variable that will hold the status of xQueueSendToBack() */
  BaseType_t xStatus;

  /* get 100 milliseconds equivalency in tick interrupts */
  const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* Send the value to the queue.
    
       The first parameter is the queue to which data is being sent.
       The queue was created before the scheduler was started, so
       before this task started to execute.
       
       The second parameter is the address of the data to be sent (in this case
       a Data_t structure). The address is passed in as the task parameter so
       pvParameters is used directly.
       
       The third parameter is the Block time - the time the task should be
       kept in the Blocked state to wait for space to become available on
       the queue if the queue is already full.
       A block time is specified because the sending tasks have a higher priority
       than the receiving task so the queue is expected to become full.
       The receiving task will remove items from the queue when both sending tasks
       are in the Blocked state */
    xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );

    if ( xStatus != pdPASS )
    {
      /* The send operation could not complete, even after waiting for 100ms.
         This must be an error as the receiving task should make space in the queue
         as soon as both sending tasks are in the Blocked state */
      UART2_Print_Text( &huart2, "Could not send to the queue\n\r" );
    }
  }
}

static void vReceiverTask( void *pvParameters )
{
  /* declare the structure that will hold the values received from the queue */
  Data_t xReceivedStructure;

  /* declare the variable that will hold the status of xQueueReceive() */
  BaseType_t xStatus;

  /* as per most tasks, this task is implemented in an infinite loop */
  for ( ;; )
  {
    /* Because it has the lowest priority this task will only run when the
       sending tasks are in the Blocked state.
       The sending tasks will only enter the Blocked state when the queue is
       full, so this task always expects the number of items in the queue to
       be equal to the queue length, which is 3 in this case */
    if ( uxQueueMessagesWaiting( xQueue ) != 3 )
    {
      UART2_Print_Text( &huart2, "Queue should have been full!\n\r" );
    }

    /* Receive data from the queue.
    
       The first parameter is the queue from which data is to be received.
       The queue is created before the scheduler is started, and therefore
       before this task runs for the first time.
       
       The second parameter is the buffer into which the received data will
       be placed. In this case the buffer is simply the address of a
       variable that has the required size to hold the received structure (Data_t).
       
       The last parameter is the block time - the maximum amount of time
       that the task will remain in the Blocked state to wait for data to
       be available if the queue is already empty.
       In this case a block time is not necessary because this task will only
       run when the queue is full */
    xStatus = xQueueReceive( xQueue, &xReceivedStructure, 0 );

    if ( xStatus == pdPASS )
    {
      /* data was successfully received from the queue,
         print out the received value and the source of the value */
      if ( xReceivedStructure.eDataSource == eSender1 )
      {
        sprintf( pcReceivedQueue, "From Sender 1 = %d\n\r", xReceivedStructure.ucValue );
      }
      else
      {
        sprintf( pcReceivedQueue, "From Sender 2 = %d\n\r", xReceivedStructure.ucValue );
      }

      UART2_Print_Text( &huart2, ( const char * ) pcReceivedQueue );
    }
    else
    {
      /* Nothing was received from the queue.
         This must be an error as this task should only run when the queue is full */
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
