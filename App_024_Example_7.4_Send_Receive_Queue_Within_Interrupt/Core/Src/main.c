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
 * @brief   This is the "Example 7.4 Sending and Receiving on a queue from within an interrupt"
 *          from the Mastering the FreeRTOS Real Time Kernel v1.0 book.
 *          
 *          This example demonstrates xQueueSendToBackFromISR() and xQueueReceiveFromISR()
 *          being used within the same interrupt. For convenience the interrupt is generated
 *          by software. This is the EXTI0 interrupt.
 * 
 *          A periodic task (vIntegerGenerator()) is created that sends five numbers to a
 *          queue every 500ms. It generates an interrupt (EXTI0) by software (although it
 *          could also be generated by hardware, but this is not intended as it breaks the
 *          program workflow) only after all five numbers have been sent.
 * 
 *          The EXTI0 interrupt callback (HAL_GPIO_EXTI_Callback()) calls xQueueReceiveFromISR()
 *          repeteadly until all the values written to the integer queue by the periodic task
 *          (vIntegerGenerator()) have been read out, and the integer queue is left empty.
 *          The last two bits of each received value are used as an index into an array of
 *          strings. A pointer to the string at the corresponding index position is then
 *          sent to a different queue (a string queue) using a call to xQueueSendFromISR().
 * 
 *          The task that receives the character pointers (i.e. the strings) from the
 *          EXTI0 interrupt callback (HAL_GPIO_EXTI_Callback()) blocks on the queue until a
 *          message arrives, printing out each string as it is received.
 * 
 *          main() creates the required queues and tasks before starting the scheduler.
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
 * @date    2024-11-16
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
#include "timers.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STRING_SIZE    (10)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char ucOutputString[ STRING_SIZE ];

QueueSetHandle_t xIntegerQueue;
QueueSetHandle_t xStringQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void vIntegerGenerator( void *pvParameters );
static void vStringPrinter( void *pvParameters );
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
  /* Before a queue can be used it must first be created.
     Create both queues used by this example:
     - one queue can hold variables of type uint32_t
     - the other queue can hold variables of type char*
     
     Both queues can hold a maximum of 10 items.
     A real application should check the return values to ensure the queues
     have been successfully created */
  xIntegerQueue = xQueueCreate( 10, sizeof( uint32_t ) );
  xStringQueue  = xQueueCreate( 10, sizeof( char* ) );

  /* Create the task that uses a queue to pass integers to the EXTI0 interrupt
     service routine. The task is created at priority 1 */
     xTaskCreate( vIntegerGenerator, "IntGen", 1000, NULL, 1, NULL );

  /* Create the task that prints out the strings sent to it from the EXTI0
     interrupt service routine. This task is created at a higher priority of 2 */
  xTaskCreate( vStringPrinter, "String", 1000, NULL, 2, NULL );

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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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

static void vIntegerGenerator( void *pvParameters )
{
  TickType_t xLastExecutionTime;
  uint32_t ulValueToSend = 0;
  int i;

  /* initialize the variable used by the call to vTaskDelayUntil() */
  xLastExecutionTime = xTaskGetTickCount();

  for ( ;; )
  {
    /* This is a periodic task. Block until it is time to run again.
       The task will execute every 200ms */
    vTaskDelayUntil( &xLastExecutionTime, pdMS_TO_TICKS( 200 ) );

    /* Send five numbers to the integer queue, each value one higher than the previous value.
       The numbers are read from the queue by the interrupt service routine (HAL_GPIO_EXTI_Callback()).
       The interrupt service routine always empties the integer queue, so this task is guaranteed to be
       able to write all five values without needing to specify a block time */
    for ( i = 0; i < 5; i++ )
    {
      xQueueSendToBack( xIntegerQueue, &ulValueToSend, 0 );
      ulValueToSend++;
    }

    /* Generate the EXTI0 timeout interrupt by software, so the EXTI0 callback can read the
       values from the integer queue */
    UART2_Print_Text( &huart2, "Generator task - About to generate an EXTI0 interrupt by software.\r\n" );
    HAL_NVIC_SetPendingIRQ( EXTI0_IRQn );
    UART2_Print_Text( &huart2, "Generator task - EXTI0 interrupt by software generated.\r\n\n\n" );
  }
}

static void vStringPrinter( void *pvParameters )
{
  char *pcString;

  for ( ;; )
  {
    /* block on the string queue for wait for data to arrive */
    xQueueReceive( xStringQueue, &pcString, portMAX_DELAY );

    /* print out the string received */
    ( void ) snprintf( ucOutputString, STRING_SIZE, "%s", pcString );
    UART2_Print_Text( &huart2, ( const char * ) ucOutputString );
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken;
  uint32_t ulReceivedNumber;

  /* The strings are declared static const to ensure they are not allocated on the
     interrupt service routine's stack, and so exist even when the interrupt
     service routine is not executing */
  static const char *pcStrings[] = { "String0\r\n", "String1\r\n", "String2\r\n", "String3\r\n" };

  /* if EXTI0 */
  if ( GPIO_Pin == GPIO_PIN_0 )
  {
    /* used for debugging purposes */
    HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_13 );

    /* As always, xHigherPriorityTaskWoken is initialized to pdFALSE to be able to detect
       it getting set to pdTRUE inside an interrupt safe API function. Note that as an
       interrupt safe API function can only set xHigherPriorityTaskWoken to pdTRUE,
       it is safe to use the same xHigherPriorityTaskWoken variable in both the call
       to xQueueReceiveFromISR() and the call to xQueueSendToBackFromISR() */
    xHigherPriorityTaskWoken = pdFALSE;

    /* while the integer queue is not empty, read from it until the queue is empty */
    while( xQueueReceiveFromISR( xIntegerQueue, &ulReceivedNumber, &xHigherPriorityTaskWoken ) != errQUEUE_EMPTY )
    {
      /* truncate the received value of the integer queue to the last two bits (values 0 to 3 inclusive),
         then use the truncated value as an index into the pcStrings[] array to select a string (char *)
         to send on the other string queue */
      ulReceivedNumber &= 0x03;
      xQueueSendToBackFromISR( xStringQueue, &pcStrings[ ulReceivedNumber ], &xHigherPriorityTaskWoken );
    }

    /* If receiving from xIntegerQueue caused a task to leave the Blocked state, and if the priority
       of the task that left the Blocked state is higher than the priority of the task in the Running state,
       then xHigherPriorityTaskWoken will have been set to pdTRUE inside xQueueReceiveFromISR().
       
       If sending to xStringQueue caused a task to leave the Blocked state, and if the priority of the
       task that left the Blocked state is higher than the priority of the task in the Running state,
       then xHigherPriorityTaskWoken will have been set to pdTRUE inside xQueueSendToBackFromISR().

       xHigherPriorityTaskWoken is used as the parameter to portYIELD_FROM_ISR().
       - If xHigherPriorityTaskWoken equals pdTRUE, then calling portYIELD_FROM_ISR() will request
         a context switch.
       - If xHigherPriorityTaskWoken is still pdFALSE, then calling portYIELD_FROM_ISR() will have no effect */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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