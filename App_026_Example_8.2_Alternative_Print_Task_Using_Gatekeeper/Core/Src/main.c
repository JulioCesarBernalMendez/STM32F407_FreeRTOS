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
 * @brief   This is the "Example 8.2 The alternative implementatio for print task"
 *          from the Mastering the FreeRTOS Real Time Kernel v1.0 book.
 *          Note: I'm not using the windows port, instead this project uses the ARM_CM4F port
 *                and it uses my own implementation for printing from UART2 to a serial terminal.
 *                This function is UART2_Print_Text().
 *                
 *          As in example 8.1, two separate instances of the task that prints (prvPrintTask())
 *          are created. In this case the string to print is passed to a queue using the
 *          task parameter.
 * 
 *          The tick hook function (vApplicationTickHook()) counts the number of times it's called,
 *          sending its message to the gatekeeper task (prvUART2GatekeeperTask()) each time the count
 *          reaches 150.
 *          For demonstration purposes only the tick hook writes to the front
 *          of the queue, and the print tasks write to the back of the queue.
 * 
 *          The scheduler always executes immediately after the tick hook function so interrupt safe
 *          FreeRTOS API called from the tick hook don't need to use their pxHigherPriorityTaskWoken
 *          parameter (it can be set to NULL).
 * 
 *          The strings originating from the print tasks and from the interrupt all print out
 *          correctly with no corruption.
 * 
 *          The gatekeeper task is assigned a lower priority than the print tasks, so messages sent
 *          to the gatekeeper remain in the queue until both print tasks are in Blocked state.
 *          In some situations, it would be appropriate to assign the gatekeeper a higher priority,
 *          so messages get processed immediately, but doing so would be at the cost of the
 *          gatekeeper delaying lower priority tasks until it has completed accessing the protected
 *          resource.
 
 *          The frequency of pre-emption can be increased by reducing the maximum time the tasks
 *          spend in the Blocked state, which is set by the xMaxBlockTimeTicks constant.
 * 
 *          The random string ordering is a result of the random delay periods used by the tasks.
 *                   
 * @version 0.1
 * @date    2024-12-16
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
/* Declare a variable of type QueueHandle_t.
   The queue is used to send messages from the print tasks and the tick interrupt
   to the gatekeeper task */
QueueHandle_t xPrintQueue;

/* define the strings that the tasks and interrupt will print out via the gatekeeper */
static char *pcStringsToPrint[] =
{
  "Task 1 ******************************************\n\r",
  "Task 2 ------------------------------------------\n\r",
  "Message printed from the tick hook interrupt ####\r\n"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

void vApplicationTickHook( void );
static void prvUART2GatekeeperTask( void *pvParameters );
static void prvPrintTask( void *pvParameters );
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

  /* Before a queue is used it must be explicitly created.
     The queue is created to hold a maximum of 5 character pointers */
  xPrintQueue = xQueueCreate( 5, sizeof( char * ) );

  /* check the queue was created successfully */
  if ( xPrintQueue != NULL )
  {
    /* Create two instances of the tasks that send messages to the
       gatekeeper.
       The index to the string the task uses is passed to the task
       via the task parameter (the fourth parameter to xTaskCreate()).
       The tasks are created at different priorities so the higher
       priority task will occasionally preempt the lower priority task */
    xTaskCreate( prvPrintTask, "Print1", 1000, ( void * ) 0, 1, NULL );
    xTaskCreate( prvPrintTask, "Print2", 1000, ( void * ) 1, 2, NULL );

    /* Create the gatekeeper task.
       This is the only task that is permitted to directly access UART2 */
    xTaskCreate( prvUART2GatekeeperTask, "Gatekeeper", 1000, NULL, 0, NULL );

    /* start the scheduler so the created tasks start executing */
    vTaskStartScheduler();
  }

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

void vApplicationTickHook( void )
{
  static int iCount = 0;

  /* Print out a message every 150 ticks.
     The message is not written out directly, but sent to the gatekeeper task */
  
  iCount++;

  if ( iCount >= 150 )
  {
    /* As xQueueSendToFrontFromISR() is being called from the tick hook, it is not
       necessary to use the xHigherPriorityTaskWoken parameter (the third parameter),
       and the parameter is set to NULL */
    xQueueSendToFrontFromISR( xPrintQueue, &pcStringsToPrint[ 2 ], NULL );

    /* reset the count ready to print out the string again in 150 ticks time */
    iCount = 0;
  }
}

static void prvUART2GatekeeperTask( void *pvParameters )
{
  char *pcMessageToPrint;

  /* This is the only task that is allowed to write from UART2.
     Any other task wanting to write a string to UART2 does not access UART2 directly,
     but instead sends the string to this task.
     As only this task accesses standard out there are no mutual exclusion or
     serialization issuess to consider within the implementation of the task itsel */
  for ( ;; )
  {
    /* Wait for a message to arrive.
       An indefinite block time is specified so there is no need to check the
       return value - the function will only return when a message has been
       successfully received */
    xQueueReceive( xPrintQueue, &pcMessageToPrint, portMAX_DELAY );

    /* output the received string */
    UART2_Print_Text( &huart2, pcMessageToPrint );
  }
}

static void prvPrintTask( void *pvParameters )
{
  int iIndexToString;
  const TickType_t xMaxBlockTimeTicks = 0x20;

  /* two instances of this task are created.
     The task parameter is used to pass an index into an array of strings
     into the task.
     Cast this to the required type */
  iIndexToString = ( int ) pvParameters;

  for ( ;; )
  {
    /* Print out the string, not directly, but instead by passing a pointer
       to the string to the gatekeeper task via a queue.
       The queue is created before the scheduler is started so will already
       exist by the time this task executes for the first time.
       A block time is not specified because there should always be space
       in the queue */
    xQueueSendToBack( xPrintQueue, &pcStringsToPrint[ iIndexToString ], 0 );

    /* Wait a pseudo random time (0 to 0x1F ticks).
       Note that rand() is not necessarily reentrant, but in this case it
       does not really matter as the code does not care what value is returned.
       In a more secure application, a version of rand() that is known to be 
       reentrant should be used - or calls to rand() should be protected
       using a critical section */
    vTaskDelay( rand() % xMaxBlockTimeTicks );
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
