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
 * @brief   This is the "Example 9.1 Experimenting with event groups"
 *          from the Mastering the FreeRTOS Real Time Kernel v1.0 book.
 *          
 *          This example demonstrates how to:
 *          - Create an event group.
 *          - Set bits in an event group from an ISR.
 *          - Set bits in an event group from a task.
 *          - Block on an event group.
 * 
 *          There are 3 events:
 *          - event bit 2, which is set from an ISR (TIM3 timeout - HAL_TIM_PeriodElapsedCallback())
 *          - event bits 1 and 0 which are set from a periodic task (vEventBitSettingTask())
 * 
 *          The task that sets event bits 1 and 0 sits in a loop, repeteadly setting one bit,
 *          then the other with a delay of 200 milliseconds between each call to xEventGroupSetBits().
 *          A string is printed out before each bit is set to allow the sequence of execution
 *          to be seen.
 * 
 *          The TIM3 timeout ISR that sets bit 2 in the event group also prints a string before
 *          the bit is set to allow the sequence of execution to be seen. In this case however,
 *          because the message printing should not be performed directly in an ISR,
 *          xTimerPendFunctionCallFromISR() is used to perform the output in the context of the
 *          RTOS daemon task.
 *          This interrupt service routine is triggered periodically every 500 milliseconds.
 * 
 *          The interrupt callback (HAL_TIM_PeriodElapsedCallback) when called by TIM3 calls
 *          xTimerPendFunctionCallFromISR() to pass a pointer to a function called
 *          vDeferredHandlingFunction() to the daemon task. This is used to defer interrupt
 *          processing so that the printing "Bit setting ISR  - about to set bit 2\n\r"
 *          is done within the daemon task rather than within the ISR.
 * 
 *          The task (vTim3InterruptEnablingTask()) is just created to start the TIM3 as a
 *          timeout interrupt at the highest priority, but it only executes once and deletes itself.
 *          TIM3 starts after the scheduler starts because otherwise the TIM3 timeout interrupt could
 *          execute and call xTimerPendFunctionCallFromISR(), causing an assert to place the program
 *          in an infinite 'for' loop due to the timer queue not being properly created beforehand.
 * 
 *          The task (vEventBitReadingTask()) that calls xEventGroupWaitBits() to block on the event group,
 *          prints out a string for each bit that is set in the event group.
 *          The xEventGroupWaitBits() xClearOnExit parameter is set to pdTRUE, so the event bit(s) that
 *          caused the call to xEventGroupWaitBits() to return will be cleared automatically before
 *          xEventGroupWaitBits() returns.
 * 
 *          The priority of the task (vEventBitReadingTask()) that reads from the event group is higher
 *          than the priority of the task that writes to the event group (vEventBitSettingTask()),
 *          ensuring the reading task will pre-empt the writing task each time the reading task's
 *          unblock condition is met.
 * 
 *          When the xEventGroupWaitBits() xWaitForAllBits parameter is set to pdFALSE, the task
 *          that reads from the event group leaves the Blocked state and executes immediately
 *          every time any of the event bits are set, this causes the corresponding message
 *          to be printed as soon as any event occurs.
 * 
 *          On the other hand, when the xEventGroupWaitBits() xWaitForAllBits parameter is set to
 *          pdTRUE, the task that read from the event group only leaves the Blocked state
 *          after all three event bits are set. This causes all the corresponding message to
 *          be printed only when the three events occur.
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
 * @date    2024-12-19
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
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* definitions for the vent bits in the event group */
#define mainISR_BIT            ( 1UL << 2UL ) /* event bit 2, set by an ISR (HAL_TIM_PeriodElapsedCallback()) */
#define mainSECOND_TASK_BIT    ( 1UL << 1UL ) /* event bit 1, set by a task (vEventBitSettingTask()) */
#define mainFIRST_TASK_BIT     ( 1UL << 0UL ) /* event bit 0, set by a task (vEventBitSettingTask()) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
EventGroupHandle_t xEventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void vTim3InterruptEnablingTask( void *pvParameter );
static void vEventBitSettingTask( void *pvParameter );
static void vEventBitReadingTask( void *pvParameter );
static void vDeferredHandlingFunction( void *pvParameter1, uint32_t ulParameter2 );
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
  
  /* before an event group can be used, it must first be created */
  xEventGroup = xEventGroupCreate();

  /* Create the task that will start the TIM3 timeout interrupt. This will only
     run once before being deleted by itself as it won't be needed anymore */
  xTaskCreate( vTim3InterruptEnablingTask, "TIM3 INT start", 1000, NULL, configMAX_PRIORITIES - 1, NULL );

  /* create the task that waits for event bits to get set in the event group */
  xTaskCreate( vEventBitReadingTask, "Bit Reader", 1000, NULL, 2, NULL );

  /* create the task that sets event bits in the event group */
  xTaskCreate( vEventBitSettingTask, "Bit Setter", 1000, NULL, 1, NULL );

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

static void vTim3InterruptEnablingTask( void *pvParameter )
{
  /* Start the TIM3 in interrupt mode.
     This is done in a task after the scheduler is started because otherwise
     the TIM3 timeout would be triggered and then its callback would try to defer the
     processing to the daemon task before the scheduler started, causing an assertion
     due to the timer queue not being properly created */
  HAL_TIM_Base_Start_IT( &htim3 );

  /* delete this task as it will be only required to run once at the beginning */
  vTaskDelete( NULL );
}

static void vEventBitSettingTask( void *pvParameter )
{
  const TickType_t xDelay200ms = pdMS_TO_TICKS( 200UL );

  for ( ;; )
  {
    /* delay for a short time before starting the next loop */
    vTaskDelay( xDelay200ms );

    /* print out a message to say event bit 0 is about to be set by the task,
       then set event bit 0 */
    UART2_Print_Text( &huart2, "Bit setting task - about to set bit 0\r\n" );
    xEventGroupSetBits( xEventGroup, mainFIRST_TASK_BIT );

    /* delay for a short while before setting the other bit */
    vTaskDelay( xDelay200ms );

    /* print out a message to say event bit 1 is about to be set by the task,
       then set event bit 1 */
    UART2_Print_Text( &huart2, "Bit setting task - about to set bit 1\r\n" );
    xEventGroupSetBits( xEventGroup, mainSECOND_TASK_BIT );
  }
}

static void vEventBitReadingTask( void *pvParameter )
{
  EventBits_t xEventGroupValue;
  const EventBits_t xBitsToWaitFor = ( mainISR_BIT | mainSECOND_TASK_BIT | mainFIRST_TASK_BIT );

  for ( ;; )
  {
    /* block to wait for event bits to become set within the event group */
    xEventGroupValue = xEventGroupWaitBits( xEventGroup,    /* the event group to read */
                                            xBitsToWaitFor, /* bits to test */
                                            pdTRUE,         /* clear bits on exit if the unblock condition is met */
                                            pdFALSE,        /* Don't wait for all bits. This parameter is set to
                                                               pdTRUE for the second execution */
                                            portMAX_DELAY   /* don't time out */
                                          );

    /* print a message for each bit that was set */
    if ( ( xEventGroupValue & mainFIRST_TASK_BIT ) == mainFIRST_TASK_BIT )
    {
      UART2_Print_Text( &huart2, "Bit reading task - Event bit 0 was set\n\r" );
    }

    if ( ( xEventGroupValue & mainSECOND_TASK_BIT ) == mainSECOND_TASK_BIT )
    {
      UART2_Print_Text( &huart2, "Bit reading task - Event bit 1 was set\n\r" );
    }

    if ( ( xEventGroupValue & mainISR_BIT ) == mainISR_BIT )
    {
      UART2_Print_Text( &huart2, "Bit reading task - Event bit 2 was set\n\r" );
    }
  }
}

static void vDeferredHandlingFunction( void *pvParameter1, uint32_t ulParameter2 )
{
  /* Process the event, in this case just print out a message.
     ulParameter2 is not used in this example */
  UART2_Print_Text( &huart2, ( const char * ) pvParameter1 );
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
  /* The string is not printed within the TIM3 interrupt service routine, but is instead
     sent to the RTOS daemon task for printing.
     It is therefore declared static to ensure the compiler does not allocate the
     string on the stack of the ISR, as the ISR's stack frame will not exist when
     the string is printed from the daemon task */
  static const char *pcString = "Bit setting ISR  - about to set bit 2\n\r";

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

    /* Print out a message to say bit 2 is about to be set.
       Messages should not be printed from an ISR (would take too much ISR processing time),
       so defer the actual output to the RTOS daemon task by pending a function call to run
       in the context of the RTOS daemon task.

       Send a pointer to the interrupt's deferred handling function to the daemon task:
       - The deferred handling function's pvParameter1 parameter is used to pass the string to print.
       - The deferred handling function's ulParameter2 parameter is not used so it's just set to 0. */
    xTimerPendFunctionCallFromISR(  vDeferredHandlingFunction, /* function to be executed by the daemon task */
                                    ( void * ) pcString,       /* pvParameter1 used as the string to print */
                                    0,                      /* ulParameter2 is not used */
                                    &xHigherPriorityTaskWoken  /* context switch flag updated by this FromISR() API */
                                 );

    /* set bit 2 in the event group */
    xEventGroupSetBitsFromISR( xEventGroup, mainISR_BIT, &xHigherPriorityTaskWoken );

    /* xTimerPendFunctionCallFromISR() and xEventGroupSetBitsFromISR() both write to the
       timer command queue, and both used the same xHigherPriorityTaskWoken variable.
       If writting to the timer command queue resulted in the RTOS daemon task leaving
       the Blocked state, and if the priority of the RTOS daemon task is higher than the
       priority of the currently executing task (the task that this interrupt interrupted)
       then xHigherPriorityTaskWoken will have been set to pdTRUE.
    
       Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR():
       - If xHigherPriorityTaskWoken was set to pdTRUE inside xTimerPendFunctionCallFromISR()
         or xEventGroupSetBitsFromISR, then calling portYIELD_FROM_ISR() will request a context switch.
       - If xHigherPriorityTaskWoken is still pdFALSE then calling portYIELD_FROM_ISR() will have no effect */
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
