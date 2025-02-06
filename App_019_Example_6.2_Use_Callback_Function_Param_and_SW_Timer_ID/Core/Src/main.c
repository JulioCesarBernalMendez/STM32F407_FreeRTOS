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
 * @brief   This is the "Example 6.2 Using the callback function parameter and the software timer ID"
 *          from the Mastering the FreeRTOS Real Time Kernel v1.0 book.
 *          
 *          This example creates and starts a one-shot timer and an auto-reload timer.
 * 
 *          Example 6.1 used two separate callback functions, one callback was used by the
 *          one-shot timer, and the other callback function was used by the auto-reload timer.
 * 
 *          Example 6.2 creates similar functionality to that created by Example 6.1, but assigns
 *          a signle callback function (pvTimerCallback()) to both software timers.
 * 
 *          The same callback function can be assigned to more than one software timer. When that
 *          is done, the callback function parameter is used to determine which software timer
 *          expired.
 * 
 *          The auto-reload timer's executes the callback with a fixed period of 500 ticks
 *          as specified by mainAUTO_RELOAD_TIMER_PERIOD.
 *
 *          The one-shot timer's executes the callback only once, when the tick count is 3100 ticks
 *          as specified by mainONE_SHOT_TIMER_PERIOD.
 * 
 *          pvTimerCallback():
 *          - will execute when either timer (one-shot / auto-reload) expires.
 *          - uses the function's parameter to determine if it was called because the one-shot timer expired,
 *            or because the auto-reload timer expired
 *          - demonstrates how to use the software timer ID as timer specific storage; each software timer
 *            keeps a count of the number of times it has expired in its own ID, and the auto-reload timer
 *            uses the count to stop itself the fifth time it executes.
 *          
 *
 * @version 0.1
 * @date    2024-11-1
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STRING_SIZE                     (50)

/* the periods assigned to the one-shot and auto-reload timers are 3.100 seconds
   and 0.5 seconds respectively */
#define mainONE_SHOT_TIMER_PERIOD       pdMS_TO_TICKS( 3100 )
#define mainAUTO_RELOAD_TIMER_PERIOD    pdMS_TO_TICKS( 500 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char     ucOutputString[ STRING_SIZE ];

/* variables that hold the timer handles */
TimerHandle_t xAutoReloadTimer;
TimerHandle_t xOneShotTimer;

/* variables that hold the status of xTimerStart() for the 2 software timers */
BaseType_t    xTimer1Started;
BaseType_t    xTimer2Started;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void pvTimerCallback( TimerHandle_t xTimer );
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

  /* create the one-shot timer, storing the handle to the created timer in xOneShotTimer */
  xOneShotTimer = xTimerCreate( "OneShot", /* text name for the software timer - not used by FreeRTOS */
                                mainONE_SHOT_TIMER_PERIOD, /* software timer's period in ticks */
                                pdFALSE,   /* setting xAutoReload to pdFALSE creates a one-shot software timer */
                                NULL,      /* the timer's ID is initialized to NULL */
                                pvTimerCallback /* pvTimerCallback callback function is used by both software timers */
                              );

  /* create the auto-reload timer, storing the handle to the created timer in xAutoReloadTimer */
  xAutoReloadTimer = xTimerCreate( "AutoReload", /* text name for the software timer - not used by FreeRTOS */
                                   mainAUTO_RELOAD_TIMER_PERIOD, /* software timer's period in ticks */
                                   pdTRUE,       /* setting xAutoReload to pdTRUE creates an auto-reload timer */
                                   NULL,         /* the timer's ID is initialized to NULL */
                                   pvTimerCallback /* pvTimerCallback callback function is used by both software timers */
                                 );

  /* check the software timers were created */
  if ( ( xOneShotTimer != NULL ) && ( xAutoReloadTimer != NULL ) )
  {
    /* Start the software timers, using a block time of 0 (no block time).
       The scheduler has not been started yet so any block time specified
       here would be ignored anyway */
    xTimer1Started = xTimerStart( xOneShotTimer, 0 );
    xTimer2Started = xTimerStart( xAutoReloadTimer, 0 );

    /* The implementation of xTimerStart() uses the timer command queue,
       and xTimerStart() will fail if the timer command queue gets full.
       The timer service task does not get created until the scheduler is
       started, so all commands sent to the command queue will stay in the
       queue until after the scheduler has been started. Check both calls
       to xTimerStart() passed */
    if ( ( xTimer1Started == pdPASS ) && ( xTimer2Started == pdPASS ) )
    {
      /* start the scheduler */
      vTaskStartScheduler();
    }
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

static void pvTimerCallback( TimerHandle_t xTimer )
{
  TickType_t xTimeNow;
  uint32_t   ulExecutionCount;

  /* A count of the number of times this software timer has expired is stored in the timer's ID.
     Obtain the ID, increment it, then save it as the new ID value.
     The ID is a void pointer, so is cast to a uint32_t.
     
     note: since the timer IDs were set to NULL by default (when the timers were created via xTimerCreate()),
           the first time pvTimerGetTimerID() is called (for each timer) will assign a value of zero to
           ulExecutionCount since the timer IDs will return NULL (i.e. 0 when cast to integer) */
  ulExecutionCount = ( uint32_t ) pvTimerGetTimerID( xTimer ); /*  */
  ulExecutionCount++;
  vTimerSetTimerID( xTimer, ( void * ) ulExecutionCount );

  /* obtain the current tick count */
  xTimeNow = xTaskGetTickCount();

  /* The handle of the one-shot timer was stored in xOneShotTimer when the timer was created.
     Compare the handle passed into this function with xOneShotTimer to determine if it was the
     one-shot or auto-reload timer that expired, then output a string to show the time at which
     the callback was executed */
  if ( xTimer == xOneShotTimer )
  {
    ( void ) snprintf( ucOutputString, STRING_SIZE, "One-shot timer callback executing %lu\n\r", xTimeNow );
    UART2_Print_Text( &huart2, ( const char * ) ucOutputString );
  }
  else
  {
    /* xTimer did not equal xOneShotTimer, so it must have been the auto-reload timer that expired */
    ( void ) snprintf( ucOutputString, STRING_SIZE, "Auto-reload timer callback executing %lu\n\r", xTimeNow );
    UART2_Print_Text( &huart2, ( const char * ) ucOutputString );

    if ( ulExecutionCount == 5 )
    {
      /* Stop the auto-reload timer after it has executed 5 times.
         This callback function executes in the context of the RTOS daemon task
         so must not call any function that might place the daemon task into the
         Blocked state. Therefore a block time of 0 is used */
      xTimerStop( xTimer, 0 );
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
