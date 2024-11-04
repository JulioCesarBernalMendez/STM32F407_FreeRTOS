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
 * @brief   This is the "Example 6.3 Resetting a software timer"
 *          from the Mastering the FreeRTOS Real Time Kernel v1.0 book.
 *          
 *          This example simulates the behavior of the backlight on a cellphone.
 *          The backlight:
 *          - Turns on when the button is pressed.
 *          - Remains on when the button is pressed again within a certain time period.
 *          - Automatically turns off if the button is not pressed within a certain time period.
 * 
 *          A one-shot timer (5 seconds) is used to implement this behavior:
 *          - The simulated backlight is turned on when a key is pressed, and turned off in the
 *            software timer's callback function (vBacklightTimerCallback).
 *          - The software timer is reset each time a key is pressed.
 *          - The time period during which a key must be pressed to prevent the backlight being turned off
 *            is therefore equal to the period of the software timer; if the software timer is not reset by
 *            a key press before the timer expires, then the timer's callback function executes, and the
 *            the backlight is turned off.
 * 
 *          Example 6.3 creates a task to poll the button, ideally an interrupt should be used otherwise
 *          when polling is used, processing time is wasted waiting for events that have not occurred.
 *          Example 6.3 is not event driven because interrupt management techniques are covered later.
 *
 * @version 0.1
 * @date    2024-11-4
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
#define STRING_SIZE                   (62)

/* the period assigned to the one-shot timer is 5000 milliseconds */
#define mainBACKLIGHT_TIMER_PERIOD    pdMS_TO_TICKS( 5000 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* char array used to print out via UART */
char     pcOutputString[ STRING_SIZE ];

/* variable that holds the backlight timer handle */
TimerHandle_t xBacklightTimer;

/* variable that holds the status of xTaskCreate() for the button task */
BaseType_t xButtonTaskCreated;

/* variable that holds the status of xTimerStart() for the software timer */
BaseType_t xBacklightTimerStarted;

/* variable that holds the status of the backlight (ON/OFF) */
BaseType_t xSimulatedBacklightOn = pdFALSE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_Print_Text( UART_HandleTypeDef *huart, const char *text );

static void vBacklightTimerCallback( TimerHandle_t xTimer );
static void vButtonHitTask( void *pvParameters );
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
  /* create the button polling task */
  xButtonTaskCreated =  xTaskCreate( vButtonHitTask, "Button", 1000, NULL, 1, NULL );

  /* create the one-shot timer, storing the handle to the created timer in xOneShotTimer */
  xBacklightTimer = xTimerCreate( "Backlight", /* text name for the software timer - not used by FreeRTOS */
                                  mainBACKLIGHT_TIMER_PERIOD, /* software timer's period in ticks */
                                  pdFALSE,   /* setting xAutoReload to pdFALSE creates a one-shot software timer */
                                  NULL,      /* the timer's ID is initialized to NULL */
                                  vBacklightTimerCallback /* vBacklightTimerCallback function is used by this software timer */
                                );

  /* check both the button task and the software timer (backlight) were created */
  if ( ( xButtonTaskCreated == pdPASS ) && ( xBacklightTimer != NULL ) )
  {
    /* Start the software timer (backlight) using a block time of 0 (no block time).
       The scheduler has not been started yet so any block time specified
       here would be ignored anyway */
    xBacklightTimerStarted = xTimerStart( xBacklightTimer, 0 );

    /* The implementation of xTimerStart() uses the timer command queue,
       and xTimerStart() will fail if the timer command queue gets full.
       The timer service task does not get created until the scheduler is
       started, so all commands sent to the command queue will stay in the
       queue until after the scheduler has been started. Check the call
       to xTimerStart() passed */
    if ( xBacklightTimerStarted == pdPASS )
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

static void vBacklightTimerCallback( TimerHandle_t xTimer )
{
  /* record the time at which the one-shot timer callback was executed */
  TickType_t xTimeNow = xTaskGetTickCount();

  /* the backlight timer expired, indicate the backlight is off */
  xSimulatedBacklightOn = pdFALSE;

  /* print the time at which the backlight was turned off */
  ( void ) snprintf( pcOutputString, STRING_SIZE, "Timer expired, turning backlight OFF at time     %lu\n\r", xTimeNow );
  UART2_Print_Text( &huart2, ( const char * ) pcOutputString );
}

static void vButtonHitTask( void *pvParameters )
{
  const TickType_t xShortDelay = pdMS_TO_TICKS( 50 );
  TickType_t xTimeNow;

  /* prompt user to press the button */
  UART2_Print_Text( &huart2, "Press button to turn the backlight on.\n\r" );

  /* Ideally an application would be event driven and use an interrupt to process button presses.
     Since interrupts are covered later in the book, this task is used to poll for a button press */
  for ( ;; )
  {
    /* has the button been pressed */
    if ( HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ) == GPIO_PIN_SET )
    {
      /* button has been pressed, record the time */
      xTimeNow = xTaskGetTickCount();

      /* Reset the software timer:
       - if the backlight was previously off, then this call will start the timer.
       - if the backlight was previously on, then this call will restart the timer.

       A real application may read key presses in an interrupt. If this function was an ISR,
       then xTimerResetFromISR() must be used instead of xTimerReset() */
      xTimerReset( xBacklightTimer, xShortDelay );

      /* if backlight already went off */
      if ( xSimulatedBacklightOn == pdFALSE )
      {
        /* the backlight was off, so turn it on and print the time at which it was turned on */
        xSimulatedBacklightOn = pdTRUE;
      
        ( void ) snprintf( pcOutputString, STRING_SIZE, "Button pressed, turning backlight ON at time     %lu\n\r", xTimeNow );
        UART2_Print_Text( &huart2, ( const char * ) pcOutputString );
      }
      /* if backlight is still on */
      else
      {
        /* the backlight was already on, so print a message to say the timer is about to be reset
           and the time at which it was reset */
        ( void ) snprintf( pcOutputString, STRING_SIZE, "Button pressed, resetting software timer at time %lu\n\r", xTimeNow );
        UART2_Print_Text( &huart2, ( const char * ) pcOutputString );
      }
    }

    /* delay used to prevent many button presses are processed at once */
    vTaskDelay( pdMS_TO_TICKS( 200 ) );
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
