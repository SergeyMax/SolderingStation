
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define TIP_DISABLED            -1
#define TIP_DELAY               100 // ms
#define IWDG_REFRESH_DELAY      100 // ms
#define LED_REFRESH_DELAY       300 // ms
#define HELLO_DELAY             1000 // ms
#define X_RES                   5 // char columns
#define Y_RES                   8 // char rows
#define BAR_LENGTH              14 // chars
#define POWER_THRESHOLD         400000 // ~0.4W/measurement
  
uint32_t hal_tick = 0;
uint32_t hal_tick_prev = 0;
uint32_t voltage = 0;
uint32_t current = 0;
int32_t power = 0;
int32_t power_prev = 0;
int32_t tip_delay = TIP_DISABLED;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM17_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//==================================================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if( GPIO_Pin & GPIO_PIN_9 )
  {
    CLEAR_BIT( EXTI->IMR, GPIO_PIN_9 ); // temporarily disable EXTI line to avoid tip detector glitches
    __HAL_GPIO_EXTI_CLEAR_IT( GPIO_PIN_9 ); //clear pending interrupt request, if exists
    
    HAL_TIM_Base_Start_IT( &htim17 ); //start one-shot debouncing timer
  }
}

//==================================================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM17 )
  {
    if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET ) // tip disconnected
    {
      //HAL_TIM_PWM_Stop cannot be used here
      //because it switches TIM1 output to the noise sensitive 3-rd state
      __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY( &htim1 ); // stop RF generator immediately
      tip_delay = TIP_DISABLED;
    }
    else
      if( tip_delay == TIP_DISABLED ) // avoid restart of the running generator 
        tip_delay = TIP_DELAY; // schedule RF generator startup
    
    SET_BIT( EXTI->IMR, GPIO_PIN_9 ); // enable EXTI line
  }
}

//==================================================================================================
void HAL_RCC_CSSCallback(void)
{
  _Error_Handler(__FILE__, __LINE__); // restart on crystal failure
}

//==================================================================================================
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_CLEAR_FLAG( &htim17, TIM_FLAG_UPDATE ); // clear pending timer update interrupt 
  __HAL_SYSCFG_BREAK_LOCKUP_LOCK(); // break TIM1 on MCU lockup
  __HAL_SYSCFG_BREAK_SRAMPARITY_LOCK(); // break TIM1 on RAM parity error

  oled_init();
  lcd_clear();
  
  lcd_position( 0x00 ); // Hello, world!
  lcd_prints( "Soldering System" );
  lcd_position( 0x40 ); 
  lcd_prints( " ChBRR (C) 2018 " );

  HAL_IWDG_Refresh( &hiwdg );
  
  for( unsigned int i = 0; i < HELLO_DELAY / IWDG_REFRESH_DELAY; i++ )
  {
    HAL_Delay( IWDG_REFRESH_DELAY );
    HAL_IWDG_Refresh( &hiwdg );
  }

  if( HAL_ADCEx_Calibration_Start( &hadc ) != HAL_OK )
    _Error_Handler(__FILE__, __LINE__);
  
  lcd_command( 0x48 ); // uploading custom 5x8 fonts into the CGRAM with base address 08
  uint8_t dots = 0;
  
  // char(1) means |, char(2) means ||, and so on up to char(X_RES)
  for( int x = 0; x < X_RES; x++ )
  {
    dots += 1 << (X_RES - 1 - x);

    for( int y = 0; y < Y_RES; y++ )
      lcd_printc( dots );
  }

  lcd_position( 0x00 ); // switch back to the DDRAM
  
  if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET ) // tip is connected
  {
    tip_delay = 0;
    HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 ); // enable RF generator
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    do
    {
      HAL_PWR_EnterSLEEPMode( 0, PWR_SLEEPENTRY_WFI );
      hal_tick = HAL_GetTick();
    }
    while( hal_tick == hal_tick_prev ); // avoid same tick reentrance
    
    hal_tick_prev = hal_tick;
    
//    if( hal_tick % 200 == 0)
//      HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 ); // enable RF generator after delay
//    
//    if( hal_tick % 73 == 0)
//      HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_3 ); // stop RF generator immediately

    if( hal_tick % IWDG_REFRESH_DELAY == 0 )
    {
      HAL_IWDG_Refresh( &hiwdg );
      HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_1 ); //heartbeat just for fun
    }
    
    if( tip_delay > 0 )
    {
      tip_delay--;
      
      if( tip_delay == 0 )
        HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 ); // enable RF generator after delay
    }

    if( hal_tick % LED_REFRESH_DELAY == 0 )
    {
      if( tip_delay != 0 )
      {
        lcd_position( 0x00 );
        lcd_prints( "   TIP FAILED   " );
        lcd_position( 0x40 );
        lcd_prints( "                " );
        
        continue;
      }
      
      if( HAL_ADC_Start( &hadc ) != HAL_OK )
        _Error_Handler(__FILE__, __LINE__);
  
      if( HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK ) // 10-bit precision
        current = HAL_ADC_GetValue( &hadc );
      else
        _Error_Handler(__FILE__, __LINE__);

      if( HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK ) // 10-bit precision
        voltage = HAL_ADC_GetValue( &hadc );
      else
        _Error_Handler(__FILE__, __LINE__);
      
      power = voltage * current * 33 * 5; // full scale up to 33V*5A = 165W
      
      lcd_position( 0x00 );
      if( power - power_prev > POWER_THRESHOLD )
        lcd_prints( "      POWER \x7e   " );
      else
        lcd_prints( "      READY     " );
      power_prev = power;
      
      lcd_position( 0x4e ); // digits
      lcd_printd2( power >> 20 ); // 10+10 bit removed
      
      power = power * 7; // 70 power lines 
      power = power / 9; // for 90 W
      power = power >> 20;

      lcd_position( 0x40 ); // bar graph 
      
      for( int x = 0; x < BAR_LENGTH; x++ ) 
      {
        if( power <= 0 )
          lcd_printc( ' ' );
        
        if( power > 0 && power <= X_RES ) 
          lcd_printc( (uint8_t)power );

        if( power > X_RES) 
          lcd_printc( X_RES );

        power -= X_RES;
      }
    }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_10B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 2048;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_3;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 8135;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim17, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  NVIC_SystemReset();

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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  _Error_Handler( file, line );

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
