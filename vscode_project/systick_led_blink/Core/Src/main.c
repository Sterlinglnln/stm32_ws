/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_it.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 定义LED引脚和端口
#define LEDR_PIN GPIO_PIN_0
#define LEDR_GPIO_PORT GPIOB
#define LEDG_PIN GPIO_PIN_1
#define LEDG_GPIO_PORT GPIOB
#define LEDB_PIN GPIO_PIN_10
#define LEDB_GPIO_PORT GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint32_t tick_count = 0; // 毫秒计数器
volatile uint8_t led_state = 0;  // LED状态变量, 0: 红, 1: 绿, 2: 蓝

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void SysTick_Init(uint32_t ticks);
void SysTick_Handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  初始化SysTick定时器
  * @param  ticks: 定时器重载值
  * @retval None
  */
void SysTick_Init(uint32_t ticks) {
  // 关闭SysTick定时器
  SysTick->CTRL = 0;

  // 设置重载寄存器
  SysTick->LOAD = ticks - 1;

  // 设置中断优先级
  NVIC_SetPriority(SysTick_IRQn, 0x0);

  // 重置计数器
  SysTick->VAL = 0;

  // 启用SysTick定时器，选择系统时钟，并启用中断
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | 
                 SysTick_CTRL_TICKINT_Msk | 
                 SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief  SysTick中断服务程序
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  tick_count++;

  // 每1000ms切换一次LED状态
  if (tick_count % 1000 == 0) {
    // 关闭所有LED
    HAL_GPIO_WritePin(LEDR_GPIO_PORT, LEDR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEDG_GPIO_PORT, LEDG_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEDB_GPIO_PORT, LEDB_PIN, GPIO_PIN_SET);

   // 根据当前状态点亮对应的LED
    switch (led_state) {
      case 0:
        HAL_GPIO_WritePin(LEDR_GPIO_PORT, LEDR_PIN, GPIO_PIN_RESET); // 点亮红色LED
        break;
      case 1:
        HAL_GPIO_WritePin(LEDG_GPIO_PORT, LEDG_PIN, GPIO_PIN_RESET); // 点亮绿色LED
        break;
      case 2:
        HAL_GPIO_WritePin(LEDB_GPIO_PORT, LEDB_PIN, GPIO_PIN_RESET); // 点亮蓝色LED
        break;
    }

    // 更新LED状态
    led_state = (led_state + 1) % 3; // 循环切换状态
  }

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

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
  /* USER CODE BEGIN 2 */

  // 初始化SysTick定时器，系统时钟为72MHz，每1ms中断一次
  SysTick_Init(72000);

  // 初始化时，关闭所有LED
  HAL_GPIO_WritePin(LEDR_GPIO_PORT, LEDR_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEDG_GPIO_PORT, LEDG_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEDB_GPIO_PORT, LEDB_PIN, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 主循环中不需要做任何事情，所有操作都在SysTick中断中处理
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
    // 错误时红灯闪烁
    HAL_GPIO_TogglePin(LEDR_GPIO_PORT, LEDR_PIN);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
