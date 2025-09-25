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
#include "stm32f1xx_hal_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 定义RGB灯的颜色
typedef enum {
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  WHITE,
  OFF
} LED_ColorTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 定义RGB灯的引脚和端口
#define KEY1_PIN        GPIO_PIN_0
#define KEY1_GPIO_PORT  GPIOA
#define KEY2_PIN        GPIO_PIN_1
#define KEY2_GPIO_PORT  GPIOA

#define LED_R_PIN       GPIO_PIN_0
#define LED_R_GPIO_PORT GPIOB
#define LED_G_PIN       GPIO_PIN_1
#define LED_G_GPIO_PORT GPIOB
#define LED_B_PIN       GPIO_PIN_10
#define LED_B_GPIO_PORT GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 当前LED颜色状态
LED_ColorTypeDef current_color = OFF;

// 按键消抖用计时器
uint32_t key1_debounce_time = 0;
uint32_t key2_debounce_time = 0;
uint32_t debounce_delay = 500; // 500ms消抖时间

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue);
void RGB_ToggleColor(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  设置RGB灯颜色
  * @param  red: 0 = 熄灭, 1 = 点亮
  * @param  green: 0 = 熄灭, 1 = 点亮
  * @param  blue: 0 = 熄灭, 1 = 点亮
  * @retval None
  */
void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue) {
  // RGB为共阳极，点亮时引脚拉低
  HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_PIN, red ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_PIN, green ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_PIN, blue ? GPIO_PIN_RESET : GPIO_PIN_SET);  
}

/**
  * @brief  切换到下一个颜色
  * @param  None
  * @retval None
  */
void RGB_ToggleColor(void) {
  current_color = (current_color + 1) % 8; // 循环切换颜色

  switch(current_color) {
    case RED:
      RGB_SetColor(1, 0, 0);
      break;
    case GREEN:
      RGB_SetColor(0, 1, 0);
      break;
    case BLUE:
      RGB_SetColor(0, 0, 1);
      break;
    case YELLOW:
      RGB_SetColor(1, 1, 0);
      break;
    case CYAN:
      RGB_SetColor(0, 1, 1);
      break;
    case MAGENTA:
      RGB_SetColor(1, 0, 1);
      break;
    case WHITE:
      RGB_SetColor(1, 1, 1);
      break;
    case OFF:
      RGB_SetColor(0, 0, 0);
      break;
  }
}

/**
  * @brief  按键中断回调函数
  * @param  GPIO_Pin: 触发中断的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  uint32_t current_time = HAL_GetTick();

  if (GPIO_Pin == KEY1_PIN)  {
    // 按键1消抖处理
    if (current_time - key1_debounce_time > debounce_delay) {
      RGB_ToggleColor(); // 切换颜色
      key1_debounce_time = current_time;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_PIN); // 清除中断标志
  } else if (GPIO_Pin == KEY2_PIN) {
    // 按键2消抖处理
    if (current_time - key2_debounce_time > debounce_delay) {
      RGB_ToggleColor(); // 切换颜色
      key2_debounce_time = current_time;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY2_PIN); // 清除中断标志
  }
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

  // 初始化RGB灯为关闭状态
  RGB_SetColor(0, 0, 0);

  // 初始化为红色
  current_color = RED;
  RGB_SetColor(1, 0, 0);

  // 使能按键中断
  HAL_NVIC_EnableIRQ(EXTI0_IRQn); // KEY1对应EXTI0
  HAL_NVIC_EnableIRQ(EXTI1_IRQn); // KEY2对应EXTI1

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 主循环中不需要做任何事情，所有操作都在中断中处理
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
    // 错误状态下闪烁LED
    RGB_SetColor(1, 0, 0); // 红色
    HAL_Delay(500);
    RGB_SetColor(0, 0, 0); // 关闭
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
