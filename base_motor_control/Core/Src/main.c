#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>
#include <string.h>

void SystemClock_Config(void);

// Motor control pins and functions
#define MOTOR_DIR1_Pin GPIO_PIN_4
#define MOTOR_DIR1_Port GPIOA
#define MOTOR_DIR2_Pin GPIO_PIN_5
#define MOTOR_DIR2_Port GPIOA

typedef enum {
  MOTOR_FORWARD,  // Forward direction
  MOTOR_BACKWARD, // Backward direction
  MOTOR_STOP      // Stop
} MotorState;

void Motor_SetDirection(MotorState state) {
    switch (state) {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(MOTOR_DIR1_Port, MOTOR_DIR1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_DIR2_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
            break;
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(MOTOR_DIR1_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_DIR2_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
            break;
        case MOTOR_STOP:
        default:
            HAL_GPIO_WritePin(MOTOR_DIR1_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_DIR2_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
            break;
    }
}

void Motor_SetSpeed(uint16_t duty_cycle) {
  if (duty_cycle > 99) duty_cycle = 99; // Cap at 99%
  TIM2->CCR3 = duty_cycle; // Assuming TIM2 Channel 3 is used for PWM
}

// USART communication functions
#define RX_BUFFER_SIZE 64
char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t rx_data;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    rx_buffer[rx_index++] = rx_data;
    // Check for newline or buffer overflow
    if (rx_data == '\n' || rx_index >= RX_BUFFER_SIZE - 1) {
      rx_buffer[rx_index] = '\0';
      // Process command
      if (strncmp(rx_buffer, "forward", 7) == 0) {
        Motor_SetDirection(MOTOR_FORWARD);
        Motor_SetSpeed(50); // Example speed
      } else if (strncmp(rx_buffer, "backward", 8) == 0) {
        Motor_SetDirection(MOTOR_BACKWARD);
        Motor_SetSpeed(50); // Example speed
      } else if (strncmp(rx_buffer, "stop", 4) == 0) {
        Motor_SetDirection(MOTOR_STOP);
      }
      rx_index = 0; // Reset buffer index
    }
    // Restart reception
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
}

int main(void)
{
  // Initialize all configured peripherals
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  // Start PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  // Start UART reception in interrupt mode
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);

  while (1)
  {
  }

}

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
