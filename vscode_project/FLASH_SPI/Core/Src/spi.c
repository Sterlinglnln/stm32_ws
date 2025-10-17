/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef SPI1_TransferBuffer(const uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout);
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint8_t SPI1_TransferByte(uint8_t tx_byte)
{
  uint8_t rx_byte = 0U;
  if (SPI1_TransferBuffer(&tx_byte, &rx_byte, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
  return rx_byte;
}

HAL_StatusTypeDef SPI1_Transmit(const uint8_t *data, uint16_t length, uint32_t timeout)
{
  return SPI1_TransferBuffer(data, NULL, length, timeout);
}

HAL_StatusTypeDef SPI1_Receive(uint8_t *data, uint16_t length, uint32_t timeout)
{
  return SPI1_TransferBuffer(NULL, data, length, timeout);
}

static HAL_StatusTypeDef SPI1_TransferBuffer(const uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout)
{
  for (uint16_t i = 0U; i < length; ++i)
  {
    uint32_t tickstart = HAL_GetTick();
    while ((SPI1->SR & SPI_SR_TXE) == 0U)
    {
      if ((HAL_GetTick() - tickstart) > timeout)
      {
        return HAL_TIMEOUT;
      }
    }

    uint8_t tx_byte = (tx != NULL) ? tx[i] : 0xFFU;
    *((__IO uint8_t *)&SPI1->DR) = tx_byte;

    tickstart = HAL_GetTick();
    while ((SPI1->SR & SPI_SR_RXNE) == 0U)
    {
      if ((HAL_GetTick() - tickstart) > timeout)
      {
        return HAL_TIMEOUT;
      }
    }

    uint8_t rx_byte = *((__IO uint8_t *)&SPI1->DR);
    if (rx != NULL)
    {
      rx[i] = rx_byte;
    }
  }

  uint32_t tickstart = HAL_GetTick();
  while ((SPI1->SR & SPI_SR_BSY) != 0U)
  {
    if ((HAL_GetTick() - tickstart) > timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/* USER CODE END 1 */
