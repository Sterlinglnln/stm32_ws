/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    flash.c
  * @brief   External FLASH operations over SPI
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
#include "flash.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */
#define FLASH_CMD_WRITE_ENABLE   0x06
#define FLASH_CMD_READ_STATUS    0x05
#define FLASH_CMD_PAGE_PROGRAM   0x02
#define FLASH_CMD_READ_DATA      0x03
#define FLASH_CMD_SECTOR_ERASE   0x20

#define FLASH_STATUS_BUSY_MASK   0x01
#define FLASH_DEFAULT_TIMEOUT_MS 5000U

static void Flash_Select(void);
static void Flash_Deselect(void);
static HAL_StatusTypeDef Flash_WriteEnable(void);
static uint8_t Flash_ReadStatus(void);
static HAL_StatusTypeDef Flash_WaitWhileBusy(uint32_t timeout_ms);
static HAL_StatusTypeDef Flash_PageProgram(uint32_t address, const uint8_t *data, uint32_t length);
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
void Flash_Init(void)
{
  Flash_Deselect();
  HAL_Delay(1);
}

HAL_StatusTypeDef Flash_Read(uint32_t address, uint8_t *buffer, uint32_t length)
{
  if ((buffer == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  uint8_t cmd[4] = {
      FLASH_CMD_READ_DATA,
      (uint8_t)(address >> 16U),
      (uint8_t)(address >> 8U),
      (uint8_t)(address)};

  Flash_Select();
  HAL_StatusTypeDef status = SPI1_Transmit(cmd, (uint16_t)sizeof(cmd), HAL_MAX_DELAY);
  if (status == HAL_OK)
  {
    status = SPI1_Receive(buffer, (uint16_t)length, HAL_MAX_DELAY);
  }
  Flash_Deselect();

  return status;
}

HAL_StatusTypeDef Flash_EraseSector(uint32_t address)
{
  HAL_StatusTypeDef status = Flash_WriteEnable();
  if (status != HAL_OK)
  {
    return status;
  }

  uint32_t sector_address = address & ~(EXT_FLASH_SECTOR_SIZE - 1U);
  uint8_t cmd[4] = {
      FLASH_CMD_SECTOR_ERASE,
      (uint8_t)(sector_address >> 16U),
      (uint8_t)(sector_address >> 8U),
      (uint8_t)(sector_address)};

  Flash_Select();
  status = SPI1_Transmit(cmd, (uint16_t)sizeof(cmd), HAL_MAX_DELAY);
  Flash_Deselect();

  if (status != HAL_OK)
  {
    return status;
  }

  return Flash_WaitWhileBusy(FLASH_DEFAULT_TIMEOUT_MS);
}

HAL_StatusTypeDef Flash_Write(uint32_t address, const uint8_t *data, uint32_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_OK;
  uint32_t remaining = length;
  uint32_t current_address = address;
  const uint8_t *current_data = data;

  while ((remaining > 0U) && (status == HAL_OK))
  {
    uint32_t page_offset = current_address % EXT_FLASH_PAGE_SIZE;
    uint32_t space_in_page = EXT_FLASH_PAGE_SIZE - page_offset;
    uint32_t chunk = (remaining < space_in_page) ? remaining : space_in_page;

    if (chunk > EXT_FLASH_PAGE_SIZE)
    {
      chunk = EXT_FLASH_PAGE_SIZE;
    }

    status = Flash_PageProgram(current_address, current_data, (uint32_t)chunk);
    current_address += chunk;
    current_data += chunk;
    remaining -= chunk;
  }

  return status;
}

static HAL_StatusTypeDef Flash_PageProgram(uint32_t address, const uint8_t *data, uint32_t length)
{
  if ((length == 0U) || (length > EXT_FLASH_PAGE_SIZE))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = Flash_WriteEnable();
  if (status != HAL_OK)
  {
    return status;
  }

  uint8_t cmd[4] = {
      FLASH_CMD_PAGE_PROGRAM,
      (uint8_t)(address >> 16U),
      (uint8_t)(address >> 8U),
      (uint8_t)(address)};

  Flash_Select();
  status = SPI1_Transmit(cmd, (uint16_t)sizeof(cmd), HAL_MAX_DELAY);
  if (status == HAL_OK)
  {
    status = SPI1_Transmit(data, (uint16_t)length, HAL_MAX_DELAY);
  }
  Flash_Deselect();

  if (status != HAL_OK)
  {
    return status;
  }

  return Flash_WaitWhileBusy(FLASH_DEFAULT_TIMEOUT_MS);
}

static HAL_StatusTypeDef Flash_WriteEnable(void)
{
  uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
  Flash_Select();
  HAL_StatusTypeDef status = SPI1_Transmit(&cmd, 1, HAL_MAX_DELAY);
  Flash_Deselect();
  return status;
}

static uint8_t Flash_ReadStatus(void)
{
  uint8_t cmd = FLASH_CMD_READ_STATUS;
  Flash_Select();
  if (SPI1_Transmit(&cmd, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    Flash_Deselect();
    return FLASH_STATUS_BUSY_MASK;
  }
  uint8_t status = SPI1_TransferByte(0xFF);
  Flash_Deselect();
  return status;
}

static HAL_StatusTypeDef Flash_WaitWhileBusy(uint32_t timeout_ms)
{
  uint32_t tickstart = HAL_GetTick();
  while ((Flash_ReadStatus() & FLASH_STATUS_BUSY_MASK) != 0U)
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

static void Flash_Select(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void Flash_Deselect(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}
/* USER CODE END 1 */
