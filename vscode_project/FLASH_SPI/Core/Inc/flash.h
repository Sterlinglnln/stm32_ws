/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    flash.h
  * @brief   External flash access interfaces
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H__
#define __FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Exported constants */
#define EXT_FLASH_PAGE_SIZE            256U
#define EXT_FLASH_SECTOR_SIZE          4096U
/* USER CODE END Exported constants */

/* USER CODE BEGIN Exported functions */
void Flash_Init(void);
HAL_StatusTypeDef Flash_Read(uint32_t address, uint8_t *buffer, uint32_t length);
HAL_StatusTypeDef Flash_Write(uint32_t address, const uint8_t *data, uint32_t length);
HAL_StatusTypeDef Flash_EraseSector(uint32_t address);
/* USER CODE END Exported functions */

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_H__ */
