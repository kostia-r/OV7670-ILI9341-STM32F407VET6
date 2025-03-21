/*
 * W25Q16.h
 * W25Q16JV Flash driver
 *  Created on: Dec 4, 2024
 *      Author: ashen
 */

#ifndef W25Q16_H_
#define W25Q16_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "Bootloader.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *                             GLOBAL DATA TYPES                              *
 ******************************************************************************/

/******************************************************************************
 *                               GLOBAL MACRO                                 *
 ******************************************************************************/
#define W25_CS_PORT                                          FLASH_CS_GPIO_Port
#define W25_CS_PIN                                                 FLASH_CS_Pin
/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern BL_Status_t W25Q_Init(void);
extern BL_Status_t W25Q_ReadData(uint32_t addr, uint8_t* pBuffer, uint32_t size);
extern BL_Status_t W25Q_ReadPage(uint32_t page_numb, uint32_t offset, uint8_t* pBuffer, uint32_t size);
extern BL_Status_t W25Q_WriteData(uint32_t addr, const uint8_t* pData, uint32_t size);
extern BL_Status_t W25Q_WritePage(uint32_t page_addr, uint32_t offset, const uint8_t *pData, uint32_t size);
extern BL_Status_t W25Q_Erase(uint32_t addr, uint32_t size);
extern BL_Status_t W25Q_EraseChip(void);

#endif /* W25Q16_H_ */
