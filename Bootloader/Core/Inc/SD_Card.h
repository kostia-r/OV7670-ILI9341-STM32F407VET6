/*
 * SD_Card.h
 * SD Card handler
 *  Created on: Nov 20, 2024
 *      Author: K.Rudenko
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "Bootloader.h"
#include "main.h"

/******************************************************************************
 *                           GLOBAL DATA TYPES                                *
 ******************************************************************************/

typedef void (*FlashChunkCallback)(uint8_t *data, uint32_t length, uint32_t current_address);

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern HAL_StatusTypeDef SD_Card_Init(void);
extern HAL_StatusTypeDef SD_Card_Read_AppHeader(const char* filename, AppHeader* header, uint32_t* bin_size);
extern HAL_StatusTypeDef SD_Card_Read_Metadata(const char *filename, const AppHeader* header, AppMetadata* metadata, uint32_t bin_offset);
extern HAL_StatusTypeDef SD_Card_CalcCRC(const char* filename, const AppHeader* header, uint32_t* crc, uint32_t bin_offset);
extern HAL_StatusTypeDef SD_Card_ReadFlashBIN(const char *filename, uint32_t start_address, FlashChunkCallback callback);

#endif /* SD_CARD_H_ */
