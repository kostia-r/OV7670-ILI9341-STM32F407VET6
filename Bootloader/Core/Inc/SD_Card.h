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
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern HAL_StatusTypeDef SD_Card_Init(void);
extern HAL_StatusTypeDef SD_Card_Read_AppHeader(const char* filename, AppHeader* header, uint32_t* bin_size);
extern HAL_StatusTypeDef SD_Card_Read_Metadata(const char *filename, const AppHeader* header, AppMetadata* metadata, uint32_t bin_offset);
extern HAL_StatusTypeDef SD_Card_CalcCRC(const char* filename, const AppHeader* header, uint32_t* crc, uint32_t bin_offset);

#endif /* SD_CARD_H_ */
