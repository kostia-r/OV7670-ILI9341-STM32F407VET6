/*
 * SD_Card.h
 * SD Card handler
 *  Created on: Nov 20, 2024
 *      Author: K.Rudenko
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include "Bootloader.h"
#include "main.h"

extern HAL_StatusTypeDef SD_Card_Init(void);
extern HAL_StatusTypeDef SD_Card_Read_Metadata(const char *filename, AppMetadata* metadata);
extern HAL_StatusTypeDef SD_Card_CheckCRC(const char *filename, const AppHeader* header, uint32_t* crc);

#endif /* SD_CARD_H_ */
