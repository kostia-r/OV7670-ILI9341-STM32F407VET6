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

typedef BL_Status_t (*SWChunkCallback)(uint8_t *data, uint32_t length, uint32_t current_address);

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern BL_Status_t SD_Card_Init(void);
extern BL_Status_t SD_Card_Read_Header(const char* filename, Header_t* header, uint32_t* bin_size);
extern BL_Status_t SD_Card_Read_Metadata(const char *filename, const Header_t* header, Metadata_t* metadata, uint32_t bin_offset);
extern BL_Status_t SD_Card_Read_Binary(const char *filename, uint32_t start_address, uint32_t data_len,
		                                     uint8_t* buffer, uint32_t buffer_size, SWChunkCallback callback);
extern const char *SD_Card_ScanAndSelectFile(Binary_t target_type, uint32_t start_addr);

#endif /* SD_CARD_H_ */
