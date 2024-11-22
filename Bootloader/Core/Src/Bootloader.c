/*
 * Bootloader.c
 * Bootloader
 *  Created on: Nov 19, 2024
 *      Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "Bootloader.h"
#include "stm32f4xx_hal.h"
#include "crc.h"
#include "SD_Card.h"
#include "main.h"
#include "fatfs.h"
#include <stdbool.h>

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern CRC_HandleTypeDef hcrc;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static const char* app_file_name = "F407VET6_OV7670_ILI9341_HAL.bin";

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void bl_JumpToApp(void);
static HAL_StatusTypeDef bl_GetAppMetaData_BIN(AppMetadata* metadata, AppHeader* header);
static HAL_StatusTypeDef bl_GetAppMetaData_FLASH(AppMetadata* metadata);
static HAL_StatusTypeDef bl_Verify_FLASH(void);
static HAL_StatusTypeDef bl_Verify_BIN(const char* filename);;
static void bl_ErrorTrap(void);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

void BL_Main(void)
{
	/* Check for update */
	do
	{
		/* If two buttons are pressed - go to updater mode */
		if (HAL_GPIO_ReadPin(GPIOC, CAM_BTN1_Pin | CAM_BTN2_Pin) != GPIO_PIN_SET)
		{
			break;
		}

		/* Try to Init SD Card */
		if (HAL_OK != SD_Card_Init())
		{
			break;
		}

		/* Check for binary on SD Card and verify */
		if (HAL_OK != bl_Verify_BIN(app_file_name))
		{
			break;
		}

		/* Erase the FLASH */

		/* Copy binary from SD Card to FLASH */

		/* Reset the uC */
	}
	while(false);

	/* Proceed with existing SW on FLASH */
	if (HAL_OK != bl_Verify_FLASH())
	{
		// There is an error
		bl_ErrorTrap();

	}
	else
	{
		// Jump to the Application
		bl_JumpToApp();
	}
}

/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/

static void bl_JumpToApp(void)
{
	// pointer to APP Reset Handler
	void (*App_Reset_Handler)(void);
	// Deinitialize HAL
	HAL_DeInit();
	// Configure the MSP by reading the value from the base address of the sector 2
	__set_MSP(*(volatile uint32_t*) BL_APP_ADDR);
	// Fetch the Reset Handler address of the user app
	uint32_t reset_handler_address = *(volatile uint32_t*) (BL_APP_ADDR + sizeof(uint32_t));
	App_Reset_Handler = (void*) reset_handler_address;
	// Jump to Reset Handler of the User Application
	App_Reset_Handler();
}


static HAL_StatusTypeDef bl_GetAppMetaData_BIN(AppMetadata* metadata, AppHeader* header)
{
	HAL_StatusTypeDef retVal = HAL_ERROR;
	AppMetadata metadata_bin;
	uint32_t bin_size;

	do
	{
		// Read Application Header from Binary
		if (HAL_OK != SD_Card_Read_AppHeader(app_file_name, header, &bin_size))
		{
			retVal = HAL_ERROR;
			break;
		}

		// Validate the metadata address
		if (header->metadata_addr < BL_APP_ADDR || header->metadata_addr >= (bin_size + BL_APP_ADDR))
		{
			//Invalid metadata address
			retVal = HAL_ERROR;
			break;
		}

		// Read Metadata by passing App Header
		if (HAL_OK != SD_Card_Read_Metadata(app_file_name, header, &metadata_bin, BL_APP_ADDR))
		{
			//Invalid metadata address
			retVal = HAL_ERROR;
			break;
		}

		metadata->crc_value = metadata_bin.crc_value;
		metadata->version = metadata_bin.version;
		retVal = HAL_OK;
	}
	while (false);

	return retVal;
}


static HAL_StatusTypeDef bl_GetAppMetaData_FLASH(AppMetadata* metadata)
{
	AppHeader *AppHeaderPtr = (AppHeader*) BL_APP_HEADER_ADDR;
	metadata->crc_value = ((AppMetadata* )(AppHeaderPtr->metadata_addr))->crc_value;
	metadata->version = ((AppMetadata* )(AppHeaderPtr->metadata_addr))->version;
	return HAL_OK;
}


static HAL_StatusTypeDef bl_Verify_FLASH(void)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	AppMetadata metadata;
	uint32_t crc;
	/* Read Application Metadata from FLASH */
	retVal |= bl_GetAppMetaData_FLASH(&metadata);
	/* Calculate CRC32 for FLASH APP */
	uint32_t* app_data = (uint32_t*)BL_APP_ADDR;
	uint32_t app_len = ((AppHeader* )BL_APP_HEADER_ADDR)->metadata_addr - BL_APP_ADDR;
	crc = HAL_CRC_Calculate(&hcrc, app_data, (app_len / sizeof(uint32_t)));
	/* Check CRC32 */
	retVal |= (crc == metadata.crc_value) ? HAL_OK : HAL_ERROR;
	return retVal;
}


static HAL_StatusTypeDef bl_Verify_BIN(const char* filename)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	AppMetadata metadata_bin, matadata_flash;
	AppHeader header_bin;
	uint32_t crc;
	/* Read Application Header and Metadata from Binary */
	retVal |= bl_GetAppMetaData_BIN(&metadata_bin, &header_bin);
	/* Read Application Metadata from FLASH */
	retVal |= bl_GetAppMetaData_FLASH(&matadata_flash);

	/* Compare SW Version */
	if (metadata_bin.version > matadata_flash.version)
	{
		/* Calculate CRC32 for binary */
		retVal |= SD_Card_CalcCRC(filename, &header_bin, &crc, BL_APP_ADDR);
		/* Check CRC32 */
		retVal |= (crc == metadata_bin.crc_value) ? HAL_OK : HAL_ERROR;
	}
	else
	{
		// The SW in the Binary is older than what already exists on Flash
		retVal |= HAL_ERROR;
	}

	return retVal;
}


static void bl_ErrorTrap(void)
{
	while (true)
	{
		HAL_GPIO_TogglePin(CAM_LED_GPIO_Port, CAM_LED_Pin);
		HAL_Delay(500);
	}
}

