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
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define GET_SECTOR_NUMBER(address) (\
    ((address) < 0x08004000UL) ? FLASH_SECTOR_0 : \
    ((address) < 0x08008000UL) ? FLASH_SECTOR_1 : \
    ((address) < 0x0800C000UL) ? FLASH_SECTOR_2 : \
    ((address) < 0x08010000UL) ? FLASH_SECTOR_3 : \
    ((address) < 0x08020000UL) ? FLASH_SECTOR_4 : \
    ((address) < 0x08040000UL) ? FLASH_SECTOR_5 : \
    ((address) < 0x08060000UL) ? FLASH_SECTOR_6 : \
    ((address) < 0x08080000UL) ? FLASH_SECTOR_7 : \
    ((address) < 0x080A0000UL) ? FLASH_SECTOR_8 : \
    ((address) < 0x080C0000UL) ? FLASH_SECTOR_9 : \
    ((address) < 0x080E0000UL) ? FLASH_SECTOR_10 : \
    FLASH_SECTOR_11)


/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern CRC_HandleTypeDef hcrc;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static const char* app_file_name = "F407VET6_OV7670_ILI9341_HAL.bin";
static uint32_t crc_accumulator = 0xFFFFFFFF; // Initial CRC value (seed)

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void bl_JumpToApp(void);
static HAL_StatusTypeDef bl_GetAppMetaData_BIN(AppMetadata* metadata, AppHeader* header, uint32_t* bin_size);
static HAL_StatusTypeDef bl_GetAppMetaData_FLASH(AppMetadata* metadata);
static HAL_StatusTypeDef bl_Verify_FLASH(void);
static HAL_StatusTypeDef bl_Verify_BIN(const char* filename, uint32_t* bin_size);
static HAL_StatusTypeDef bl_Erase_FLASH(uint32_t start_addr, uint32_t bin_size);
static HAL_StatusTypeDef bl_Write_BIN_to_FLASH(const char* filename, uint32_t start_addr, uint32_t bin_size);
/* Write data chunk to FLASH callback */
static HAL_StatusTypeDef bl_WriteChunk_cbk(uint8_t *data, uint32_t length, uint32_t start_address);
/* CRC calculation for data chunk callback */
static HAL_StatusTypeDef bl_CRC_calc_cbk(uint8_t *data, uint32_t length, uint32_t current_address);
/* Error Handler */
static void bl_ErrorTrap(void);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

void BL_Main(void)
{
	uint32_t bin_size;

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
		if (HAL_OK != bl_Verify_BIN(app_file_name, &bin_size))
		{
			break;
		}

		/* Erase the FLASH */
		if (HAL_OK == bl_Erase_FLASH(BL_APP_ADDR, bin_size))
		{
			/* Copy binary from SD Card to FLASH */
			if (HAL_OK != bl_Write_BIN_to_FLASH(app_file_name, BL_APP_ADDR, bin_size))
			{
				// There is a hard error
				bl_ErrorTrap();
			}

			/* Update is done - reset the uC */
			NVIC_SystemReset();
		}
		else
		{
			// There is a hard error
			bl_ErrorTrap();
		}
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
	// Configure the MSP by reading the value from the start of APP Vector Table
	__set_MSP(*(volatile uint32_t*) BL_APP_ADDR);
	// Fetch the Reset Handler address of the user app
	uint32_t reset_handler_address = *(volatile uint32_t*) (BL_APP_ADDR + BL_WORD_SIZE);
	App_Reset_Handler = (void*) reset_handler_address;
	// Jump to Reset Handler of the User Application
	App_Reset_Handler();
}


static HAL_StatusTypeDef bl_GetAppMetaData_BIN(AppMetadata* metadata, AppHeader* header, uint32_t* bin_size)
{
	HAL_StatusTypeDef retVal = HAL_ERROR;
	AppMetadata metadata_bin;

	do
	{
		// Read Application Header from Binary
		if (HAL_OK != SD_Card_Read_AppHeader(app_file_name, header, bin_size))
		{
			retVal = HAL_ERROR;
			break;
		}

		// Validate the metadata address
		if (header->metadata_addr < BL_APP_ADDR || header->metadata_addr >= (*bin_size + BL_APP_ADDR))
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


static HAL_StatusTypeDef bl_Verify_BIN(const char* filename, uint32_t* bin_size)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	AppMetadata metadata_bin, matadata_flash;
	AppHeader header_bin;
	uint32_t data_offset;

	/* Read Application Header and Metadata from Binary */
	retVal |= bl_GetAppMetaData_BIN(&metadata_bin, &header_bin, bin_size);
	/* Read Application Metadata from FLASH */
	retVal |= bl_GetAppMetaData_FLASH(&matadata_flash);

	/* Compare SW Version */
	if (metadata_bin.version > matadata_flash.version)
	{
		/* Calculate CRC32 for binary */
		data_offset = header_bin.metadata_addr - BL_APP_ADDR;

		__HAL_CRC_DR_RESET(&hcrc);
		retVal |= SD_Card_Read_Binary(filename, BL_APP_ADDR, data_offset, bl_CRC_calc_cbk);
		// crc_accumulator will be updated under bl_CRC_calc_cbk() callback

		/* Check CRC32 */
		retVal |= (crc_accumulator == metadata_bin.crc_value) ? HAL_OK : HAL_ERROR;
	}
	else
	{
		// The SW in the Binary is older than what already exists on Flash
		retVal |= HAL_ERROR;
	}

	return retVal;
}

static HAL_StatusTypeDef bl_Erase_FLASH(uint32_t start_addr, uint32_t bin_size)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t sector_error;

	/* Check if the file size exceeds the size of the available FLASH */
	if (bin_size > (FLASH_BASE + BL_FLASH_SIZE - start_addr))
	{
		retVal = HAL_ERROR;
	}
	else
	{
		/* If it does not exceed, erase necessary sectors used for writing this file */

	    // Calculate the end address
	    uint32_t end_addr = start_addr + bin_size - 1UL;

	    // Unlock flash
	    HAL_FLASH_Unlock();

	    // Initialize erase parameters
	    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
	    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V to 3.6V

	    // Identify the first and last sectors
	    erase_init.Sector = GET_SECTOR_NUMBER(start_addr);
	    uint32_t last_sector = GET_SECTOR_NUMBER(end_addr);

	    // Loop through sectors and erase each one
	    while (erase_init.Sector <= last_sector)
	    {
	        erase_init.NbSectors = 1; // Erase one sector at a time

	        // Perform the erase operation
	        if (HAL_OK != HAL_FLASHEx_Erase(&erase_init, &sector_error))
	        {
	            //Error erasing sector
	        	retVal = HAL_ERROR;
	            break;
	        }

	        // Move to the next sector
	        erase_init.Sector++;
	    }

	    // Lock flash after erase
	    HAL_FLASH_Lock();
	}

	return retVal;
}

static HAL_StatusTypeDef bl_Write_BIN_to_FLASH(const char* filename, uint32_t start_addr, uint32_t bin_size)
{
	HAL_StatusTypeDef retVal;

	// Flash the binary from the SD card
	retVal = SD_Card_Read_Binary(filename, start_addr, bin_size, bl_WriteChunk_cbk);
	// Flash memory will be updated under bl_WriteChunk_cbk() callback

	return retVal;
}

/* CRC calculation for data chunk callback */
static HAL_StatusTypeDef bl_CRC_calc_cbk(uint8_t *data, uint32_t length, uint32_t current_address)
{
    // Accumulate the CRC for the current chunk
    crc_accumulator = HAL_CRC_Accumulate(&hcrc, (uint32_t *)data, length / BL_WORD_SIZE);
    return HAL_OK;
}

/* Write data chunk to FLASH callback */
static HAL_StatusTypeDef bl_WriteChunk_cbk(uint8_t *data, uint32_t length, uint32_t start_address)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	uint32_t word;

	// Ensure address alignment for flash programming
	if ((start_address % BL_WORD_SIZE) == 0U)
	{
		// Unlock flash memory for writing
		HAL_FLASH_Unlock();
		// Blink LED
		HAL_GPIO_TogglePin(CAM_LED_GPIO_Port, CAM_LED_Pin);

		// Write data in 32-bit words
		for (uint32_t i = 0; i < length; i += BL_WORD_SIZE)
		{
			word = *(uint32_t*) (data + i);

			// Write the word to flash memory
			if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_address + i, word))
			{
				//Flash programming error
				retVal = HAL_ERROR;
				break;
			}
		}

		// Lock flash memory after programming
		HAL_FLASH_Lock();
	}
	else
	{
		/* Do nothing - address not aligned to 32-bit word */
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

