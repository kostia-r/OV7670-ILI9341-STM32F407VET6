/*
 * SD_Card.c
 * SD Card handler
 *  Created on: Nov 20, 2024
 *      Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "SD_Card.h"
#include "fatfs.h"
#include "crc.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// TODO: optimize all repeating pieces of code!!

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern const Diskio_drvTypeDef  SD_Driver;
extern CRC_HandleTypeDef hcrc;

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static HAL_StatusTypeDef checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

HAL_StatusTypeDef SD_Card_Init(void)
{
	return (HAL_OK == checkAndInitSD(&SDFatFS, SDPath, &SD_Driver)) ? HAL_OK : HAL_ERROR;
}


HAL_StatusTypeDef SD_Card_Read_AppHeader(const char* filename, AppHeader* header, uint32_t* bin_size)
{
	HAL_StatusTypeDef retVal = HAL_OK;
    FIL file;
    UINT bytesRead;
    uint32_t data_offset = BL_APP_VT_SIZE;

    do
    {
        if (HAL_OK != checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
        	retVal = HAL_ERROR;
            break;
        }

        if (f_open(&file, filename, FA_READ) != FR_OK)
        {
        	retVal = HAL_ERROR;
            break;
        }

        // get file size
        *bin_size = f_size(&file);

        // Read the application header to find the metadata address
		if (f_lseek(&file, data_offset) != FR_OK)
		{
			retVal = HAL_ERROR;
			break;
		}

		// Read the .app_header section from the binary
		if (f_read(&file, header, BL_APP_HEADER_SIZE, &bytesRead) != FR_OK || bytesRead != BL_APP_HEADER_SIZE)
		{
			retVal = HAL_ERROR;
			break;
		}

        retVal = HAL_OK;
    }
    while (false);

    f_close(&file);

    return retVal;
}


HAL_StatusTypeDef SD_Card_Read_Metadata(const char *filename, const AppHeader* header, AppMetadata* metadata, uint32_t bin_offset)
{
	HAL_StatusTypeDef retVal = HAL_OK;
    FIL file;
    UINT bytesRead;
    uint32_t data_offset;

    do
    {
        if (HAL_OK != checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
        	retVal = HAL_ERROR;
            break;
        }

        if (f_open(&file, filename, FA_READ) != FR_OK)
        {
        	retVal = HAL_ERROR;
            break;
        }

        // Calculate metadata offset in the binary file
        data_offset = header->metadata_addr - bin_offset;

        // Read metadata from binary
		if (f_lseek(&file, data_offset) != FR_OK)
		{
			retVal = HAL_ERROR;
			break;
		}

		// Read the .app_metadata section from the binary
		if (f_read(&file, metadata, BL_APP_METADATA_SIZE, &bytesRead) != FR_OK || bytesRead != BL_APP_METADATA_SIZE)
		{
			retVal = HAL_ERROR;
			break;
		}

        retVal = HAL_OK;
    }
    while (false);

    f_close(&file);

    return retVal;
}

// TODO: move HAL CRC calculation to Bootloader component
HAL_StatusTypeDef SD_Card_CalcCRC(const char* filename, const AppHeader* header, uint32_t* crc, uint32_t bin_offset)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	FIL file;
	UINT bytesRead;
	uint32_t data_offset = header->metadata_addr - bin_offset;
	static uint8_t buffer[256];  // Temporary buffer for file reading

	do
	{
		if (HAL_OK != checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
		{
			// Error
			retVal = HAL_ERROR;
			break;
		}

		if (f_open(&file, filename, FA_READ) != FR_OK)
		{
			// Error
			retVal = HAL_ERROR;
			break;
		}

		// Start from the beginning of the file
		if (f_lseek(&file, 0) != FR_OK)
		{
			// Error
			retVal = HAL_ERROR;
			break;
		}

		__HAL_CRC_DR_RESET(&hcrc);

		while (f_read(&file, buffer, sizeof(buffer), &bytesRead) == FR_OK && bytesRead > 0)
		{
		    // Check if the current chunk includes part of the metadata
		    uint32_t current_position = file.fptr - bytesRead;

		    // If this chunk extends beyond the metadata offset
		    if (current_position + bytesRead > data_offset)
		    {
		        // Calculate the number of bytes before the metadata starts
		        uint32_t valid_bytes = data_offset - current_position;

		        // Process only the valid portion of the chunk
		        if (valid_bytes > 0)
		        {
		        	*crc = HAL_CRC_Accumulate(&hcrc, (uint32_t *)buffer, valid_bytes / 4);
		        }

		        // Break the loop as metadata and any remaining file data are not considered
		        break;
		    }

		    // If the entire chunk is valid (before metadata), include it in the CRC calculation
		    *crc = HAL_CRC_Accumulate(&hcrc, (uint32_t *)buffer, bytesRead / 4);
		}

		retVal = HAL_OK;
	}
	while (false);

	f_close(&file);

	return retVal;
}

HAL_StatusTypeDef SD_Card_ReadFlashBIN(const char *filename, uint32_t start_address, FlashChunkCallback callback)
{
	HAL_StatusTypeDef retVal = HAL_OK;
    FIL file;                 // FatFS file object
    UINT read_len;            // Number of bytes read
    uint8_t buffer[256];      // Buffer for reading file data
    FRESULT res;              // FatFS result
    uint32_t current_address = start_address;

    // Open the binary file
    res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("Error opening file: %d\n", res);
        return HAL_ERROR;
    }

    // Read the binary file in chunks
    while ((res = f_read(&file, buffer, sizeof(buffer), &read_len)) == FR_OK && read_len > 0)
    {
        // Call the callback function to process the chunk
        callback(buffer, read_len, current_address);

        // Increment the flash address
        current_address += read_len;
    }

    if (res != FR_OK)
    {
        //Error reading file
        f_close(&file);
        return HAL_ERROR;
    }

    // Close the file
    f_close(&file);

    return retVal; // Success
}

/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/

static HAL_StatusTypeDef checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver)
{
	HAL_StatusTypeDef retVal = HAL_OK;
    DWORD free_clusters;

    if (f_getfree(SDPath, &free_clusters, &fs) != FR_OK)
    {
        /* Unmount drive */
        f_mount(NULL, (TCHAR const*) SDPath, 1);
        FATFS_UnLinkDriver((TCHAR*) SDPath);
        FATFS_LinkDriver(SD_Driver, (TCHAR*) SDPath);
        /* Mount drive */
        if (FR_OK != f_mount(fs, (TCHAR const*) SDPath, 1))
        {
            retVal = HAL_ERROR;
        }
    }

    return retVal;
}
