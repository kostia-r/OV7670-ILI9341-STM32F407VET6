/*
 * SD_Card.c
 * SD Card handler
 *  Created on: Nov 20, 2024
 *      Author: K.Rudenko
 */

#include "SD_Card.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

extern const Diskio_drvTypeDef  SD_Driver;
extern CRC_HandleTypeDef hcrc;

//TODO: redo this
static uint32_t offset;

static bool checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver);
void SD_Test(void);

HAL_StatusTypeDef SD_Card_Init(void)
{
	return (checkAndInitSD(&SDFatFS, SDPath, &SD_Driver)) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef SD_Card_Read_Metadata(const char *filename, AppMetadata* metadata)
{
	HAL_StatusTypeDef retVal = HAL_OK;
    FIL file;
    UINT bytesRead;
    AppHeader app_header;
    uint32_t data_offset = BL_APP_VT_SIZE;

    do
    {
        if (!checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
        	retVal = HAL_ERROR;
            break;
        }

        if (f_open(&file, filename, FA_READ) != FR_OK)
        {
        	retVal = HAL_ERROR;
            break;
        }

        // Read the application header to find the metadata address
		if (f_lseek(&file, data_offset) != FR_OK)
		{
			retVal = HAL_ERROR;
			break;
		}

		// Read the .app_header section from the binary
		if (f_read(&file, &app_header, BL_APP_HEADER_SIZE, &bytesRead) != FR_OK || bytesRead != BL_APP_HEADER_SIZE)
		{
			retVal = HAL_ERROR;
			break;
		}

        // Validate the metadata address
        if (app_header.metadata_addr < BL_APP_ADDR || app_header.metadata_addr >= (f_size(&file) + BL_APP_ADDR))
        {
            //Invalid metadata address
        	retVal = HAL_ERROR;
        	break;
        }

        // Calculate metadata offset in the binary file
        data_offset = app_header.metadata_addr - BL_APP_ADDR;
        // TODO: remove the following line
        offset = data_offset;

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

uint8_t buffer[256];  // Temporary buffer for file reading

#include "crc.h"

HAL_StatusTypeDef SD_Card_CheckCRC(const char* filename, const AppHeader* header, uint32_t* crc)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	FIL file;
	UINT bytesRead;
	uint32_t data_offset;
	//data_offset = header->metadata_addr - BL_APP_ADDR;
	// TODO: redo
	data_offset = offset;

	do
	{
		if (!checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
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


static bool checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver)
{
    bool retVal = true;
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
            retVal = false;
        }
    }

    return retVal;
}
