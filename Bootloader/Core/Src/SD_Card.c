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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define SD_CARD_BUFFER_SIZE_BYTES                                         (512U)

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern const Diskio_drvTypeDef  SD_Driver;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static uint8_t buffer[SD_CARD_BUFFER_SIZE_BYTES]; // Buffer for reading file data

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

/* Function that sequentially reads the specified file 256 bytes at a time,
 * and calls the callback from the parameter for each chunk of read data. */
HAL_StatusTypeDef SD_Card_Read_Binary(const char *filename, uint32_t start_address, uint32_t data_len, SWChunkCallback callback)
{
	HAL_StatusTypeDef retVal = HAL_OK;
    FIL file;                   // FatFS file object
    UINT bytesRead;             // Number of bytes read
    uint32_t current_address = start_address;
    uint32_t total_read = 0;  // Tracks the total number of bytes read

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

		// Read the binary file in chunks
		while (total_read < data_len)
		{
			uint32_t bytes_to_read = sizeof(buffer);

			// Adjust the chunk size if near the data_len limit
			if (total_read + bytes_to_read > data_len)
			{
				bytes_to_read = data_len - total_read;
			}

			// Read a chunk from the file
			if (FR_OK != f_read(&file, buffer, bytes_to_read, &bytesRead) || bytesRead == 0)
			{
				//Error reading file
				retVal = HAL_ERROR;
				break;
			}

			// Call the callback function to process the chunk
			if (HAL_OK != callback(buffer, bytesRead, current_address))
			{
				retVal = HAL_ERROR;
				break;
			}

			// Update counters and addresses
			current_address += bytesRead;
			total_read += bytesRead;
		}
	}
	while (false);

	f_close(&file);

	return retVal;
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
