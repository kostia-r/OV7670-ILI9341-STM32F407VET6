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
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern const Diskio_drvTypeDef  SD_Driver;

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static BL_Status_t checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

BL_Status_t SD_Card_Init(void)
{
	return (BL_OK == checkAndInitSD(&SDFatFS, SDPath, &SD_Driver)) ? BL_OK : BL_ERROR;
}


BL_Status_t SD_Card_Read_Header(const char* filename, Header_t* header, uint32_t* bin_size)
{
	BL_Status_t retVal = BL_OK;
    FIL file;
    UINT bytesRead;
    uint32_t data_offset = BL_APP_VT_SIZE;

    do
    {
        if (BL_OK != checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
        	retVal = BL_ERROR;
            break;
        }

        if (f_open(&file, filename, FA_READ) != FR_OK)
        {
        	retVal = BL_ERROR;
            break;
        }

        // get file size
        *bin_size = f_size(&file);

        // Read the application header to find the metadata address
		if (f_lseek(&file, data_offset) != FR_OK)
		{
			retVal = BL_ERROR;
			break;
		}

		// Read the .app_header section from the binary
		if (f_read(&file, header, BL_HEADER_SIZE, &bytesRead) != FR_OK || bytesRead != BL_HEADER_SIZE)
		{
			retVal = BL_ERROR;
			break;
		}

        retVal = BL_OK;
    }
    while (false);

    f_close(&file);

    return retVal;
}


BL_Status_t SD_Card_Read_Metadata(const char *filename, const Header_t* header, Metadata_t* metadata, uint32_t bin_offset)
{
	BL_Status_t retVal = BL_OK;
    FIL file;
    UINT bytesRead;
    uint32_t data_offset;

    do
    {
        if (BL_OK != checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
        	retVal = BL_ERROR;
            break;
        }

        if (f_open(&file, filename, FA_READ) != FR_OK)
        {
        	retVal = BL_ERROR;
            break;
        }

        // Calculate metadata offset in the binary file
        data_offset = header->metadata_addr - bin_offset;

        // Read metadata from binary
		if (f_lseek(&file, data_offset) != FR_OK)
		{
			retVal = BL_ERROR;
			break;
		}

		// Read the .app_metadata section from the binary
		if (f_read(&file, metadata, BL_METADATA_SIZE, &bytesRead) != FR_OK || bytesRead != BL_METADATA_SIZE)
		{
			retVal = BL_ERROR;
			break;
		}

        retVal = BL_OK;
    }
    while (false);

    f_close(&file);

    return retVal;
}

/* Function that sequentially reads the specified file 256 bytes at a time,
 * and calls the callback from the parameter for each chunk of read data. */
BL_Status_t SD_Card_Read_Binary(const char *filename, uint32_t start_address, uint32_t data_len, uint8_t* buffer, uint32_t buffer_size, SWChunkCallback callback)
{
	BL_Status_t retVal = BL_OK;
    FIL file;                   // FatFS file object
    UINT bytesRead;             // Number of bytes read
    uint32_t current_address = start_address;
    uint32_t total_read = 0;  // Tracks the total number of bytes read

	do
	{
		if (BL_OK != checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
		{
			// Error
			retVal = BL_ERROR;
			break;
		}

		if (f_open(&file, filename, FA_READ) != FR_OK)
		{
			// Error
			retVal = BL_ERROR;
			break;
		}

		// Read the binary file in chunks
		while (total_read < data_len)
		{
			uint32_t bytes_to_read = buffer_size;

			// Adjust the chunk size if near the data_len limit
			if (total_read + bytes_to_read > data_len)
			{
				bytes_to_read = data_len - total_read;
			}

			// Read a chunk from the file
			if (FR_OK != f_read(&file, buffer, bytes_to_read, &bytesRead) || bytesRead == 0)
			{
				//Error reading file
				retVal = BL_ERROR;
				break;
			}

			// Call the callback function to process the chunk
			if (NULL != callback)
			{
				if (BL_OK != callback(buffer, bytesRead, current_address))
				{
					retVal = BL_ERROR;
					break;
				}
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


const char *SD_Card_ScanAndSelectFile(Binary_t target_type, uint32_t start_addr)
{
    DIR dir;
    static FILINFO fno;
    static char selected_file_name[_MAX_LFN + 1];
    FIL file;
    Metadata_t metadata;
    Header_t header;
    UINT read_len;
    uint32_t data_offset;
    const char *selected_file;
    const char *ext;
    uint32_t highest_version;

    // Open root directory
    if (f_opendir(&dir, "/") != FR_OK)
    {
        return NULL; // Error opening directory
    }

    selected_file = NULL;
    highest_version = 0;

    // View files in a directory
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0')
    {
        // Skipping directories
        if (fno.fattrib & AM_DIR)
        {
            continue;
        }

        // Check file extension
        ext = strrchr(fno.fname, '.');

        if (!ext || strcmp(ext, ".bin") != 0)
        {
            continue; // Skipping files without the required extension
        }

        if (f_open(&file, fno.fname, FA_READ) != FR_OK)
        {
        	f_close(&file);
            continue;
        }

        data_offset = BL_APP_VT_SIZE;

		if (f_lseek(&file, data_offset) != FR_OK)
		{
			f_close(&file);
			continue;
		}

		if (f_read(&file, &header, BL_HEADER_SIZE, &read_len) != FR_OK || read_len != BL_HEADER_SIZE)
		{
			f_close(&file);
			continue;
		}
		else
		{
			// check binary type
			if (header.label != target_type)
			{
				f_close(&file);
				continue;
			}

			// Calculate metadata offset in the binary file
			data_offset = header.metadata_addr - start_addr;

			if (f_lseek(&file, data_offset) != FR_OK)
			{
				f_close(&file);
				continue;
			}

			if (f_read(&file, &metadata, BL_METADATA_SIZE, &read_len) != FR_OK || read_len != BL_METADATA_SIZE)
			{
				f_close(&file);
				continue;
			}
			//If the binary type matches, check the version
			if (metadata.version > highest_version)
			{
				highest_version = metadata.version;
				selected_file = (const char *)&fno.fname;
				// copy file name to the temporary buffer
				strcpy(selected_file_name, (const char *)&fno.fname);
			}
		}

        f_close(&file);
    }

    f_closedir(&dir);

    if (selected_file_name[0] != '\0')
    {
    	selected_file = (const char *)&selected_file_name;
    }

    return selected_file;
}

/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/

static BL_Status_t checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver)
{
	BL_Status_t retVal = BL_OK;
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
            retVal = BL_ERROR;
        }
    }

    return retVal;
}
