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


#define GET_START_ADDR(header_ptr)\
	     (((header_ptr->label) == (BOOTLOADER_SW)) ? (BL_ADDR) : (BL_APP_ADDR))


#define FLASH_WaitForLastOperation()\
                       do{while(FLASH->SR & FLASH_SR_BSY){__NOP();}}while(true)

#define RAM_BUFFER_SIZE                                              (0x10000U)

#define FLASH_WORD_SIZE                                                    (4U)

// Flash erase timeout in ms
#define FLASH_TIMEOUT_MS                                                 (500U)

#define BLINK_LED_SINGLE()\
	           do{CAM_LED_GPIO_Port->BSRR = (1U << CAM_LED_Pin_N);\
                 for (volatile uint32_t i = 0; i < (0x60000UL); i++){__NOP();}\
                 CAM_LED_GPIO_Port->BSRR = (1U << (CAM_LED_Pin_N + 16U));\
	           }while(false)

#define BLINK_LED_DOUBLE()\
	           do{CAM_LED_GPIO_Port->BSRR = (1U << CAM_LED_Pin_N);\
                 for (volatile uint32_t i = 0; i < (0x60000UL); i++){__NOP();}\
                 CAM_LED_GPIO_Port->BSRR = (1U << (CAM_LED_Pin_N + 16U));\
                 for (volatile uint32_t i = 0; i < (0x60000UL); i++){__NOP();}\
				 CAM_LED_GPIO_Port->BSRR = (1U << CAM_LED_Pin_N);\
				 for (volatile uint32_t i = 0; i < (0x60000UL); i++){__NOP();}\
				 CAM_LED_GPIO_Port->BSRR = (1U << (CAM_LED_Pin_N + 16U));\
	           }while(false)
/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern CRC_HandleTypeDef hcrc;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static uint32_t crc_accumulator = 0xFFFFFFFF;       // Initial CRC value (seed)
static uint8_t RAM_buffer[RAM_BUFFER_SIZE];         //64K RAM buffer

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void bl_JumpToApp(void);

static HAL_StatusTypeDef bl_GetMetadata_BIN(const char* filename, Metadata_t* metadata, Header_t* header, uint32_t* bin_size);
static HAL_StatusTypeDef bl_GetMetadata_FLASH(Metadata_t* metadata, Binary_t bin_type);

static HAL_StatusTypeDef bl_Verify_BIN(const char* filename);
static HAL_StatusTypeDef bl_Verify_FLASH(void);

static HAL_StatusTypeDef bl_FindBinary(char** filename);

static HAL_StatusTypeDef bl_Write_AppBIN_to_FLASH(const char* filename, uint32_t start_addr, uint32_t bin_size);
static HAL_StatusTypeDef bl_Copy_BIN_to_RAM(const char* filename, uint32_t start_addr, uint32_t bin_size);

/* Write data chunk to FLASH callback */
static HAL_StatusTypeDef bl_WriteChunk_cbk(uint8_t *data, uint32_t length, uint32_t start_address);
/* CRC calculation for data chunk callback */
static HAL_StatusTypeDef bl_CRC_calc_cbk(uint8_t *data, uint32_t length, uint32_t current_address);

/* RAM functions: */
static __attribute__((section(".RamFunc"), used)) HAL_StatusTypeDef bl_SW_Update(const char* filename);
static __attribute__((section(".RamFunc"), used)) HAL_StatusTypeDef bl_Erase_FLASH(uint32_t start_addr, uint32_t bin_size);
static __attribute__((section(".RamFunc"), used)) HAL_StatusTypeDef bl_Write_BlBIN_to_FLASH(uint32_t start_addr, uint32_t bin_size);
/* Independent RAM version of NVIC_SystemReset() */
static __attribute__((section(".RamFunc"), used)) __NO_RETURN void bl_SystemReset_RAM(void);

/* Error Handler */
static void bl_ErrorTrap(void);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

void BL_Main(void)
{
	char *file_name; // binary file name

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

		/* Find the most suitable binary file on the SD card to update the firmware */
		if (HAL_OK != bl_FindBinary(&file_name))
		{
			break;
		}

		/* Check for binary on SD Card and verify */
		if (HAL_OK != bl_Verify_BIN(file_name))
		{
			break;
		}

		/* Perform SW Update */
		if (HAL_OK != bl_SW_Update(file_name))
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


static HAL_StatusTypeDef bl_GetMetadata_BIN(const char* filename, Metadata_t* metadata, Header_t* header, uint32_t* bin_size)
{
	HAL_StatusTypeDef retVal = HAL_ERROR;
	Metadata_t metadata_bin;

	do
	{
		// Read Application Header from Binary
		if (HAL_OK != SD_Card_Read_Header(filename, header, bin_size))
		{
			retVal = HAL_ERROR;
			break;
		}

		// Validate the metadata address
		if (header->metadata_addr < GET_START_ADDR(header) || header->metadata_addr >= (*bin_size + GET_START_ADDR(header)))
		{
			//Invalid metadata address
			retVal = HAL_ERROR;
			break;
		}

		// Read Metadata by passing App Header
		if (HAL_OK != SD_Card_Read_Metadata(filename, header, &metadata_bin, GET_START_ADDR(header)))
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


static HAL_StatusTypeDef bl_GetMetadata_FLASH(Metadata_t* metadata, Binary_t bin_type)
{
	HAL_StatusTypeDef retVal;
	Header_t *AppHeaderPtr = NULL;

	if (bin_type == BOOTLOADER_SW)
	{
		AppHeaderPtr = (Header_t*) BL_HEADER_ADDR;
		retVal = HAL_OK;
	}
	else if (bin_type == APPLICATION_SW)
	{
		AppHeaderPtr = (Header_t*) BL_APP_HEADER_ADDR;
		retVal = HAL_OK;
	}
	else
	{
		retVal = HAL_ERROR;
	}

	if (AppHeaderPtr != NULL)
	{
		metadata->crc_value = ((Metadata_t* )(AppHeaderPtr->metadata_addr))->crc_value;
		metadata->version = ((Metadata_t* )(AppHeaderPtr->metadata_addr))->version;
	}

	return retVal;
}

static HAL_StatusTypeDef bl_FindBinary(char** filename)
{
	const char* fn = NULL;
	Metadata_t metadata_bin, metadata_flash;
	Header_t header;
	uint32_t bin_size;
	HAL_StatusTypeDef retVal;

	/* Scan SD Card for the latest Bootloader SW binary file */
	fn = SD_Card_ScanAndSelectFile(BOOTLOADER_SW, BL_ADDR);

	/* If a bootloader binary is found, check if its version is more recent than the current one */
	if (fn != NULL)
	{
		if (HAL_OK != bl_GetMetadata_BIN(fn, &metadata_bin, &header, &bin_size))
		{
			retVal = HAL_ERROR;
			fn = NULL;
		}
		if (HAL_OK != bl_GetMetadata_FLASH(&metadata_flash, BOOTLOADER_SW))
		{
			// Bootloader code is corrupted - needs to be updated ASAP
			*filename = (char*)fn;
			retVal = HAL_OK;
		}
		else if (metadata_bin.version > metadata_flash.version && retVal != HAL_ERROR)
		{
			*filename = (char*)fn;
			retVal = HAL_OK;
		}
		else
		{
			retVal = HAL_ERROR;
			fn = NULL;
		}
	}

	if (retVal == HAL_ERROR)
	{
		/* Scan SD Card for the latest Application SW binary file */
		fn = SD_Card_ScanAndSelectFile(APPLICATION_SW, BL_APP_ADDR);

		/* If not, check the version of the Application binary file to see if it is more recent than the current one */
		if (fn != NULL)
		{
			if (HAL_OK != bl_GetMetadata_BIN(fn, &metadata_bin, &header, &bin_size))
			{
				retVal = HAL_ERROR;
				fn = NULL;
			}
			if (HAL_OK != bl_GetMetadata_FLASH(&metadata_flash, APPLICATION_SW))
			{
				// Application code is corrupted - needs to be updated ASAP
				*filename = (char*) fn;
				retVal = HAL_OK;
			}
			else if (metadata_bin.version > metadata_flash.version)
			{
				*filename = (char*)fn;
				retVal = HAL_OK;
			}
			else
			{
				retVal = HAL_ERROR;
				fn = NULL;
			}
		}
		else
		{
			retVal = HAL_ERROR;
		}
	}

	return retVal;
}


static HAL_StatusTypeDef bl_Verify_FLASH(void)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	Metadata_t metadata;
	uint32_t crc;
	/* Read Application Metadata from FLASH */
	retVal |= bl_GetMetadata_FLASH(&metadata, APPLICATION_SW);
	/* Calculate CRC32 for FLASH APP */
	uint32_t* app_data = (uint32_t*)BL_APP_ADDR;
	uint32_t app_len = ((Header_t* )BL_APP_HEADER_ADDR)->metadata_addr - BL_APP_ADDR;
	crc = HAL_CRC_Calculate(&hcrc, app_data, (app_len / sizeof(uint32_t)));
	/* Check CRC32 */
	retVal |= (crc == metadata.crc_value) ? HAL_OK : HAL_ERROR;
	return retVal;
}


static HAL_StatusTypeDef bl_Verify_BIN(const char* filename)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	Metadata_t metadata_bin, metadata_flash;
	Header_t header_bin;
	uint32_t data_offset;
	uint32_t bin_size;

	/* Read Application Header and Metadata from Binary */
	retVal |= bl_GetMetadata_BIN(filename, &metadata_bin, &header_bin, &bin_size);
	/* Read Application Metadata from FLASH */
	retVal |= bl_GetMetadata_FLASH(&metadata_flash, header_bin.label);

	/* Compare SW Version */
	if (metadata_bin.version > metadata_flash.version)
	{
		data_offset = header_bin.metadata_addr - GET_START_ADDR(((Header_t*)&header_bin));
		/* Calculate CRC32 for binary */
		__HAL_CRC_DR_RESET(&hcrc);
		retVal |= SD_Card_Read_Binary(filename, GET_START_ADDR(((Header_t*)&header_bin)), data_offset, RAM_buffer, RAM_BUFFER_SIZE, bl_CRC_calc_cbk);
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


static __attribute__((section(".RamFunc"), used)) HAL_StatusTypeDef bl_Erase_FLASH(uint32_t start_addr, uint32_t bin_size)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t last_sector;
	uint32_t end_addr;
	uint32_t timeout;
	//uint32_t sector_error;

	/* Check if the file size exceeds the size of the available FLASH */
	if (bin_size > (FLASH_BASE + BL_FLASH_SIZE - start_addr))
	{
		retVal = HAL_ERROR;
	}
	else
	{
		/* If it does not exceed, erase necessary sectors used for writing this file */

	    // Calculate the end address
	    end_addr = start_addr + bin_size - 1UL;

	    // Unlock flash
	    //HAL_FLASH_Unlock();
	    if (FLASH->CR & FLASH_CR_LOCK)
		{
			FLASH->KEYR = FLASH_KEY1;
			FLASH->KEYR = FLASH_KEY2;
		}

	    // Initialize erase parameters
	    //erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
	    //erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V to 3.6V

	    // Identify the first and last sectors
	    erase_init.Sector = GET_SECTOR_NUMBER(start_addr);
	    last_sector = GET_SECTOR_NUMBER(end_addr);

	    // Loop through sectors and erase each one
	    while (erase_init.Sector <= last_sector)
	    {
	        erase_init.NbSectors = 1; // Erase one sector at a time

	        // Perform the erase operation
//	        if (HAL_OK != HAL_FLASHEx_Erase(&erase_init, &sector_error))
//	        {
//	            //Error erasing sector
//	        	retVal = HAL_ERROR;
//	            break;
//	        }

			// Clear program size
			CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
			// Set program size
			SET_BIT(FLASH->CR, FLASH_CR_PSIZE_1);
			// Clear current SNB
			CLEAR_BIT(FLASH->CR, FLASH_CR_SNB);
			// Activate Sector Erase (not MASS Erase)
			SET_BIT(FLASH->CR, FLASH_CR_SER);
			// Deactivate Mass Erase
			CLEAR_BIT(FLASH->CR, FLASH_CR_MER);
			// Set the sector
			SET_BIT(FLASH->CR, (erase_init.Sector << FLASH_CR_SNB_Pos));
			// Start operation
			SET_BIT(FLASH->CR, FLASH_CR_STRT);

			// Wait for operation to complete
			timeout = FLASH_TIMEOUT_MS * (SystemCoreClock / 1000UL);

			while (FLASH->SR & FLASH_SR_BSY)
			{
				if (timeout-- == 0U)
				{
					retVal = HAL_ERROR;
					break;
				}
			}

			if (retVal != HAL_OK)
			{
				break;
			}

			// Check error flags
			if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR))
			{
				retVal = HAL_ERROR;
				break;
			}


	        // Move to the next sector
	        erase_init.Sector++;
	    }

	    // Lock flash after erase
	    //HAL_FLASH_Lock();
	    FLASH->CR |= FLASH_CR_LOCK;
	}

	return retVal;
}

static HAL_StatusTypeDef bl_Write_AppBIN_to_FLASH(const char* filename, uint32_t start_addr, uint32_t bin_size)
{
	HAL_StatusTypeDef retVal;

	// Flash the binary from the SD card
	retVal = SD_Card_Read_Binary(filename, start_addr, bin_size, RAM_buffer, RAM_BUFFER_SIZE, bl_WriteChunk_cbk);
	// Flash memory will be updated under bl_WriteChunk_cbk() callback

	return retVal;
}

static __attribute__((section(".RamFunc"), used)) HAL_StatusTypeDef bl_Write_BlBIN_to_FLASH(uint32_t start_addr, uint32_t bin_size)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	uint32_t addr = start_addr;
	uint32_t remaining = bin_size;
	uint32_t *buffer = (uint32_t*) RAM_buffer;
	uint32_t timeout;

    // Check input parameters
    if (start_addr < FLASH_BASE || (start_addr + bin_size) > FLASH_END)
    {
    	retVal = HAL_ERROR; // wrong address range
    }
    else
    {
		// Unlock flash memory for writing
		if (FLASH->CR & FLASH_CR_LOCK)
		{
			FLASH->KEYR = FLASH_KEY1;
			FLASH->KEYR = FLASH_KEY2;
		}

		// Write data to the FLASH
		while (remaining >= FLASH_WORD_SIZE)
		{
			// Make sure FLASH is ready for writing
			while (FLASH->SR & FLASH_SR_BSY);

			// Clear error flags
			FLASH->SR |= (FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR | FLASH_SR_WRPERR);

			// Set program size (32-bit word)
			FLASH->CR &= ~FLASH_CR_PSIZE;
			FLASH->CR |= (FLASH_VOLTAGE_RANGE_3 << FLASH_CR_PSIZE_Pos);

			// Activate programming mode
			FLASH->CR |= FLASH_CR_PG;

			// Write 32-bit word
			*((volatile uint32_t*) addr) = *buffer;

			// Wait for the operation to complete
			timeout = FLASH_TIMEOUT_MS * (SystemCoreClock / 1000U);
			while (FLASH->SR & FLASH_SR_BSY)
			{
				if (timeout-- == 0)
				{
					retVal = HAL_ERROR;        // Timeout
					break;
				}
			}

			// Check error flags
			if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR | FLASH_SR_WRPERR))
			{
				retVal = HAL_ERROR;
				break;
			}

			// Update address and buffer
			addr += FLASH_WORD_SIZE;
			buffer++;
			remaining -= FLASH_WORD_SIZE;
		}

		// Lock flash after erase
		//HAL_FLASH_Lock();
		FLASH->CR |= FLASH_CR_LOCK;
	}

    // Success
	return retVal;
}

/* CRC calculation for data chunk callback */
static HAL_StatusTypeDef bl_CRC_calc_cbk(uint8_t *data, uint32_t length, uint32_t current_address)
{
    // Accumulate the CRC for the current chunk
    crc_accumulator = HAL_CRC_Accumulate(&hcrc, (uint32_t *)data, length / BL_WORD_SIZE);
    return HAL_OK;
}

static HAL_StatusTypeDef bl_Copy_BIN_to_RAM(const char* filename, uint32_t start_addr, uint32_t bin_size)
{
	HAL_StatusTypeDef retVal;

	// Copy the binary from the SD card to RAM buffer
	retVal = SD_Card_Read_Binary(filename, start_addr, bin_size, RAM_buffer, RAM_BUFFER_SIZE, NULL);
	// Flash memory will be updated under bl_WriteChunk_cbk() callback

	return retVal;
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


static __attribute__((section(".RamFunc"), used)) HAL_StatusTypeDef bl_SW_Update(const char* filename)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	uint32_t start_addr;
	Metadata_t metadata;
	Header_t header;
	uint32_t bin_size;

	if (HAL_OK != bl_GetMetadata_BIN(filename, &metadata, &header, &bin_size))
	{
		retVal = HAL_ERROR;
	}
	else
	{
		switch (header.label)
		{
			case BOOTLOADER_SW:
			{
				start_addr = BL_ADDR;
				/* Copy BL binary to RAM buffer */
				retVal = bl_Copy_BIN_to_RAM(filename, start_addr, bin_size);
				__disable_irq();
				/* Erate the FLASH */
				retVal = bl_Erase_FLASH(start_addr, bin_size);
				/* Write binary to FLASH */
				retVal = bl_Write_BlBIN_to_FLASH(start_addr, bin_size);
				/* Signal the user with a double LED blink that the bootloader update is complete */
				BLINK_LED_DOUBLE();
				break;
			}

			case APPLICATION_SW:
			{
				start_addr = BL_APP_ADDR;

				/* Erate the FLASH */
				retVal = bl_Erase_FLASH(start_addr, bin_size);

				if (HAL_OK == retVal)
				{
					/* Write binary to FLASH  */
					retVal = bl_Write_AppBIN_to_FLASH(filename, start_addr, bin_size);
					/* Signal the user with a single LED blink that the application update is complete */
					BLINK_LED_SINGLE();
				}
				break;
			}
			default:
			{
				retVal = HAL_ERROR;
				break;
			}
		}

		/* Update is done - reset the uC */
		bl_SystemReset_RAM();
	}

	return retVal;
}


static __attribute__((section(".RamFunc"), used)) __NO_RETURN void bl_SystemReset_RAM(void)
{
    // Force clearing of all pending write operations
    __DSB();

    // Setting bits in AIRCR to initiate a system reset
    SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)|(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk)|SCB_AIRCR_SYSRESETREQ_Msk);

    __DSB(); // Ensure that all memory operations are completed

    // Infinite loop waiting for reset
    for(;;)
    {
        __NOP();
    }
}


static void bl_ErrorTrap(void)
{
	while (true)
	{
		HAL_GPIO_TogglePin(CAM_LED_GPIO_Port, CAM_LED_Pin);
		HAL_Delay(500);
	}
}

