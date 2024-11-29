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

/* NOTE! Shall be defined if compiler optimization level is -O2 or upper! */
#define USE_ASM_JUMP_CODE

#define RAM_BUFFER_SIZE                                              (0x10000U)

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

#define FLASH_WORD_SIZE                                                    (4U)

// Flash erase timeout in ms
#define FLASH_TIMEOUT_MS                                                 (500U)

#define BL_ZERO                                                          (0x0U)
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
#ifndef USE_ASM_JUMP_CODE
static void bl_JumpToApp(void);
#else
static void __attribute__((naked)) bl_JumpToApp(void);
#endif /* !USE_ASM_JUMP_CODE */

static BL_Status_t bl_GetMetadata_BIN(const char* filename, Metadata_t* metadata, Header_t* header, uint32_t* bin_size);
static BL_Status_t bl_GetMetadata_FLASH(Metadata_t* metadata, Binary_t bin_type);

static BL_Status_t bl_Verify_BIN(const char* filename, const Metadata_t* metadata, const Header_t* header, uint32_t bin_size);
static BL_Status_t bl_Verify_FLASH(void);

static BL_Status_t bl_FindBinary(char** filename, Metadata_t* metadata, Header_t *header, uint32_t* bin_size);

static BL_Status_t bl_Write_BIN_to_FLASH(const char* filename, uint32_t start_addr, uint32_t bin_size);
static BL_Status_t bl_Copy_BIN_to_RAM(const char* filename, uint32_t start_addr, uint32_t bin_size);

/* Write data chunk to FLASH callback */
static BL_Status_t bl_WriteChunk_cbk(uint8_t *data, uint32_t length, uint32_t start_address);
/* CRC calculation for data chunk callback */
static BL_Status_t bl_CRC_calc_cbk(uint8_t *data, uint32_t length, uint32_t current_address);

/************************** RAM functions: ***********************************/

static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_SW_Update(const char* filename, const Header_t* header, uint32_t bin_size);
static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_Erase_FLASH(uint32_t start_addr, uint32_t bin_size);
static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_Write_RAM_to_FLASH(uint32_t start_addr, uint32_t bin_size);

/* STM32F407VET6 FLASH CMSIS DRIVER */
static __attribute__((section(".RamFunc"), used)) void bl_FLASH_Unlock(void);
static __attribute__((section(".RamFunc"), used)) void bl_FLASH_Lock(void);
static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_FLASH_GetErrStatus(void);
static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_FLASH_EraseSector(uint32_t sector_num);
static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_FLASH_ProgramWord(uint32_t Address, uint32_t Data);

/* RAM version of NVIC_SystemReset() */
static __attribute__((section(".RamFunc"), used)) __NO_RETURN void bl_SystemReset_RAM(void);

/* Helper RAM functions: */
static __attribute__((section(".RamFunc"), used)) void bl_ErrorHook(void);
static __attribute__((section(".RamFunc"), used)) void bl_LED_blink(uint32_t cnt);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

void BL_Main(void)
{
	char *file_name_bin; // binary file name
	Metadata_t metadata_bin = {BL_ZERO};
	Header_t header_bin = {BL_ZERO};
	uint32_t size_bin = BL_ZERO;
	bool isReadyToJump = false;

	/* Check for update */
	do
	{
		/* Proceed with existing SW on FLASH */
		if (BL_OK == bl_Verify_FLASH())
		{
			isReadyToJump = true;
			/* FLASH Application is ok, but to continue check
			 * if two buttons are pressed - go to updater mode */
			if (HAL_GPIO_ReadPin(GPIOC, CAM_BTN1_Pin | CAM_BTN2_Pin) != GPIO_PIN_SET)
			{
				break;
			}
		}

		/* Try to Init SD Card */
		if (BL_OK != SD_Card_Init())
		{
			break;
		}

		/* Find the most suitable binary file on the SD card to update the firmware */
		if (BL_OK != bl_FindBinary(&file_name_bin, &metadata_bin, &header_bin, &size_bin))
		{
			break;
		}

		/* Check for binary on SD Card and verify */
		if (BL_OK != bl_Verify_BIN(file_name_bin, &metadata_bin, &header_bin, size_bin))
		{
			break;
		}

		/* Perform SW Update */
		if (BL_OK != bl_SW_Update(file_name_bin, &header_bin, size_bin))
		{
			// There is a hard error
			bl_ErrorHook();
		}
	}
	while(false);

	if (isReadyToJump)
	{
		// Signal the user that the Bootloader passes control to the Application
		bl_LED_blink(1UL);
		// Jump to the Application
		bl_JumpToApp();
	}
}

/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/
#ifndef USE_ASM_JUMP_CODE
static void bl_JumpToApp(void)
{
	/* NOTE: SCB->VTOR shall be updated in the Application! */
	// pointer to APP Reset Handler
	volatile void (*App_Reset_Handler)(void);
	// Reset handler address
	volatile uint32_t reset_handler_address;

	// Deinitialize HAL
	HAL_DeInit();
	// Configure the MSP by reading the value from the start of APP Vector Table
	__set_MSP(*(volatile uint32_t*) __bl_appl_addr);
	// Fetch the Reset Handler address of the user app
	reset_handler_address = *(volatile uint32_t*) (__bl_appl_addr + BL_WORD_SIZE);
	// Set T-bit
	reset_handler_address |= 0x01UL;
	// Update Function pointer to jump
	App_Reset_Handler = (void*) reset_handler_address;
	/* Memory barrier to complete all operations before branching */
	__DSB();
	__ISB();
	// Jump to Reset Handler of the User Application
	App_Reset_Handler();
}
#else
/* Assembly version of the function for compilation with optimization level -O2 */
static void __attribute__((naked)) bl_JumpToApp(void)
{
	__asm volatile (
			/* NOTE: SCB->VTOR shall be updated in the Application! */
			"    push {r0, r1, r2, lr}    \n"  // Save context (registers) , acc to ABI convention
			"    ldr r0, =HAL_DeInit      \n"  // Load address of HAL_DeInit() function
			"    blx r0                   \n"  // call HAL_DeInit()
			"    pop {r0, r1, r2, lr}     \n"  // Restore context (registers)
	        "    ldr r0, =__bl_appl_addr  \n"  // Load the application start address
	        "    ldr r1, [r0]             \n"  // Read MSP init value from application vector table
	        "    msr msp, r1              \n"  // Update MSP
	        "    ldr r1, [r0, #4]         \n"  // Read Reset_Handler address from application vector table
	        "    orr r1, r1, #1           \n"  // Set T-bit (switch to Thumb mode)
	        "    bx r1                    \n"  // Jump to Application Reset Handler
	    );
}
#endif /* !USE_ASM_JUMP_CODE */


static BL_Status_t bl_GetMetadata_BIN(const char* filename, Metadata_t* metadata, Header_t* header, uint32_t* bin_size)
{
	BL_Status_t retVal = BL_ERROR;
	Metadata_t metadata_bin;

	do
	{
		// Read Application Header from Binary
		if (BL_OK != SD_Card_Read_Header(filename, header, bin_size))
		{
			retVal = BL_ERROR;
			break;
		}

		// Validate the metadata address
		if (header->metadata_addr < GET_START_ADDR(header) || header->metadata_addr >= (*bin_size + GET_START_ADDR(header)))
		{
			//Invalid metadata address
			retVal = BL_ERROR;
			break;
		}

		// Read Metadata by passing App Header
		if (BL_OK != SD_Card_Read_Metadata(filename, header, &metadata_bin, GET_START_ADDR(header)))
		{
			//Invalid metadata address
			retVal = BL_ERROR;
			break;
		}

		metadata->crc_value = metadata_bin.crc_value;
		metadata->version = metadata_bin.version;
		retVal = BL_OK;
	}
	while (false);

	return retVal;
}


static BL_Status_t bl_GetMetadata_FLASH(Metadata_t* metadata, Binary_t bin_type)
{
	BL_Status_t retVal;
	Header_t *AppHeaderPtr = NULL;
	uint32_t crc_value;
	uint32_t version;

	if (bin_type == BOOTLOADER_SW)
	{
		AppHeaderPtr = (Header_t*) BL_HEADER_ADDR;
		retVal = BL_OK;
	}
	else if (bin_type == APPLICATION_SW)
	{
		AppHeaderPtr = (Header_t*) BL_APP_HEADER_ADDR;
		retVal = BL_OK;
	}
	else
	{
		retVal = BL_ERROR;
	}

	if (AppHeaderPtr->metadata_addr != 0x00000000UL && AppHeaderPtr->metadata_addr != 0xFFFFFFFFUL)
	{
		crc_value = ((Metadata_t* )(AppHeaderPtr->metadata_addr))->crc_value;
		version = ((Metadata_t* )(AppHeaderPtr->metadata_addr))->version;

		if (version != 0xFFFFFFFFUL)
		{
			metadata->crc_value = crc_value;
			metadata->version = version;
			retVal = BL_OK;
		}
		else
		{
			retVal = BL_ERROR;
		}
	}
	else
	{
		retVal = BL_ERROR;
	}

	return retVal;
}

static BL_Status_t bl_FindBinary(char** filename, Metadata_t* metadata, Header_t *header, uint32_t* bin_size)
{
	const char* fn = NULL;
	Metadata_t  metadata_flash = {BL_ZERO};
	BL_Status_t retVal = BL_ERROR;

	do
	{
		// Check input parameter
		if (filename == NULL)
		{
			break; // Error
		}

		*filename = NULL;

		// Scan SD Card for Bootloader SW binary
		fn = SD_Card_ScanAndSelectFile(BOOTLOADER_SW, BL_ADDR);
		if (fn != NULL)
		{
			if (bl_GetMetadata_BIN(fn, metadata, header, bin_size) != BL_OK)
			{
				break; // Error reading file metadata
			}

			if (bl_GetMetadata_FLASH(&metadata_flash, BOOTLOADER_SW) != BL_OK)
			{
				// Bootloader code is corrupted - needs immediate update
				*filename = (char*) fn;
				retVal = BL_OK;
				break; // Successful completion, file for update found
			}

			if (metadata->version > metadata_flash.version)
			{
				*filename = (char*) fn;
				retVal = BL_OK;
				break; // Successful completion, file for update found
			}
		}

		// Scan SD Card for Application SW binary (if Bootloader SW binary has not been found)
		fn = SD_Card_ScanAndSelectFile(APPLICATION_SW, BL_APP_ADDR);

		if (fn != NULL)
		{
			if (bl_GetMetadata_BIN(fn, metadata, header, bin_size) != BL_OK)
			{
				break; // Error reading file metadata
			}

			if (bl_GetMetadata_FLASH(&metadata_flash, APPLICATION_SW) != BL_OK)
			{
				// Application code is corrupted - needs immediate update
				*filename = (char*) fn;
				retVal = BL_OK;
				break; // Successful completion, file for update found
			}

			if (metadata->version > metadata_flash.version)
			{
				*filename = (char*) fn;
				retVal = BL_OK;
				break; // Successful completion, file for update found
			}
		}

		// If no matching file was found
		retVal = BL_ERROR;
	}
	while(false);

	return retVal;
}


static BL_Status_t bl_Verify_FLASH(void)
{
	BL_Status_t retVal = BL_OK;
	Metadata_t metadata = {BL_ZERO};
	uint32_t crc = 0;
	uint32_t *app_data;
	uint32_t app_len;

	do
	{
		/* Read Application Metadata from FLASH */
		retVal = bl_GetMetadata_FLASH(&metadata, APPLICATION_SW);

		if (retVal != BL_OK)
		{
			retVal = BL_ERROR;
			break;
		}

		/* Check Application Header */
		Header_t *app_header = (Header_t*) BL_APP_HEADER_ADDR;

		if (app_header == NULL || app_header->metadata_addr <= BL_APP_ADDR)
		{
			retVal = BL_ERROR;
			break;
		}

		app_data = (uint32_t*) BL_APP_ADDR;
		app_len = app_header->metadata_addr - BL_APP_ADDR;

		/* Check Application length */
		if ((app_len == BL_ZERO) || (app_len % sizeof(uint32_t) != BL_ZERO))
		{
			retVal = BL_ERROR;
			break;
		}

		/* Calculate CRC32 for FLASH APP */
		crc = HAL_CRC_Calculate(&hcrc, app_data, (app_len / sizeof(uint32_t)));

		if (crc != metadata.crc_value)
		{
			retVal = BL_ERROR;
			break;
		}

	} while (false);

	return retVal;
}


static BL_Status_t bl_Verify_BIN(const char* filename, const Metadata_t* metadata, const Header_t* header, uint32_t bin_size)
{
	BL_Status_t retVal = BL_OK;
	uint32_t data_offset = header->metadata_addr - GET_START_ADDR(header);

	/* Calculate CRC32 for binary */
	__HAL_CRC_DR_RESET(&hcrc);
	retVal |= SD_Card_Read_Binary(filename, GET_START_ADDR(header), data_offset, RAM_buffer, RAM_BUFFER_SIZE, bl_CRC_calc_cbk);
	// crc_accumulator will be updated under bl_CRC_calc_cbk() callback

	/* Check CRC32 */
	retVal |= (crc_accumulator == metadata->crc_value) ? BL_OK : BL_ERROR;

	return retVal;
}


static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_Erase_FLASH(uint32_t start_addr, uint32_t bin_size)
{
	BL_Status_t retVal = BL_OK;
	uint32_t sector;
	uint32_t last_sector;
	uint32_t end_addr;

	/* Check if the file size exceeds the size of the available FLASH */
	if (bin_size > (FLASH_BASE + BL_FLASH_SIZE - start_addr))
	{
		retVal = BL_ERROR;
	}
	else
	{
		/* If it does not exceed, erase necessary sectors used for writing this file */

	    // Calculate the end address
	    end_addr = start_addr + bin_size - 1UL;

	    // Unlock flash
	    bl_FLASH_Unlock();

	    // Identify the first and last sectors
	    sector = GET_SECTOR_NUMBER(start_addr);
	    last_sector = GET_SECTOR_NUMBER(end_addr);

	    // Loop through sectors and erase each one
	    while (sector <= last_sector)
	    {
			if (BL_OK != bl_FLASH_EraseSector(sector))
			{
				retVal = BL_ERROR;
				break;
			}

	        // Move to the next sector
			sector++;
	    }

	    // Lock flash after erase
	    bl_FLASH_Lock();
	}

	return retVal;
}

static BL_Status_t bl_Write_BIN_to_FLASH(const char* filename, uint32_t start_addr, uint32_t bin_size)
{
	BL_Status_t retVal;
	// Flash the binary from the SD card
	retVal = SD_Card_Read_Binary(filename, start_addr, bin_size, RAM_buffer, RAM_BUFFER_SIZE, bl_WriteChunk_cbk);
	// Flash memory will be updated under bl_WriteChunk_cbk() callback
	return retVal;
}

static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_Write_RAM_to_FLASH(uint32_t start_addr, uint32_t bin_size)
{
	BL_Status_t retVal = BL_OK;
	uint32_t addr = start_addr;
	uint32_t remaining = bin_size;
	uint32_t *buffer = (uint32_t*) RAM_buffer;

    // Check input parameters
    if (start_addr < FLASH_BASE || (start_addr + bin_size) > FLASH_END)
    {
    	retVal = BL_ERROR; // wrong address range
    }
    else
    {
		// Unlock flash memory for writing
    	bl_FLASH_Unlock();

		// Write data to the FLASH
		while (remaining >= FLASH_WORD_SIZE)
		{
			bl_FLASH_ProgramWord(addr, *buffer);
			// Update address and buffer
			addr += FLASH_WORD_SIZE;
			buffer++;
			remaining -= FLASH_WORD_SIZE;
		}

		// Lock flash after erase
		bl_FLASH_Lock();
	}

	return retVal;
}


static BL_Status_t bl_Copy_BIN_to_RAM(const char* filename, uint32_t start_addr, uint32_t bin_size)
{
	BL_Status_t retVal;

	// Copy the binary from the SD card to RAM buffer
	retVal = SD_Card_Read_Binary(filename, start_addr, bin_size, RAM_buffer, RAM_BUFFER_SIZE, NULL);
	// Flash memory will be updated under bl_WriteChunk_cbk() callback

	return retVal;
}


/* CRC calculation for data chunk callback */
static BL_Status_t bl_CRC_calc_cbk(uint8_t *data, uint32_t length, uint32_t current_address)
{
    // Accumulate the CRC for the current chunk
    crc_accumulator = HAL_CRC_Accumulate(&hcrc, (uint32_t *)data, length / BL_WORD_SIZE);

    return BL_OK;
}


/* Write data chunk to FLASH callback */
static BL_Status_t bl_WriteChunk_cbk(uint8_t *data, uint32_t length, uint32_t start_address)
{
	BL_Status_t retVal = BL_OK;
	uint32_t word;

	// Ensure address alignment for flash programming
	if ((start_address % BL_WORD_SIZE) == 0U)
	{
		// Unlock flash memory for writing
		bl_FLASH_Unlock();

		// Write data in 32-bit words
		for (uint32_t i = 0; i < length; i += BL_WORD_SIZE)
		{
			word = *(uint32_t*) (data + i);

			// Write the word to flash memory
			if (BL_OK != bl_FLASH_ProgramWord(start_address + i, word))
			{
				//Flash programming error
				retVal = BL_ERROR;
				break;
			}
		}

		// Lock flash memory after programming
		bl_FLASH_Lock();
	}
	else
	{
		/* Do nothing - address not aligned to 32-bit word */
	}

	return retVal;
}


static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_SW_Update(const char* filename, const Header_t* header, uint32_t bin_size)
{
	BL_Status_t retVal = BL_OK;
	uint32_t start_addr;

	switch (header->label)
	{
		case BOOTLOADER_SW:
		{
			start_addr = BL_ADDR;
			/* Copy BL binary to RAM buffer */
			retVal = bl_Copy_BIN_to_RAM(filename, start_addr, bin_size);
			__disable_irq();
			/* Erate the FLASH */
			retVal = bl_Erase_FLASH(start_addr, bin_size);

			if (BL_OK == retVal)
			{
				/* Write binary to FLASH */
				retVal = bl_Write_RAM_to_FLASH(start_addr, bin_size);
				/* Signal the user that the bootloader update is complete */
				bl_LED_blink(5UL);
			}
			break;
		}

		case APPLICATION_SW:
		{
			start_addr = BL_APP_ADDR;
			/* Erate the FLASH */
			retVal = bl_Erase_FLASH(start_addr, bin_size);

			if (BL_OK == retVal)
			{
				/* Write binary to FLASH  */
				retVal = bl_Write_BIN_to_FLASH(filename, start_addr, bin_size);
				/* Signal the user that the application update is complete */
				bl_LED_blink(10UL);
			}
			break;
		}

		default:
		{
			retVal = BL_ERROR;
			break;
		}
	}

	/* Update is done - reset the uC */
	bl_SystemReset_RAM();

	return retVal;
}


static __attribute__((section(".RamFunc"), used)) __NO_RETURN void bl_SystemReset_RAM(void)
{
    // Force clearing of all pending write operations
    __DSB();
    // Setting bits in AIRCR to initiate a system reset
    WRITE_REG(SCB->AIRCR, (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) | SCB_AIRCR_SYSRESETREQ_Msk));
    // Ensure that all memory operations are completed
    __DSB();
    // wait for reset
    for(;;){__NOP();}
}


static __attribute__((section(".RamFunc"), used)) void bl_FLASH_Unlock(void)
 {
	if (READ_BIT(FLASH->CR, FLASH_CR_LOCK))
	{
		WRITE_REG(FLASH->KEYR, FLASH_KEY1);
		WRITE_REG(FLASH->KEYR, FLASH_KEY2);
	}
}


static __attribute__((section(".RamFunc"), used)) void bl_FLASH_Lock(void)
{
	SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}


static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_FLASH_GetErrStatus(void)
{
	BL_Status_t retVal = BL_OK;
	if (READ_BIT(FLASH->SR, (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)))
	{
		retVal = BL_ERROR;
	}

	return retVal;
}

static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_FLASH_EraseSector(uint32_t sector_num)
{
	BL_Status_t retVal = BL_OK;
	uint32_t timeout = FLASH_TIMEOUT_MS * (SystemCoreClock / 1000UL);

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
	SET_BIT(FLASH->CR, (sector_num << FLASH_CR_SNB_Pos));
	// Start operation
	SET_BIT(FLASH->CR, FLASH_CR_STRT);

	// Wait for operation to complete
	while (READ_BIT(FLASH->SR, FLASH_SR_BSY))
	{
		if (timeout-- == 0U)
		{
			retVal = BL_ERROR;
			break;
		}
	}

	// Check error flags
	if (BL_OK != bl_FLASH_GetErrStatus())
	{
		retVal = BL_ERROR;
	}

	return retVal;
}

static __attribute__((section(".RamFunc"), used)) BL_Status_t bl_FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
	BL_Status_t retVal = BL_OK;
	uint32_t timeout;

	// Wait until FLASH is free
	while (READ_BIT(FLASH->SR, FLASH_SR_BSY));

	// Clear error flags
	SET_BIT(FLASH->SR, (FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR | FLASH_SR_WRPERR));
	// Set program size (32-bit word)
	CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
	SET_BIT(FLASH->CR, (FLASH_VOLTAGE_RANGE_3 << FLASH_CR_PSIZE_Pos));
	// Activate programming mode
	SET_BIT(FLASH->CR, FLASH_CR_PG);
	// Write 32-bit word
	*((volatile uint32_t*) Address) = Data;

	// Wait for the operation to complete
	timeout = FLASH_TIMEOUT_MS * (SystemCoreClock / 1000U);
	while (READ_BIT(FLASH->SR, FLASH_SR_BSY))
	{
		if (timeout-- == 0U)
		{
			retVal = BL_ERROR;        // Timeout
			break;
		}
	}

	// Check error flags
	if (BL_OK != bl_FLASH_GetErrStatus())
	{
		retVal = BL_ERROR;
	}

	return retVal;
}


static __attribute__((section(".RamFunc"), used)) void bl_ErrorHook(void)
{
	bl_LED_blink(0xFFFFFFFFUL);
}

static __attribute__((section(".RamFunc"), used)) void bl_LED_blink(uint32_t cnt)
{
	while(cnt)
	{
		SET_BIT(CAM_LED_GPIO_Port->BSRR, (1U << CAM_LED_Pin_N));

		for (volatile uint32_t i = 0; i < (0x60000UL); i++)
		{
			__NOP();
		}

		SET_BIT(CAM_LED_GPIO_Port->BSRR, (1U << (CAM_LED_Pin_N + 16U)));

		for (volatile uint32_t i = 0; i < (0x100000UL); i++)
		{
			__NOP();
		}

		cnt--;
	}
}
