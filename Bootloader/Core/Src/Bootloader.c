/*
 * Bootloader.c
 * Bootloader
 *  Created on: Nov 19, 2024
 *      Author: K.Rudenko
 */

#include "Bootloader.h"
#include "stm32f4xx_hal.h"
#include "crc.h"
#include "main.h"
#include <stdbool.h>


typedef struct
{
    uint32_t metadata_addr;
    uint32_t reserved;
} AppHeader;

typedef struct
{
    uint32_t crc_value;
    uint32_t version;
} AppMetadata;

#define BL_APP_ADDR                                            (0x08010000U)
#define BL_APP_VT_SIZE                                              (0x188U)
#define BL_APP_HEADER_ADDR                    (BL_APP_ADDR + BL_APP_VT_SIZE)

extern CRC_HandleTypeDef hcrc;

static uint32_t bl_GetVersion_FLASH(void);
static uint32_t bl_GetVersion_BIN(void);
static bool bl_VerifyCRC_FLASH(void);
static uint32_t bl_GetCRC_FLASH(void);
static uint32_t bl_GetCRC_BIN(void);
static uint32_t bl_CalcCRC_FLASH(void);


void BL_JumpToApp(void)
{
	// pointer to APP Reset Handler
	void (*App_Reset_Handler)(void);

	// Check APP CRC32 in FLASH before jumping
	if (bl_VerifyCRC_FLASH())
	{
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
	else
	{
		// there is a CRC error!
		Error_Handler();
	}
}

void BL_UpdateApp(void)
{

}

void BL_ReadApp(void)
{

}

void BL_CheckBinaries(void)
{

}

void BL_Updater(void)
{

}

//TODO: Implement BL Updater



uint8_t BL_CheckVersion_FLASH(void)
{
	uint32_t ver_flash = bl_GetVersion_FLASH();
	(void)ver_flash;
	return 0;
}




static uint32_t bl_GetVersion_FLASH(void)
{
	AppHeader* AppHeaderPtr = (AppHeader* )BL_APP_HEADER_ADDR;
	AppMetadata* AppMetadataPtr = (AppMetadata* )(AppHeaderPtr->metadata_addr);
	return AppMetadataPtr->version;
}

static uint32_t bl_GetVersion_BIN(void)
{
	return 0;
}

static uint32_t bl_GetCRC_BIN(void)
{
	return 0;
}

static bool bl_VerifyCRC_FLASH(void)
{
	return (bl_CalcCRC_FLASH() == bl_GetCRC_FLASH()) ? true : false;
}

static uint32_t bl_GetCRC_FLASH(void)
{
	AppHeader* AppHeaderPtr = (AppHeader* )BL_APP_HEADER_ADDR;
	AppMetadata* AppMetadataPtr = (AppMetadata* )(AppHeaderPtr->metadata_addr);
	return AppMetadataPtr->crc_value;
}

static uint32_t bl_CalcCRC_FLASH(void)
{
	uint32_t* app_data = (uint32_t*)BL_APP_ADDR;
	uint32_t app_len = ((AppHeader* )BL_APP_HEADER_ADDR)->metadata_addr - BL_APP_ADDR;
	return HAL_CRC_Calculate(&hcrc, app_data, (app_len / sizeof(uint32_t)));
}

