/*
 * Bootloader.c
 * Bootloader
 *  Created on: Nov 19, 2024
 *      Author: K.Rudenko
 */

#include "Bootloader.h"
#include "stm32f4xx_hal.h"
#include "crc.h"
#include "SD_Card.h"
#include "main.h"
#include "fatfs.h"
#include <stdbool.h>

extern CRC_HandleTypeDef hcrc;

static const char* app_file_name = "F407VET6_OV7670_ILI9341_HAL.bin";

static void bl_JumpToApp(void);
static HAL_StatusTypeDef bl_GetAppMetaData_BIN(AppMetadata* metadata);
static HAL_StatusTypeDef bl_GetAppMetaData_FLASH(AppMetadata* metadata);
static uint32_t bl_CalcCRC_FLASH(void);

void BL_Main(void)
{
	AppMetadata metadata_bin, metadata_flash;
	uint32_t crc_bin, crc_flash;

	// If two buttons are pressed - go to updater mode
	if (HAL_GPIO_ReadPin(GPIOC, CAM_BTN1_Pin | CAM_BTN2_Pin) == GPIO_PIN_SET)
	{
		SD_Card_Init();

		bl_GetAppMetaData_BIN(&metadata_bin);
		bl_GetAppMetaData_FLASH(&metadata_flash);

		SD_Card_CheckCRC(app_file_name, NULL, &crc_bin);
		crc_flash = bl_CalcCRC_FLASH();

		while(1)
		{
			HAL_GPIO_TogglePin(CAM_LED_GPIO_Port, CAM_LED_Pin);
			HAL_Delay(500);
		}
	}
	// otherwize - jump to the main application
	else
	{
		bl_JumpToApp();
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


static void bl_JumpToApp(void)
{
	// pointer to APP Reset Handler
	void (*App_Reset_Handler)(void);

	// Verify Application on Flash
	AppMetadata metadata_flash;
	uint32_t crc_flash;
	bl_GetAppMetaData_FLASH(&metadata_flash);
	crc_flash = bl_CalcCRC_FLASH();

	// Check APP CRC32 in FLASH before jumping
	if (crc_flash == metadata_flash.crc_value)
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
		while (1)
		{
			HAL_GPIO_TogglePin(CAM_LED_GPIO_Port, CAM_LED_Pin);
			HAL_Delay(500);
		}
	}
}

static HAL_StatusTypeDef bl_GetAppMetaData_BIN(AppMetadata* metadata)
{
	return SD_Card_Read_Metadata(app_file_name, metadata);
}

static HAL_StatusTypeDef bl_GetAppMetaData_FLASH(AppMetadata* metadata)
{
	AppHeader *AppHeaderPtr = (AppHeader*) BL_APP_HEADER_ADDR;
	metadata->crc_value = ((AppMetadata* )(AppHeaderPtr->metadata_addr))->crc_value;
	metadata->version = ((AppMetadata* )(AppHeaderPtr->metadata_addr))->version;
	return HAL_OK;
}

static uint32_t bl_CalcCRC_FLASH(void)
{
	uint32_t* app_data = (uint32_t*)BL_APP_ADDR;
	uint32_t app_len = ((AppHeader* )BL_APP_HEADER_ADDR)->metadata_addr - BL_APP_ADDR;
	return HAL_CRC_Calculate(&hcrc, app_data, (app_len / sizeof(uint32_t)));
}

static uint32_t bl_CalcCRC_BIN(void)
{
	uint32_t* app_data = (uint32_t*)BL_APP_ADDR;
	uint32_t app_len = ((AppHeader* )BL_APP_HEADER_ADDR)->metadata_addr - BL_APP_ADDR;
	return HAL_CRC_Calculate(&hcrc, app_data, (app_len / sizeof(uint32_t)));
}

