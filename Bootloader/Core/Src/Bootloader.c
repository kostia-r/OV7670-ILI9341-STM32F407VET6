/*
 * Bootloader.c
 * Bootloader
 *  Created on: Nov 19, 2024
 *      Author: K.Rudenko
 */

#include "Bootloader.h"
#include "stm32f4xx_hal.h"


#define BOOTLOADER_APP_ADDR            (0x08010000U)



void BL_JumpToApp(void)
{
	// pointer to APP Reset Handler
	void (*App_Reset_Handler)(void);
	// Configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_val = *(volatile uint32_t*)BOOTLOADER_APP_ADDR;
	__set_MSP(msp_val);
	// Fetch the Reset Handler address of the user app
	uint32_t reset_handler_address = *(volatile uint32_t*)(BOOTLOADER_APP_ADDR + 4U);
	App_Reset_Handler = (void*)reset_handler_address;
	// Deinitialize HAL
	HAL_DeInit();
	// Jump to Reset Handler of the User Application
	App_Reset_Handler();
}
