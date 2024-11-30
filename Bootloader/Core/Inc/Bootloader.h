/*
 * Bootloader.h
 * Bootloader
 *  Created on: Nov 19, 2024
 *      Author: K.Rudenko
 */


#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

typedef enum
{
  BL_OK       = 0x00U,
  BL_ERROR    = 0x01U,
  BL_BUSY     = 0x02U,
  BL_TIMEOUT  = 0x03U
} BL_Status_t;

typedef enum
{
	BOOTLOADER_SW = 0xA5A5A5A5UL,
	APPLICATION_SW = 0x5A5A5A5AUL,
}Binary_t;

typedef struct
{
    uint32_t metadata_addr;
    Binary_t label;
} Header_t;

typedef struct
{
    uint32_t crc_value;
    uint32_t version;
} Metadata_t;

/******************************************************************************
 *                               GLOBAL MACRO                                 *
 ******************************************************************************/
/* Flash memory size for STM32F407VET6 uC: 512K */
#define BL_FLASH_SIZE                                            (0x0007FFFFUL)
#define BL_ADDR                                                  (0x08000000UL)
#define BL_APP_ADDR                                              (0x08010000UL)
#define BL_APP_VT_SIZE                                                (0x188UL)
#define BL_APP_HEADER_ADDR                       (BL_APP_ADDR + BL_APP_VT_SIZE)
#define BL_HEADER_ADDR                               (BL_ADDR + BL_APP_VT_SIZE)
#define BL_HEADER_SIZE                                         sizeof(Header_t)
#define BL_METADATA_SIZE                                     sizeof(Metadata_t)
#define BL_WORD_SIZE                                         (sizeof(uint32_t))

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern void BL_Main(void);

#endif /* BOOTLOADER_H_ */
