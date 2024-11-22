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

#include <stdint.h>

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

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

/******************************************************************************
 *                               GLOBAL MACRO                                 *
 ******************************************************************************/

#define BL_APP_ADDR                                            (0x08010000U)
#define BL_APP_VT_SIZE                                              (0x188U)
#define BL_APP_HEADER_ADDR                    (BL_APP_ADDR + BL_APP_VT_SIZE)
#define BL_APP_HEADER_SIZE                                 sizeof(AppHeader)
#define BL_APP_METADATA_SIZE                             sizeof(AppMetadata)

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern void BL_Main(void);

#endif /* BOOTLOADER_H_ */
