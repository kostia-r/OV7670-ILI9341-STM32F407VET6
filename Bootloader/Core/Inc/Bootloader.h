/*
 * Bootloader.h
 * Bootloader
 *  Created on: Nov 19, 2024
 *      Author: K.Rudenko
 */


#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include <stdint.h>

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
#define BL_APP_HEADER_SIZE                                 sizeof(AppHeader)
#define BL_APP_METADATA_SIZE                             sizeof(AppMetadata)

extern void BL_Main(void);

#endif /* BOOTLOADER_H_ */
