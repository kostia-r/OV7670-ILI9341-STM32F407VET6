/*
 * SD_Card.c
 * SD Card Handler
 *  Created on: Oct 24, 2024
 *      Author: K.Rudenko
 */

#include <stdio.h>
#include <stdint.h>
#include "CAMERA_APP.h"
#include "SD_Card.h"
#include "fatfs.h"

void SD_Test(void)
 {
    char filename[] = "test22.txt";
    char input[] = "HiFromStm32F407 and this is a test program\r\n";
    char output[100] = { 0x0U };
    retSD = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);

    if (retSD == FR_OK)
    {
        retSD = f_open(&SDFile, filename, FA_WRITE | FA_OPEN_ALWAYS);

        if (retSD == FR_OK)
        {
            UINT i;
            f_write(&SDFile, input, sizeof(input), &i);
            f_close(&SDFile);
        }

        retSD = f_open(&SDFile, filename, FA_READ);

        if (retSD == FR_OK)
        {
            UINT i;
            f_read(&SDFile, output, sizeof(output), &i);
            f_close(&SDFile);
        }
    }
}
