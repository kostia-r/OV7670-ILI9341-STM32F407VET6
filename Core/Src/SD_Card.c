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

//TODO: MAKE IT AS CALLBACK FROM APPLICATION!!!!
#include "ILI9341.h"

#if(ILI9341_ORIENTATION == ILI9341_PORTRAIT)
    #define  ILI9341_ACTIVE_WIDTH       ILI9341_WIDTH
    #define  ILI9341_ACTIVE_HEIGHT      ILI9341_HEIGHT
#elif(ILI9341_ORIENTATION == ILI9341_LANDSCAPE)
    #define  ILI9341_ACTIVE_WIDTH       ILI9341_HEIGHT
    #define  ILI9341_ACTIVE_HEIGHT      ILI9341_WIDTH
#endif

#define CHUNK_HEIGHT                           10U    // Height of each image chunk in pixels
#define CHUNK_SIZE                             (ILI9341_ACTIVE_WIDTH * CHUNK_HEIGHT)
#define RGB565_SIZE_BYTES                      2U
#define RGB666_SIZE_BYTES                      3U
#define RGB888_SIZE_BYTES                      3U
#define RGB_TYPE                               RGB888_SIZE_BYTES
#define BUFFER_SIZE                           (RGB_TYPE * CHUNK_SIZE)
#define BMP_HEADER_SIZE                        54U

#define FILE_FORMAT_RAW                        1
#define FILE_FORMAT_BMP                        0


static uint8_t img_chunk_buffer[BUFFER_SIZE];

#if (FILE_FORMAT_BMP == 1)
// Function to write BMP header
static void write_bmp_header(FIL *file, uint16_t width, uint16_t height)
{
    UINT bytes_written;

    // BMP Header structure:
    uint8_t bmp_header[54] = { 0 };

    // File type identifier ('BM')
    bmp_header[0] = 'B';
    bmp_header[1] = 'M';

    // Calculate file size: 54 bytes header + width * height * 3 bytes per pixel
    uint32_t file_size = 54 + (width * height * 3);
    bmp_header[2] = (uint8_t) (file_size);
    bmp_header[3] = (uint8_t) (file_size >> 8);
    bmp_header[4] = (uint8_t) (file_size >> 16);
    bmp_header[5] = (uint8_t) (file_size >> 24);

    // Reserved fields (set to 0)
    bmp_header[6] = bmp_header[7] = bmp_header[8] = bmp_header[9] = 0;

    // Offset where the pixel data (bitmap data) starts
    bmp_header[10] = 54;  // Pixel data offset

    // DIB header size (40 bytes for BITMAPINFOHEADER)
    bmp_header[14] = 40;

    // Image width and height
    bmp_header[18] = (uint8_t) (width);
    bmp_header[19] = (uint8_t) (width >> 8);
    bmp_header[20] = (uint8_t) (width >> 16);
    bmp_header[21] = (uint8_t) (width >> 24);
    bmp_header[22] = (uint8_t) (height);
    bmp_header[23] = (uint8_t) (height >> 8);
    bmp_header[24] = (uint8_t) (height >> 16);
    bmp_header[25] = (uint8_t) (height >> 24);

    // Planes (1) and Bits per Pixel (24 for RGB888)
    bmp_header[26] = 1;
    bmp_header[28] = 24;

    // Compression method (0 - none)
    bmp_header[30] = 0;

    // Image size (can be 0 for uncompressed images)
    uint32_t image_size = width * height * 3;
    bmp_header[34] = (uint8_t) (image_size);
    bmp_header[35] = (uint8_t) (image_size >> 8);
    bmp_header[36] = (uint8_t) (image_size >> 16);
    bmp_header[37] = (uint8_t) (image_size >> 24);

    // Resolution (pixels per meter - optional, setting to 2835 ~ 72 DPI)
    uint32_t ppm = 2835;
    bmp_header[38] = (uint8_t) (ppm);
    bmp_header[39] = (uint8_t) (ppm >> 8);
    bmp_header[42] = (uint8_t) (ppm);
    bmp_header[43] = (uint8_t) (ppm >> 8);

    // Write the header to the file
    f_write(file, bmp_header, sizeof(bmp_header), &bytes_written);
}
#endif

void SD_Card_WriteScreenToSD(void)
{
    FIL file;
    FRESULT res;
    UINT bytes_written;
#if (FILE_FORMAT_BMP == 1)
    char filename[] = "img.bmp";
#else
    char filename[] = "img.raw";
#endif

    retSD = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);

    if (retSD != FR_OK)
    {
        return;  // Handle file open error
    }

    // Open file for writing (create/overwrite)
    res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        return;  // Handle error
    }

#if (FILE_FORMAT_BMP == 1)
    // Write BMP header
    write_bmp_header(&file, ILI9341_ACTIVE_WIDTH, ILI9341_ACTIVE_HEIGHT);

    /* Read the full display area line-by-line in BMP image line order */
    for (uint16_t y = ILI9341_ACTIVE_HEIGHT - 1; y > 0; y-= CHUNK_HEIGHT)
#else
    /* Read the full display area line-by-line in RAW image line order */
    for (uint16_t y = 0; y < ILI9341_ACTIVE_HEIGHT; y += CHUNK_HEIGHT)
#endif
    {
        //TODO: pass here desired pixel format as input enum parameter
        ILI9341_Read_GRAM(0U, y, (ILI9341_ACTIVE_WIDTH - 1U), (y + CHUNK_HEIGHT - 1U), img_chunk_buffer);

        res = f_write(&file, img_chunk_buffer, BUFFER_SIZE, &bytes_written);
        if (res != FR_OK || bytes_written != BUFFER_SIZE)
        {
            f_close(&file);
            return;
        }
    }

    // Close file
    f_close(&file);
}


void SD_Card_Test(void)
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
