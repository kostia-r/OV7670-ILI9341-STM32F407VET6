/*
 * SD_Card.c
 * SD Card Photo Viewer
 *  Created on: Nov 5, 2024
 *      Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "SD_Card.h"
#include "ILI9341.h"
#include "fatfs.h"
#include "jpeglib.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define IMG_PREFIX                                                        "img"
#define IMG_EXTENSION                                                     "jpg"
#define MAX_IMAGE_NAME_LENGTH                                             (32U)
#define MAX_IMAGE_COUNT                                                  (400U)

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern const Diskio_drvTypeDef  SD_Driver;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static uint8_t SD_RGB888_buffer[RGB888_SIZE_BYTES * ILI9341_ACTIVE_WIDTH];
static uint8_t SD_RGB565_buffer[RGB565_SIZE_BYTES * ILI9341_ACTIVE_WIDTH];

// Array to store JPEG image filenames and a variable to track the current index
static char imageFiles[MAX_IMAGE_COUNT][MAX_IMAGE_NAME_LENGTH];
static int totalImages = 0;
static int currentIndex = -1;

// Date and Time (preconfigured) of creation for photos saved to the SD Card:
uint16_t year = 2024U;
uint8_t month = 11U;
uint8_t day = 5U;
uint8_t hours = 22U;
uint8_t minutes = 15U;
uint8_t seconds = 1U;

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static bool checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver);
static void drawJPEGFile(const char *filename);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

void SD_PhotoViewer_Init(void)
{
    (void)checkAndInitSD(&SDFatFS, SDPath, &SD_Driver);
}

void SD_PhotoViewer_Prev(void)
{
    DEBUG_LOG("[APP] CAM_readPrevFromSD");

    if (totalImages == 0 || currentIndex <= 0)
    {
        return;  // No previous image
    }

    currentIndex--;
    drawJPEGFile(imageFiles[currentIndex]);
}

void SD_PhotoViewer_Next(void)
{
    DEBUG_LOG("[APP] CAM_readNextFromSD");

    if (totalImages == 0 || currentIndex >= totalImages - 1)
    {
        return;  // No next image
    }

    currentIndex++;
    drawJPEGFile(imageFiles[currentIndex]);
}

void SD_PhotoViewer_Open(void)
{
    DEBUG_LOG("[APP] CAM_PhotoViewer_Init");

    DIR dir;
    FILINFO fno;
    totalImages = 0;
    currentIndex = -1;

    do
    {
        if (!checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
            // Error
            break;
        }

        if (f_opendir(&dir, "/") != FR_OK)
        {
            // Error
            break;
        }

        while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0])
        {
            if (!(fno.fattrib & AM_DIR)
                    && strstr(fno.fname, IMG_PREFIX) == fno.fname)
            {
                char *ext = strrchr(fno.fname, '.');
                if (ext && strcmp(ext + 1, IMG_EXTENSION) == 0)
                {
                    if (totalImages < MAX_IMAGE_COUNT)
                    {
                        strncpy(imageFiles[totalImages++], fno.fname, MAX_IMAGE_NAME_LENGTH);
                    }
                }
            }
        }
        f_closedir(&dir);

        if (totalImages > 0)
        {
            currentIndex = totalImages - 1;
            drawJPEGFile(imageFiles[currentIndex]);
        }
    }
    while (false);
}

void SD_PhotoViewer_Save(void)
{
    DEBUG_LOG("[APP] writeToSD");

    FIL file;
    FILINFO fno;
    uint16_t index = 0;

    char filename[30] = { 0x0U };

    /* Do image transfering from Display, converting to JPG and storing on SD Card */
    do
    {
        if (!checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
            // Error
            break;
        }

        /* Generate unique file name */
        do
        {
            sprintf(filename, "%s_%d.%s", IMG_PREFIX, index, IMG_EXTENSION);
            index++;
        }
        while (f_stat(filename, &fno) == FR_OK); // FR_OK returns if file exists

        /* Create a new file with the unique name */
        if (FR_OK != f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE))
        {
            // Error
            break;
        }

        /* Do LIBJPEG configuration */
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        JSAMPROW row_pointer = SD_RGB888_buffer;
        cinfo.err = jpeg_std_error(&jerr);

        jpeg_create_compress(&cinfo);
        jpeg_stdio_dest(&cinfo, &file);

        /* image width and height, in pixels */
        cinfo.image_width = ILI9341_ACTIVE_WIDTH;
        cinfo.image_height = ILI9341_ACTIVE_HEIGHT;
        /* # of color components per pixel */
        cinfo.input_components = 3;
        /* colorspace of input image */
        cinfo.in_color_space = JCS_RGB;

        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, 90, TRUE);
        jpeg_start_compress(&cinfo, TRUE);

        /* Start processing line by line */
        while (cinfo.next_scanline < cinfo.image_height)
        {
            /* Read existing image line from the display */
            ILI9341_Read_GRAM(0U, cinfo.next_scanline, (ILI9341_ACTIVE_WIDTH - 1U), cinfo.next_scanline, SD_RGB888_buffer);
            /* Pass this line to the LIBJPEG and write to SD */
            (void) jpeg_write_scanlines(&cinfo, &row_pointer, 1U);
        }

        /* Finalize compression */
        jpeg_finish_compress(&cinfo);
        /* Close the file */
        f_close(&file);
        /* Stop LIBJPEG */
        jpeg_destroy_compress(&cinfo);
    }
    while (FALSE);
}

/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/

static bool checkAndInitSD(FATFS *fs, char *SDPath, const Diskio_drvTypeDef *SD_Driver)
{
    bool retVal = true;
    DWORD free_clusters;

    if (f_getfree(SDPath, &free_clusters, &fs) != FR_OK)
    {
        /* Unmount drive */
        f_mount(NULL, (TCHAR const*) SDPath, 1);
        FATFS_UnLinkDriver((TCHAR*) SDPath);
        FATFS_LinkDriver(SD_Driver, (TCHAR*) SDPath);
        /* Mount drive */
        if (FR_OK != f_mount(fs, (TCHAR const*) SDPath, 1))
        {
            retVal = false;
        }
    }

    return retVal;
}

static void drawJPEGFile(const char *filename)
{
    FIL file;

    do
    {
        if (!checkAndInitSD(&SDFatFS, SDPath, &SD_Driver))
        {
            // Error
            break;
        }

        if (f_open(&file, filename, FA_READ) != FR_OK)
        {
            // Error
            break;
        }

        /* Do LIBJPEG configuration */
        struct jpeg_decompress_struct cinfo;
        struct jpeg_error_mgr jerr;
        JSAMPROW row_pointer = SD_RGB888_buffer;
        uint8_t *pixel_src_ptr;
        uint16_t *pixel_dst_ptr;
        cinfo.err = jpeg_std_error(&jerr);

        jpeg_create_decompress(&cinfo);
        jpeg_stdio_src(&cinfo, &file);

        jpeg_read_header(&cinfo, TRUE);
        jpeg_start_decompress(&cinfo);

        if (cinfo.output_width > ILI9341_ACTIVE_WIDTH || cinfo.output_height > ILI9341_ACTIVE_HEIGHT)
        {
            jpeg_destroy_decompress(&cinfo);
            f_close(&file);
            break;
        }

        while (cinfo.output_scanline < cinfo.output_height)
        {
            jpeg_read_scanlines(&cinfo, &row_pointer, 1);
            pixel_dst_ptr = (uint16_t*) SD_RGB565_buffer;
            pixel_src_ptr = SD_RGB888_buffer;

            /* Convert pixels by pairs, RGB888 -> RGB565 */
            for (uint16_t x = 0U; x < cinfo.output_width; x += 2)
            {
                /* First pixel */
                uint8_t r = *pixel_src_ptr++;
                uint8_t g = *pixel_src_ptr++;
                uint8_t b = *pixel_src_ptr++;
                *pixel_dst_ptr++ = RGB888_TO_RGB565(r, g, b);

                /* Second pixel */
                if (x + 1 < cinfo.output_width) // Ensure we're not out of bounds
                {
                    r = *pixel_src_ptr++;
                    g = *pixel_src_ptr++;
                    b = *pixel_src_ptr++;
                    *pixel_dst_ptr++ = RGB888_TO_RGB565(r, g, b);
                }
            }

            /* Draw line */
            ILI9341_DrawCrop(SD_RGB565_buffer, (ILI9341_ACTIVE_WIDTH * RGB565_SIZE_BYTES), 0U,
                    cinfo.output_width, cinfo.output_scanline, cinfo.output_scanline);
        }

        jpeg_finish_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        f_close(&file);
    }
    while (false);
}

