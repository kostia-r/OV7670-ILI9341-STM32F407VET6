/*
 * CAMERA_APP.c
 * Camera Application Main logic
 *  Created on: Aug 15, 2024
 *      Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "CAMERA_APP.h"
#include "StateM.h"
#include "ILI9341.h"
#include "OV7670.h"
#include "Button.h"
#include "LED.h"
#include "fatfs.h"
#include "jpeglib.h"
#include <stdio.h>
#include <string.h>

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define PWM_SET_DUTY_CYCLE(htim, tim_ch, duty_cycle)\
    __HAL_TIM_SET_COMPARE(htim, tim_ch, ((__HAL_TIM_GET_AUTORELOAD(htim) * duty_cycle) / 100UL))

#define IMG_PREFIX                                                        "img"
#define IMG_EXTENSION                                                     "jpg"
#define MAX_IMAGE_NAME_LENGTH                                             (32U)
#define MAX_IMAGE_COUNT                                                  (400U)

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim5;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim14;
extern const Diskio_drvTypeDef  SD_Driver;

/* LOGO BMP */
/* Online converter image-to-C-array https://lvgl.io/tools/imageconverter */
extern const uint8_t LOGO[];
extern const uint32_t LOGO_size;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

/* Left Button Object (PC0) */
static Button_Handler* CAM_L_BTN;
/* Right Button Object (PC1) */
static Button_Handler* CAM_R_BTN;

static uint8_t CAM_img_buffer_888[RGB888_SIZE_BYTES * ILI9341_ACTIVE_WIDTH];

static uint8_t CAM_img_buffer_565[RGB565_SIZE_BYTES * ILI9341_ACTIVE_WIDTH];

// Array to store JPEG image filenames and a variable to track the current index
static char imageFiles[MAX_IMAGE_COUNT][MAX_IMAGE_NAME_LENGTH];
static int totalImages = 0;
static int currentIndex = -1;

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

//static void CAM_LedBrightness_Test(void);
static bool CAM_IsCardPresent(void);

/* ILI9341 SPI transmittion complete callback */
static void CAM_SPI_TC_cbk(void);

/* OV7679 DCMI callbacks */
static void CAM_DCMI_DrawLine_cbk(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y);
static void CAM_DCMI_DrawFrame_cbk(const uint8_t *buffer, uint32_t nbytes);

/* Button callbacks */
static void btn_L_onSinglePress_cbk(void);
static void btn_L_onDoublePress_cbk(void);
static void btn_L_onLongPress_cbk(void);
static void btn_R_onSinglePress_cbk(void);
static void btn_R_onDoublePress_cbk(void);
static void btn_R_onLongPress_cbk(void);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

void CAMERA_APP_Init(void)
{
    DEBUG_LOG(" --- CAMERA APP on STM32F407VET6 ---");

    /* Initialize ILI9341 SPI Display */
    ILI9341_Init(&hspi2, ILI9341_PIXEL_FMT_RGB565);
    ILI9341_RegisterCallback(ILI9341_TC_CBK, CAM_SPI_TC_cbk);
    ILI9341_RegisterCallback(ILI9341_ERR_CBK, Error_Handler);
    ILI9341_SetBackgroundColor(BLACK);

    /* Initialize OV7670 DCMI Camera */
    OV7670_Init(&hdcmi, &hi2c2, &htim5, TIM_CHANNEL_3);
    OV7670_RegisterCallback(OV7670_DRAWLINE_CBK, (OV7670_FncPtr_t) CAM_DCMI_DrawLine_cbk);
    OV7670_RegisterCallback(OV7670_DRAWFRAME_CBK, (OV7670_FncPtr_t) CAM_DCMI_DrawFrame_cbk);

    /* Initialize backlight brightness PWM (PA7) */
    //HAL_TIM_OC_Start(&htim14, TIM_CHANNEL_1);

    /* Initialize Board Left Button (PС0) */
    CAM_L_BTN = Button_Init(CAM_BTN1_GPIO_Port, CAM_BTN1_Pin, GPIO_PIN_SET, &htim11);
    Button_RegisterCallback(CAM_L_BTN, BUTTON_EVENT_SINGLE_PRESS, btn_L_onSinglePress_cbk);
    Button_RegisterCallback(CAM_L_BTN, BUTTON_EVENT_DOUBLE_PRESS, btn_L_onDoublePress_cbk);
    Button_RegisterCallback(CAM_L_BTN, BUTTON_EVENT_LONG_PRESS, btn_L_onLongPress_cbk);

    /* Initialize Board Right Button (PС1) */
    CAM_R_BTN = Button_Init(CAM_BTN2_GPIO_Port, CAM_BTN2_Pin, GPIO_PIN_SET, &htim11);
    Button_RegisterCallback(CAM_R_BTN, BUTTON_EVENT_SINGLE_PRESS, btn_R_onSinglePress_cbk);
    Button_RegisterCallback(CAM_R_BTN, BUTTON_EVENT_DOUBLE_PRESS, btn_R_onDoublePress_cbk);
    Button_RegisterCallback(CAM_R_BTN, BUTTON_EVENT_LONG_PRESS, btn_R_onLongPress_cbk);

    /* Initialize Board Reset Button (PE3) */
    // This button is handler directly from ISR

    /* Initialize LED PB8 */
    LED_Init();

    /* Initiate CAMERA APP State Machine Manager */
    StateM_Init();
}

void CAMERA_APP_Main(void)
{
    /* Button Manager */
    Button_Main();
    /* State Machine Manager */
    StateM_Dispatch();
}

void CAM_clrScr(void)
{
    DEBUG_LOG("[APP] clrScr");
    ILI9341_FillRect(WHITE, 0, 320, 0, 240);
    HAL_Delay(10);
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    HAL_Delay(10);
}

void CAM_drawIdle(void)
{
    DEBUG_LOG("[APP] drawIdle");
    ILI9341_DrawFrame(LOGO, LOGO_size);
}

void CAM_startVideo(void)
{
    DEBUG_LOG("[APP] startVideo");
    OV7670_Start(DCMI_MODE_CONTINUOUS);
}

void CAM_stopVideo(void)
{
    DEBUG_LOG("[APP] stopVideo");
    OV7670_Stop();
}


// Function to convert and display a JPEG image
static void displayImage(const char *filename)
{
    FIL imageFile;
    if (f_open(&imageFile, filename, FA_READ) != FR_OK)
    {
        return;  // Error opening image file
    }

    /* Do LIBJPEG configuration */
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer = CAM_img_buffer_888;
    uint8_t* pixel_src_ptr;
    uint16_t* pixel_dst_ptr;
    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, &imageFile);

    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);

    if (cinfo.output_width > ILI9341_ACTIVE_WIDTH || cinfo.output_height > ILI9341_ACTIVE_HEIGHT)
    {
        jpeg_destroy_decompress(&cinfo);
        f_close(&imageFile);
        return;
    }

    while (cinfo.output_scanline < cinfo.output_height)
    {
        jpeg_read_scanlines(&cinfo, &row_pointer, 1);
        pixel_dst_ptr = (uint16_t*)CAM_img_buffer_565;
        pixel_src_ptr = CAM_img_buffer_888;

        // Convert each pixel from RGB888 to RGB565
        for (uint16_t x = 0U; x < cinfo.output_width; x += 2)
        {
            /* First pixel */
            uint8_t r = *pixel_src_ptr++;
            uint8_t g = *pixel_src_ptr++;
            uint8_t b = *pixel_src_ptr++;
            *pixel_dst_ptr++ = RGB888_TO_RGB565(r, g, b);

            /* Second pixel */
            if (x + 1 < cinfo.output_width)  // Ensure we're not out of bounds
            {
                r = *pixel_src_ptr++;
                g = *pixel_src_ptr++;
                b = *pixel_src_ptr++;
                *pixel_dst_ptr++ = RGB888_TO_RGB565(r, g, b);
            }
        }

        /* Draw line */
        ILI9341_DrawCrop(CAM_img_buffer_565, (ILI9341_ACTIVE_WIDTH * RGB565_SIZE_BYTES), 0U,
                         cinfo.output_width, cinfo.output_scanline, cinfo.output_scanline);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    f_close(&imageFile);
}


void CAM_PhotoViewer_prev(void)
{
    DEBUG_LOG("[APP] CAM_readPrevFromSD");

    if (totalImages == 0 || currentIndex <= 0)
    {
        return;  // No previous image
    }

    currentIndex--;
    displayImage(imageFiles[currentIndex]);
}

void CAM_PhotoViewer_next(void)
{
    DEBUG_LOG("[APP] CAM_readNextFromSD");

    if (totalImages == 0 || currentIndex >= totalImages - 1)
    {
        return;  // No next image
    }

    currentIndex++;
    displayImage(imageFiles[currentIndex]);
}

// Initialize the viewer and load all JPEG filenames
void CAM_PhotoViewer_Init(void)
{
    DEBUG_LOG("[APP] CAM_PhotoViewer_Init");

    DIR dir;
    FILINFO fno;
    totalImages = 0;
    currentIndex = -1;

    if (f_opendir(&dir, "/") != FR_OK)
    {
        return;  // Error opening root directory
    }

    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0])
    {
        if (!(fno.fattrib & AM_DIR) && strstr(fno.fname, IMG_PREFIX) == fno.fname)
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
        displayImage(imageFiles[currentIndex]);
    }
}

void CAM_writeToSD(void)
{
    DEBUG_LOG("[APP] writeToSD");

    FIL file;
    FILINFO fno;
    uint16_t index = 0;

    char filename[30] = { 0x0U };

    /* Do image transfering from Display, converting to JPG and storing on SD Card */
    do
    {
        if (true != CAM_IsCardPresent())
        {
            /* Unmount drive */
            f_mount(NULL, (TCHAR const*) SDPath, 1);
            FATFS_UnLinkDriver((TCHAR*) SDPath);
            FATFS_LinkDriver(&SD_Driver, (TCHAR*) SDPath);
            /* Mount drive */
            if (FR_OK != f_mount(&SDFatFS, (TCHAR const*) SDPath, 1))
            {
                // Error
                break;
            }
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
        JSAMPROW row_pointer = CAM_img_buffer_888;
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
            ILI9341_Read_GRAM(0U, cinfo.next_scanline, (ILI9341_ACTIVE_WIDTH - 1U), cinfo.next_scanline, CAM_img_buffer_888);
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
 *                            HAL CALLBACKS                                   *
 ******************************************************************************/
/* FOR TIM1_TRG_COM_TIM11_IRQHandler() ISR */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 10ms period */
    Button_Process();
}

/* FOR EXTI0_IRQHandler(), EXTI1_IRQHandler(), EXTI3_IRQHandler() ISR's */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case CAM_BTN1_Pin:
        {
            Button_HandleInterrupt(CAM_L_BTN);
            DEBUG_LOG("[L BTN] ISR");
            break;
        }
        case CAM_BTN2_Pin:
        {
            Button_HandleInterrupt(CAM_R_BTN);
            DEBUG_LOG("[R BTN] ISR");
            break;
        }
        case CAM_BTN_RST_Pin:
        {
            /* Do System Reset */
            NVIC_SystemReset();
            break;
        }
        default:
            break;
    }
}

/******************************************************************************
 *                        APPLICATION CALLBACKS                               *
 ******************************************************************************/

/* FROM ISR: This callback is invoked at the end of each ILI9341 SPI transaction */
static void CAM_SPI_TC_cbk(void)
{
    /* Resume Camera XLK signal once captured image data is drawn */
    if (OV7670_isDriverBusy())
    {
        HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_3);
    }
}


/* FROM ISR: This callback is invoked at the end of each OV7670 DCMI snapshot line reading */
static void CAM_DCMI_DrawLine_cbk(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y)
{
    ILI9341_DrawCrop(buffer, nbytes, x1, x2, y, y);
}


/* FROM ISR: This callback is invoked at the end of each OV7670 DCMI whole snapshot reading */
static void CAM_DCMI_DrawFrame_cbk(const uint8_t *buffer, uint32_t nbytes)
{
    ILI9341_DrawFrame(buffer, nbytes);
}


/* FROM MAIN THREAD: This callback is invoked on single left button press */
static void btn_L_onSinglePress_cbk(void)
{
    // Handle single press
    DEBUG_LOG("[L BTN] single");

    StateM_signal_t signal;

    switch (StateM_GetState())
    {
        case STATEM_STATE_VIDEO:
        {
            // to take a photo
            signal = STATEM_SIGNAL_L_SHORT_PRESS_3;
            break;
        }
        case STATEM_STATE_VIEWER:
        {
            // to open next photo from SD
            signal = STATEM_SIGNAL_L_SHORT_PRESS_2;
            break;
        }
        default:
        {
            signal = STATEM_SIGNAL_L_SHORT_PRESS;
            break;
        }
    }

    // Send signal for to State Machine trigger
    StateM_SetSignal(signal);
}


/* FROM MAIN THREAD: This callback is invoked on double left button press */
static void btn_L_onDoublePress_cbk(void)
{
    // Handle double press
    DEBUG_LOG("[L BTN] double");
    // Send signal to State Machine trigger
    StateM_SetSignal(STATEM_SIGNAL_L_DOUBLE_PRESS);
}


/* FROM MAIN THREAD: This callback is invoked on long left button press */
static void btn_L_onLongPress_cbk(void)
{
    // Handle long press
    DEBUG_LOG("[L BTN] long");
    // Send signal to State Machine trigger
    StateM_SetSignal(STATEM_SIGNAL_L_LONG_PRESS);
}


/* FROM MAIN THREAD: This callback is invoked on single right button press */
static void btn_R_onSinglePress_cbk(void)
{
    // Handle single press
    DEBUG_LOG("[R BTN] single");

    // Send signal to State Machine trigger
    if (STATEM_STATE_VIEWER != StateM_GetState())
    {
        StateM_SetSignal(STATEM_SIGNAL_R_SHORT_PRESS);
    }
    else
    {
        StateM_SetSignal(STATEM_SIGNAL_R_SHORT_PRESS_2);
    }

}


/* FROM MAIN THREAD: This callback is invoked on double right button press */
static void btn_R_onDoublePress_cbk(void)
{
    // Handle double press
    DEBUG_LOG("[R BTN] double");
    // Send signal to State Machine trigger
    StateM_SetSignal(STATEM_SIGNAL_R_DOUBLE_PRESS);
}


/* FROM MAIN THREAD: This callback is invoked on long right button press */
static void btn_R_onLongPress_cbk(void)
{
    // Handle long press
    DEBUG_LOG("[R BTN] long");
    // Send signal to State Machine trigger
    StateM_SetSignal(STATEM_SIGNAL_R_LONG_PRESS);
}

/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/

static bool CAM_IsCardPresent(void)
{
    bool retVal;
    DWORD free_clusters;
    FATFS *fs;

    if (f_getfree(SDPath, &free_clusters, &fs) != FR_OK)
    {
        retVal = false;
    }
    else
    {
        retVal = true;
    }

    return retVal;
}

/* Is not supported by the HW of the current ILI9341 */
/* Smooth blinking of the LED PA7 test */
//void CAM_LedBrightness_Test(void)
//{
//    static uint8_t brightness = 0U;
//    static uint8_t direction = 0U;
//
//    if (!direction)
//    {
//        brightness++;
//        direction = (brightness == 100U) ? 1U : direction;
//    }
//
//    if (direction)
//    {
//        brightness--;
//        direction = (brightness == 0U) ? 0U : direction;
//    }
//
//    PWM_SET_DUTY_CYCLE(&htim14, TIM_CHANNEL_1, brightness);
//    //HAL_Delay(10);
//}



