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

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define PWM_SET_DUTY_CYCLE(htim, tim_ch, duty_cycle)\
    __HAL_TIM_SET_COMPARE(htim, tim_ch, ((__HAL_TIM_GET_AUTORELOAD(htim) * duty_cycle) / 100UL))

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim5;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim14;

/* LOGO BMP */
/* Online converter image-to-C-array https://lvgl.io/tools/imageconverter */
extern const uint8_t LOGO[];
extern const uint32_t LOGO_size;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

/* Button PC0 Object */
static Button_Handler* btn_PС0;

static uint8_t img_buffer[RGB888_SIZE_BYTES * ILI9341_ACTIVE_WIDTH];

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

//static void CAM_LedBrightness_Test(void);

/* ILI9341 SPI transmittion complete callback */
static void CAM_SPI_TC_cbk(void);

/* OV7679 DCMI callbacks */
static void CAM_DCMI_DrawLine_cbk(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y);
static void CAM_DCMI_DrawFrame_cbk(const uint8_t *buffer, uint32_t nbytes);

/* Button callbacks */
static void CAM_onSinglePress_cbk(void);
static void CAM_onDoublePress_cbk(void);
static void CAM_onLongPress_cbk(void);

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

    /* Initialize button PС0 */
    btn_PС0 = Button_Init(CAM_BTN1_GPIO_Port, CAM_BTN1_Pin, GPIO_PIN_SET, &htim11);
    Button_RegisterCallback(btn_PС0, BUTTON_EVENT_SINGLE_PRESS, CAM_onSinglePress_cbk);
    Button_RegisterCallback(btn_PС0, BUTTON_EVENT_DOUBLE_PRESS, CAM_onDoublePress_cbk);
    Button_RegisterCallback(btn_PС0, BUTTON_EVENT_LONG_PRESS, CAM_onLongPress_cbk);

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

void CAM_takePhoto(void)
{
    DEBUG_LOG("[APP] takePhoto");
    // just do nothing to avoid desynchronization
    //OV7670_Start(DCMI_MODE_SNAPSHOT);
}

void CAM_writeToSD(void)
{
    DEBUG_LOG("[APP] writeToSD");

    FIL file;
    FRESULT res;
    FILINFO fno;
    uint16_t index = 0;

    char filename[30] = { 0x0U };
    char base_name[] = "img";
    char extension[] = "jpg";

    /* Do image transfering from Display, converting to JPG and storing on SD Card */
    do
    {
        /* Mount file system */
        res = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);

        if (res != FR_OK)
        {
            // Error
            break;
        }

        /* Generate unique file name */
        do
        {
            sprintf(filename, "%s_%d.%s", base_name, index, extension);
            index++;
        }
        while (f_stat(filename, &fno) == FR_OK); // FR_OK returns if file exists

        /* Create a new file with the unique name */
        res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);

        if (res != FR_OK)
        {
            // Error
            break;
        }

        /* Do LIBJPEG configuration */
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        JSAMPROW row_pointer = img_buffer;
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
            ILI9341_Read_GRAM(0U, cinfo.next_scanline, (ILI9341_ACTIVE_WIDTH - 1U),
                                  cinfo.next_scanline, img_buffer);
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
/* FROM ISR */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 10ms period */
    Button_Process();
}

/* FROM ISR */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == CAM_BTN1_Pin)
    {
        Button_HandleInterrupt(btn_PС0);
        DEBUG_LOG("[BTN] ISR");
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


/* FROM MAIN THREAD: This callback is invoked on single button press */
static void CAM_onSinglePress_cbk(void)
{
    // Handle single press
    DEBUG_LOG("[BTN] single");
    // Send signal to State Machine trigger
    StateM_SetSignal(STATEM_SIGNAL_SHORT_PRESS);
}


/* FROM MAIN THREAD: This callback is invoked on double button press */
static void CAM_onDoublePress_cbk(void)
{
    // Handle double press
    DEBUG_LOG("[BTN] double");
    // Send signal to State Machine trigger
    StateM_SetSignal(STATEM_SIGNAL_DOUBLE_PRESS);
}


/* FROM MAIN THREAD: This callback is invoked on long button press */
static void CAM_onLongPress_cbk(void)
{
    // Handle long press
    DEBUG_LOG("[BTN] long");
    // Send signal to State Machine trigger
    StateM_SetSignal(STATEM_SIGNAL_LONG_PRESS);
}


/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/

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



