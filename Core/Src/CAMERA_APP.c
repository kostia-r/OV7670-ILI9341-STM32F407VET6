/*
 * CAMERA_APP.c
 *
 *  Created on: Aug 15, 2024
 *      Author: ashen
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "CAMERA_APP.h"
#include "ILI9341.h"
#include "OV7670.h"
#include "Button.h"

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
 *                           LOCAL DATA TYPES                                 *
 ******************************************************************************/

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

/* Button PA0 Object */
Button_Handler* btn_PA0;

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void CAM_MakeSnaphot(void);
static void CAM_StartStream(void);
static void CAM_StopStream(void);
//static void CAM_LedBrightness_Test(void);
static void CAM_SPI_TC_Callback(void);
static void CAM_DCMI_DrawLine_Callback(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y);
static void CAM_DCMI_DrawFrame_Callback(const uint8_t *buffer, uint32_t nbytes);
static void CAM_onSinglePress_Callback(void);
static void CAM_onDoublePress_Callback(void);
static void CAM_onLongPress_Callback(void);

/******************************************************************************
 *                            GLOBAL FUNCTIONS                                *
 ******************************************************************************/

void CAMERA_APP_Init(void)
{
    DEBUG_LOG(" --- CAMERA APP on STM32F407VET6 ---");

    /* Initialize ILI9341 SPI Display */
    ILI9341_Init(&hspi2, ILI9341_PIXEL_FMT_RGB565);
    ILI9341_RegisterCallback(ILI9341_TC_CALLBACK, CAM_SPI_TC_Callback);
    ILI9341_RegisterCallback(ILI9341_ERR_CALLBACK, Error_Handler);
    ILI9341_SetBackgroundColor(BLACK);

    /* Initialize OV7670 DCMI Camera */
    OV7670_Init(&hdcmi, &hi2c2, &htim5, TIM_CHANNEL_3);
    OV7670_RegisterCallback(OV7670_DRAWLINE_CALLBACK,
            (OV7670_FncPtr_t) CAM_DCMI_DrawLine_Callback);
    OV7670_RegisterCallback(OV7670_DRAWFRAME_CALLBACK,
            (OV7670_FncPtr_t) CAM_DCMI_DrawFrame_Callback);

    /* Initialize backlight brightness PWM (PA7) */
    //HAL_TIM_OC_Start(&htim14, TIM_CHANNEL_1);

    /* Initialize button PA0 */
    btn_PA0 = Button_Init(K_UP_GPIO_Port, K_UP_Pin, GPIO_PIN_SET,
            CAM_onSinglePress_Callback, CAM_onDoublePress_Callback,
            CAM_onLongPress_Callback, &htim11);

    /* Draw LOGO test slide */
    ILI9341_DrawFrame(LOGO, LOGO_size);
    HAL_Delay(5000);

    /* Start video stream */
    CAM_StartStream();
    //HAL_Delay(10000);
    //APP_StopStream();
}

void CAMERA_APP_Main(void)
{
    //HAL_Delay(500);
    //APP_MakeSnaphot();

    /* Handling buton callbacks */
    Button_Main();
}

/***************************** APPLICATION LOGIC ******************************/
static void CAM_MakeSnaphot(void)
{
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    HAL_Delay(50);
    OV7670_Start(DCMI_MODE_SNAPSHOT);
}

static void CAM_StartStream(void)
{
    OV7670_Start(DCMI_MODE_CONTINUOUS);
}

static void CAM_StopStream(void)
{
    OV7670_Stop();
    HAL_Delay(50);
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    HAL_Delay(50);
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


/******************************************************************************
 *                        APPLICATION CALLBACKS                               *
 ******************************************************************************/

/* This callback is invoked at the end of each ILI9341 SPI transaction */
static void CAM_SPI_TC_Callback(void)
{
    /* Resume Camera XLK signal once captured image data is drawn */
    HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_3);
}


/* This callback is invoked at the end of each OV7670 DCMI snapshot line reading */
static void CAM_DCMI_DrawLine_Callback(const uint8_t *buffer,
        uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y)
{
    ILI9341_DrawCrop(buffer, nbytes, x1, x2, y, y);
}


/* This callback is invoked at the end of each OV7670 DCMI whole snapshot reading */
static void CAM_DCMI_DrawFrame_Callback(const uint8_t *buffer, uint32_t nbytes)
{
    ILI9341_DrawFrame(buffer, nbytes);
}


/* This callback is invoked on single button press */
static void CAM_onSinglePress_Callback(void)
{
    // Handle single press
    DEBUG_LOG("[BTN] signle");
}


/* This callback is invoked on double button press */
static void CAM_onDoublePress_Callback(void)
{
    // Handle double press
    DEBUG_LOG("[BTN] double");
}


/* This callback is invoked on long button press */
static void CAM_onLongPress_Callback(void)
{
    // Handle long press
    DEBUG_LOG("[BTN] long");
}


/******************************************************************************
 *                            HAL CALLBACKS                                   *
 ******************************************************************************/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 10ms period */
    Button_Process();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == K_UP_Pin)
    {
        Button_HandleInterrupt(btn_PA0);
        DEBUG_LOG("[BTN] ISR");
    }
}

/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/
