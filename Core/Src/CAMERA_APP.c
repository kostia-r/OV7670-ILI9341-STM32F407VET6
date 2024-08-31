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

typedef enum CAMERA_APP_states
{
    CAMERA_APP_NO_STATE,
    CAMERA_APP_IDLE_STATE,
    CAMERA_APP_VIDEOFINDER_STATE,
    CAMERA_APP_PHOTO_STATE,
    CAMERA_APP_SD_WRITE_STATE,
}CAMERA_APP_states_t;

static struct CAMERA_APP_str
{
        CAMERA_APP_states_t requestedState;
        CAMERA_APP_states_t State;
}CAMERA_APP;
/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

/* Button PA0 Object */
static Button_Handler* btn_PA0;

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void CAM_MakeSnaphot(void);
static void CAM_StartStream(void);
static void CAM_StopStream(void);
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

/* State machine handling */
static void CAM_StateM(void);
static void CAM_SetState(CAMERA_APP_states_t state);
static CAMERA_APP_states_t CAM_GetState(void);

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

    /* Initialize button PA0 */
    btn_PA0 = Button_Init(K_UP_GPIO_Port, K_UP_Pin, GPIO_PIN_SET, &htim11);
    Button_RegisterCallback(btn_PA0, BUTTON_EVENT_SINGLE_PRESS, CAM_onSinglePress_cbk);
    Button_RegisterCallback(btn_PA0, BUTTON_EVENT_DOUBLE_PRESS, CAM_onDoublePress_cbk);
    Button_RegisterCallback(btn_PA0, BUTTON_EVENT_LONG_PRESS, CAM_onLongPress_cbk);

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

    /* CAMERA APP State Machine Manager */
    CAM_StateM();
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
    if (GPIO_Pin == K_UP_Pin)
    {
        Button_HandleInterrupt(btn_PA0);
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
    HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_3);
}


/* FROM ISR: This callback is invoked at the end of each OV7670 DCMI snapshot line reading */
static void CAM_DCMI_DrawLine_cbk(const uint8_t *buffer,
        uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y)
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
    DEBUG_LOG("[BTN] signle");

    if (CAM_GetState() != CAMERA_APP_VIDEOFINDER_STATE)
    {
        /* Request "ON" Videofinder Mode */
        CAM_SetState(CAMERA_APP_VIDEOFINDER_STATE);
    }
    else
    {
        /* Request "OFF" Videofinder Mode */
        CAM_SetState(CAMERA_APP_IDLE_STATE);
    }
}


/* FROM MAIN THREAD: This callback is invoked on double button press */
static void CAM_onDoublePress_cbk(void)
{
    // Handle double press
    DEBUG_LOG("[BTN] double");
    /* Request Photomaker Mode */
    CAM_SetState(CAMERA_APP_PHOTO_STATE);
}


/* FROM MAIN THREAD: This callback is invoked on long button press */
static void CAM_onLongPress_cbk(void)
{
    // Handle long press
    DEBUG_LOG("[BTN] long");
    /* Request Save To SD Card Mode */
    CAM_SetState(CAMERA_APP_SD_WRITE_STATE);
}


/******************************************************************************
 *                            LOCAL FUNCTIONS                                 *
 ******************************************************************************/

static void CAM_MakeSnaphot(void)
{
    DEBUG_LOG("[APP] MAKE SCR");
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    //TODO replace by millis
    HAL_Delay(50);
    OV7670_Start(DCMI_MODE_SNAPSHOT);
}

static void CAM_StartStream(void)
{
    DEBUG_LOG("[APP] START STREAM");
    OV7670_Start(DCMI_MODE_CONTINUOUS);
}

static void CAM_StopStream(void)
{
    DEBUG_LOG("[APP] STOP STREAM");
    OV7670_Stop();
    //TODO replace by millis
    HAL_Delay(50);
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    //TODO replace by millis
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

static void CAM_StateM(void)
{

}

static void CAM_SetState(CAMERA_APP_states_t state)
{
    CAMERA_APP.requestedState = state;

    /* For debug purposes */
    switch (CAMERA_APP.requestedState)
    {
        case CAMERA_APP_IDLE_STATE:
            DEBUG_LOG("[APP] IDLE REQUEST");
            break;
        case CAMERA_APP_PHOTO_STATE:
            DEBUG_LOG("[APP] PHOTO REQUEST");
            break;
        case CAMERA_APP_SD_WRITE_STATE:
            DEBUG_LOG("[APP] SD_WRITE REQUEST");
            break;
        case CAMERA_APP_VIDEOFINDER_STATE:
            DEBUG_LOG("[APP] VIDEOFINDER REQUEST");
            break;
        default:
            DEBUG_LOG("[APP] WRONG STATE REQUEST");
            break;
    }
}

static CAMERA_APP_states_t CAM_GetState(void)
{
    return CAMERA_APP.State;
}
