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

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

/*#define PWM_SET_DUTY_CYCLE(htim, tim_ch, duty_cycle)\
    __HAL_TIM_SET_COMPARE(htim, tim_ch, ((__HAL_TIM_GET_AUTORELOAD(htim) * duty_cycle) / 100UL))
*/

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

/* Left Button Object (PC0) */
static Button_Handler* CAM_L_BTN;
/* Right Button Object (PC1) */
static Button_Handler* CAM_R_BTN;

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

/* OV7679 DCMI callbacks */
static void dcmi_DrawLine_cbk(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y);

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

void Camera_ILI9341_Init(void)
{
    DEBUG_LOG("[APP] Camera_ILI9341_Init()");

    static bool isInitDone = false;

    if (!isInitDone)
    {
        /* Initialize ILI9341 SPI Display */
        ILI9341_Init(&hspi2, ILI9341_PIXEL_FMT_RGB565);
        ILI9341_RegisterCallback(ILI9341_ERR_CBK, Error_Handler);
        ILI9341_SetBackgroundColor(BLACK);
        /* Initialize backlight brightness PWM (PA7) - not supported by the current HW */
        //HAL_TIM_OC_Start(&htim14, TIM_CHANNEL_1);
        isInitDone = true;
    }
}

void Camera_OV7670_Init(void)
{
    DEBUG_LOG("[APP] Camera_OV7670_Init()");

    static bool isInitDone = false;

    if (!isInitDone)
    {
        /* Initialize OV7670 DCMI Camera */
        OV7670_Init(&hdcmi, &hi2c2, &htim5, TIM_CHANNEL_3);
        OV7670_RegisterCallback(OV7670_DRAWLINE_CBK, (OV7670_FncPtr_t) dcmi_DrawLine_cbk);
        isInitDone = true;
    }
}

void Camera_Buttons_Init(void)
{
    DEBUG_LOG("[APP] Camera_Buttons_Init()");

    static bool isInitDone = false;

    if (!isInitDone)
    {
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
        // This button is managed directly from ISR
        isInitDone = true;
    }
}

void Camera_ILI9341_clrScr(void)
{
    DEBUG_LOG("[APP] clrScr");
    ILI9341_FillRect(WHITE, 0, 320, 0, 240);
    HAL_Delay(10);
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    HAL_Delay(10);
}

void Camera_ILI9341_IdleScr(void)
{
    DEBUG_LOG("[APP] drawIdle");
    ILI9341_DrawFrame(LOGO, LOGO_size);
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

/* FROM ISR: This callback is invoked at the end of each OV7670 DCMI snapshot line reading */
static void dcmi_DrawLine_cbk(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y)
{
    ILI9341_DrawCrop(buffer, nbytes, x1, x2, y, y);
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
