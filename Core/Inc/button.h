/*
 * Button.h
 * Button handler
 * Allows to handle single, double, or long presses.
 * Based on Opaque pattern.
 * Uses the STM32 HAL library.
 * Created on: Aug 15, 2024
 *     Author: K.Rudenko
 */

#ifndef BUTTON_H
#define BUTTON_H

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "main.h"

/******************************************************************************
 *                            CONFIGURATION MACRO                             *
 ******************************************************************************/

#define BUTTON_SHORT_PRESS_TIME_MS                                       (200U)
#define BUTTON_LONG_PRESS_TIME_MS                                        (600U)
#define BUTTON_DOUBLE_PRESS_TIME_MS                                      (300U)
#define BUTTON_DEBOUNCE_TIME_MS                                           (50U)
#define BUTTON_MAX_INSTANCES                                               (1U)

/******************************************************************************
 *                           GLOBAL DATA TYPES                                *
 ******************************************************************************/

typedef enum
{
    BUTTON_EVENT_NONE,
    BUTTON_EVENT_SINGLE_PRESS,
    BUTTON_EVENT_DOUBLE_PRESS,
    BUTTON_EVENT_LONG_PRESS
} ButtonEvent;

typedef void* Button_Handler;
typedef void (*Button_FncPtr_t)(void);

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

/* This function is used to initialize a button instance
 * (except for initializing the GPIO, and the ISR that is done by the CubeMX)
 * */
extern Button_Handler Button_Init(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin,
        GPIO_PinState active_level, TIM_HandleTypeDef *htim);

extern void Button_RegisterCallback(Button_Handler handle, ButtonEvent cb_type, Button_FncPtr_t fnc_ptr);

/* This function is used to handle the interrupt of this button */
/* NOTE: This interrupt handler shall be invoked on falling AND rising edges! */
extern void Button_HandleInterrupt(Button_Handler handle);

/* This function is used to analyze buttons inside the Timer ISR each 10ms */
extern void Button_Process(void);

/* This function is used to asynchronously call registered users callbacks
 * in the thread mode */
extern void Button_Main(void);

#endif /* BUTTON_H */
