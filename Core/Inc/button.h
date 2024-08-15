/*
 * Button.h
 * Button handler
 * Allows to handle single, double, or long presses.
 * Uses the STM32 HAL library.
 * Created on: Aug 15, 2024
 *     Author: k.rudenko
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

typedef void* Button_Handler;
typedef void (*Button_FncPtr_t)(void);

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

/* This function is used to initialize a button instance
 * (except for initializing the GPIO, and the ISR that is done by the CubeMX)
 * */
extern Button_Handler Button_Init(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin,
        GPIO_PinState active_level, Button_FncPtr_t single_press_cb,
        Button_FncPtr_t double_press_cb, Button_FncPtr_t long_press_cb,
        TIM_HandleTypeDef *htim);

/* This function is used to handle the interrupt of this button */
/* NOTE: This interrupt handler shall be invoked on falling AND rising edges! */
extern void Button_HandleInterrupt(Button_Handler handle);

/* This function is used to analyze buttons inside the Timer ISR each 10ms */
extern void Button_Process(void);

/* This function is used to asynchronously call registered users callbacks
 * in the thread mode */
extern void Button_Main(void);

#endif /* BUTTON_H */
