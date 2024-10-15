/*
 * LED.c
 * LED Handler
 *  Created on: Oct 15, 2024
 *      Author: k.rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "CAMERA_APP.h"
#include "LED.h"
#include "main.h"

/******************************************************************************
 *                        GLOBAL DATA PROTOTYPES                              *
 ******************************************************************************/

extern TIM_HandleTypeDef htim10;

/******************************************************************************
 *                           LOCAL DATA TYPES                                 *
 ******************************************************************************/

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

/******************************************************************************
 *                              GLOBAL FUNCTIONS                              *
 ******************************************************************************/

void LED_Init(void)
{
}

void LED_startBlinking(void)
{
    DEBUG_LOG("[APP] LED_startBlinking");
    HAL_TIM_OC_Start(&htim10, TIM_CHANNEL_1);
}

void LED_stopBlinking(void)
{
    DEBUG_LOG("[APP] LED_stopBlinking");
    HAL_TIM_OC_Stop(&htim10, TIM_CHANNEL_1);
}

void LED_onePulse(void)
{
    DEBUG_LOG("[APP] LED_onePulse");
    htim10.Instance->CR1 |= TIM_CR1_OPM;
    HAL_TIM_OC_Start(&htim10, TIM_CHANNEL_1);
    HAL_Delay(50);
    HAL_TIM_OC_Stop(&htim10, TIM_CHANNEL_1);
    htim10.Instance->CR1 &= ~TIM_CR1_OPM;
}



/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/
