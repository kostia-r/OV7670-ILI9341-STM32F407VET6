/*
 * CAMERA_APP.h
 * Camera Application main logic
 *  Created on: Aug 15, 2024
 *      Author: K.Rudenko
 */

#ifndef CAMERA_APP_H_
#define CAMERA_APP_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "main.h"

/******************************************************************************
 *                            CONFIGURATION MACRO                             *
 ******************************************************************************/

/******************************************************************************
 *                           GLOBAL DATA TYPES                                *
 ******************************************************************************/

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/
extern void Camera_ILI9341_Init(void);
extern void Camera_OV7670_Init(void);
extern void Camera_Buttons_Init(void);
extern void Camera_ILI9341_IdleScr(void);
extern void Camera_ILI9341_clrScr(void);

/******************************************************************************
 *                              HAL CALLBACKS                                 *
 ******************************************************************************/

extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* CAMERA_APP_H_ */
