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

extern void CAMERA_APP_Init(void);
extern void CAMERA_APP_Main(void);

extern void CAM_clrScr(void);
extern void CAM_drawIdle(void);
extern void CAM_startVideo(void);
extern void CAM_stopVideo(void);
extern void CAM_takePhoto(void);
extern void CAM_LED_startBlinking(void);
extern void CAM_writeToSD(void);
extern void CAM_LED_stopBlinking(void);

/******************************************************************************
 *                              HAL CALLBACKS                                 *
 ******************************************************************************/

extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* CAMERA_APP_H_ */
