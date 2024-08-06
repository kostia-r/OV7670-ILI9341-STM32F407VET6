/*
 * OV7670.h
 * OV7670 DCMI DMA Driver
 * Created on: Aug 5, 2024
 *     Author: k.rudenko
 */

#ifndef OV7670_H_
#define OV7670_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "main.h"

/******************************************************************************
 *                            CONFIGURATION MACRO                             *
 ******************************************************************************/

#define OV7670_WIDTH                             (320U)
#define OV7670_HEIGHT                            (240U)

#define OV7670_STREAM_MODE_BY_FRAME              0
#define OV7670_STREAM_MODE_BY_LINE               1
#define OV7670_STREAM_MODE                       OV7670_STREAM_MODE_BY_LINE

/* Delay API */
#define OV7670_DELAY(ms)                         HAL_Delay(ms)

/* GPIO pin mapping */
#define OV7670_GPIO_PORT_PWDN                    CAM_PWDN_GPIO_Port
#define OV7670_GPIO_PIN_PWDN                     CAM_PWDN_Pin
#define OV7670_GPIO_PORT_RET                     CAM_RET_GPIO_Port
#define OV7670_GPIO_PIN_RET                      CAM_RET_Pin
/* Note: XLK pin shall be connected to the corresponding timer OCU pin */
/* D0..D7, VS, HS, PLK shall be connected to 8-bit with External Sync DCMI */
/* SCCB SCL, SDA - to I2C */
/* For GPIO, ,DMA, Timer configuration - please see in CubeMX project */
/* Example for OV7670 <-> STM32F407VET6 connections:
 * SCL  -> PB10 (I2C2)
 * SDA  -> PB11 (I2C2)
 * VS   -> PB7  (DCMI)
 * HS   -> PA4  (DCMI)
 * PLK  -> PA6  (DCMI)
 * XLK  -> PC2  (TIM5 OCU CH3) 10.5..42 MHz
 * D7   -> PB9  (DCMI)
 * D6   -> PB8  (DCMI)
 * D5   -> PB6  (DCMI)
 * D4   -> PC11 (DCMI)
 * D3   -> PC9  (DCMI)
 * D2   -> PC8  (DCMI)
 * D1   -> PC7  (DCMI)
 * D0   -> PC6  (DCMI)
 * RET  -> PD11 (GPIO)
 * PWDN -> PD12 (GPIO)
 */
/******************************************************************************
 *                           GLOBAL DATA TYPES                                *
 ******************************************************************************/
typedef enum
{
    OV7670_DRAWLINE_CALLBACK,
    OV7670_DRAWFRAME_CALLBACK,
} OV7670_CB_t;

typedef void (*OV7670_FncPtr_t)(void);

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern void OV7670_Init(DCMI_HandleTypeDef *hdcmi, I2C_HandleTypeDef *hi2c,
        TIM_HandleTypeDef* htim, uint32_t tim_ch);
extern void OV7670_RegisterCallback(OV7670_CB_t cb_type, OV7670_FncPtr_t fnc_ptr);
extern void OV7670_Start(uint32_t capMode);
extern void OV7670_Stop(void);

/******************************************************************************
 *                  HAL callbacks for DCMI_IRQHandler                         *
 ******************************************************************************/

#if (OV7670_STREAM_MODE == OV7670_SRTEAM_MODE_BY_FRAME)
extern void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi);
extern void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi);
#else  /* (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE) */
extern void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi);
#endif /* (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE) */

#endif /* OV7670_H_ */
