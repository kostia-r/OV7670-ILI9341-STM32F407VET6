/*
 * ILI9341.h
 * ILI9341 SPI DMA Driver
 * Created on: Aug 5, 2024
 *     Author: k.rudenko
 */

#ifndef ILI9341_H_
#define ILI9341_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "main.h"

/******************************************************************************
 *                               GLOBAL MACRO                                 *
 ******************************************************************************/

/* Convert r,b,g to uint32_t RBG888 */
#define RGB888(r,g,b)                       (((r) << 16) | ((g) << 8) | (b))

/******************************************************************************
 *                            CONFIGURATION MACRO                             *
 ******************************************************************************/

#define ILI9341_PORTRAIT                    0
#define ILI9341_LANDSCAPE                   1
#define ILI9341_ORIENTATION                 ILI9341_LANDSCAPE

#define ILI9341_WIDTH                       (240U)
#define ILI9341_HEIGHT                      (320U)

/* Delay API */
#define ILI9341_DELAY(ms)                   HAL_Delay(ms)

/* Set as 1 if SPI CS pin is managed by hardware */
#define ILI9341_SPI_CS_HW_MANAGE            1

/* GPIO configuration */
#define ILI9341_GPIO_PORT_RESX              LCD_RESX_GPIO_Port
#define ILI9341_GPIO_PIN_RESX               LCD_RESX_Pin

#if (ILI9341_SPI_CS_HW_MANAGE == 1)
#define ILI9341_GPIO_PORT_CSX
#define ILI9341_GPIO_PIN_CSX
#else
#define ILI9341_GPIO_PORT_CSX               LCD_CSX_Port
#define ILI9341_GPIO_PIN_CSX                LCD_CSX_Pin
#endif

#define ILI9341_GPIO_PORT_DCX               LCD_DCX_GPIO_Port
#define ILI9341_GPIO_PIN_DCX                LCD_DCX_Pin

/* Example for ILI9341 <-> STM32F407VET6 connections:
 * DCX/DC     -> PD9  (GPIO)
 * RESX/RESET -> PD10 (GPIO)
 * LED        -> PA7  (TIM14 CH1 PWM)
 * CSX/CS     -> PB12 (SPI2 NSS)
 * SCL/SCK    -> PB13 (SPI2 SCK)
 * SDI        -> PB15 (SPI2 MOSI)
 * SDO        -> PC2  (SPI2 MISO)
 * GND        -> GND
 * VCC        -> 5V
 */
/******************************************************************************
 *                           GLOBAL DATA TYPES                                *
 ******************************************************************************/

/* Pixel format */
typedef enum
{
    ILI9341_PIXEL_FMT_L8     = 1U,
    ILI9341_PIXEL_FMT_RGB565 = 2U,
    ILI9341_PIXEL_FMT_RGB666 = 3U,
    ILI9341_PIXEL_FMT_RGB888 = 4U,
}ILI9341_PixFormat_t;

/* Basic colors definitions in RGB888 */
enum
{
    VIOLET = RGB888(148, 0, 211),
    INDIGO = RGB888(75, 0, 130),
    BLUE   = RGB888(0, 0, 255),
    GREEN  = RGB888(0, 255, 0),
    YELLOW = RGB888(255, 255, 0),
    ORANGE = RGB888(255, 127, 0),
    RED    = RGB888(255, 0, 0),
    WHITE  = RGB888(255, 255, 255),
    BLACK  = RGB888(0, 0, 0),
};

/* Callback enumeration */
typedef enum
{
    ILI9341_TC_CBK,
    ILI9341_ERR_CBK,
} ILI9341_CB_t;

/* Callback pointer type */
typedef void (*ILI9341_FncPtr_t)(void);

/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern void ILI9341_Init(SPI_HandleTypeDef* spi_handle, ILI9341_PixFormat_t pixFormat);
extern void ILI9341_RegisterCallback(ILI9341_CB_t cb_type, ILI9341_FncPtr_t fnc_ptr);
extern void ILI9341_SetBackgroundColor(uint32_t rgb888);
extern void ILI9341_DrawCrop(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
extern void ILI9341_DrawFrame(const uint8_t *fb_addr, uint32_t nbytes);
extern void ILI9341_FillRect(uint32_t rgb888, uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height);
extern void *ILI9341_GetDrawBuffer1Addr(void);
extern void *ILI9341_GetDrawBuffer2Addr(void);

/******************************************************************************
 *               HAL callbacks for DMAx_Streamx_IRQHandler                    *
 ******************************************************************************/

extern void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
extern void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#endif /* ILI9341_H_ */
