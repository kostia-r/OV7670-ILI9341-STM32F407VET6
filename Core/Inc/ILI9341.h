/*
 * ILI9341.h
 *
 *  Created on: Aug 5, 2024
 *      Author: k.rudenko
 */

#ifndef ILI9341_H_
#define ILI9341_H_

#include "main.h"

/* TODO: Basic colors definitions in RGB888, make as enum ?*/
#define RGB888(r,g,b)                       (((r) << 16) | ((g) << 8) | (b))
#define VIOLET                              RGB888(148,0,211)
#define INDIGO                              RGB888(75,0,130)
#define BLUE                                RGB888(0,0,255)
#define GREEN                               RGB888(0,255,0)
#define YELLOW                              RGB888(255,255,0)
#define ORANGE                              RGB888(255,127,0)
#define RED                                 RGB888(255,0,0)
#define WHITE                               RGB888(255,255,255)
#define BLACK                               RGB888(0,0,0)


//TODO: move all following things to CONFIG file and make as ENUM
/*Select pixel format */
#define	BSP_LCD_PIXEL_FMT_L8 		        1
#define	BSP_LCD_PIXEL_FMT_RGB565	        2
#define BSP_LCD_PIXEL_FMT_RGB666            3
#define	BSP_LCD_PIXEL_FMT_RGB888	        4
#define BSP_LCD_PIXEL_FMT 			        BSP_LCD_PIXEL_FMT_RGB565

/*TODO: Select orientation, make as enum? */
#define PORTRAIT                            0
#define LANDSCAPE                           1
#define BSP_LCD_ORIENTATION                 LANDSCAPE

#define ILI9341_WIDTH                       (240U)
#define ILI9341_HEIGHT                      (320U)

#if(BSP_LCD_ORIENTATION == PORTRAIT)
	#define  BSP_LCD_ACTIVE_WIDTH 			ILI9341_WIDTH
	#define  BSP_LCD_ACTIVE_HEIGHT  		ILI9341_HEIGHT
#elif(BSP_LCD_ORIENTATION == LANDSCAPE)
	#define  BSP_LCD_ACTIVE_WIDTH 			ILI9341_HEIGHT
	#define  BSP_LCD_ACTIVE_HEIGHT 			ILI9341_WIDTH
#endif

#define ILI9341_SPI_CS_HW_MANAGE            1

/* TODO: make as enum */
#define ILI9341_TC_CALLBACK                 (0U)
#define INI9341_ERR_CALLBACK                (1U)

extern void ILI9341_Init(SPI_HandleTypeDef* spi_handle);
extern void ILI9341_RegisterCallback(uint8_t cb_type, void (*fnc_ptr)(void));
extern void ILI9341_SetBackgroundColor(uint32_t rgb888);
extern void ILI9341_DrawCrop(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
extern void ILI9341_DrawFrame(const uint8_t *fb_addr, uint32_t nbytes);
extern void ILI9341_FillRect(uint32_t rgb888, uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height);
extern void *ILI9341_GetDrawBuffer1Addr(void);
extern void *ILI9341_GetDrawBuffer2Addr(void);

/* HAL callbacks for DMAx_Streamx_IRQHandler */
extern void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
extern void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#endif /* ILI9341_H_ */
