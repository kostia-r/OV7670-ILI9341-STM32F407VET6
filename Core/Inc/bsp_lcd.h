/*
 * bsp_lcd.h
 *
 *  Created on: May 12, 2024
 *      Author: ashen
 */

#ifndef BSP_LCD_H_
#define BSP_LCD_H_


#include "main.h"
#include "ili9341_reg.h"

#define BSP_LCD_WIDTH  		240
#define BSP_LCD_HEIGHT 		320

#define RGB888(r,g,b)  (((r) << 16) | ((g) << 8) | (b))

#define VIOLET      RGB888(148,0,211)
#define INDIGO      RGB888(75,0,130)
#define BLUE        RGB888(0,0,255)
#define GREEN       RGB888(0,255,0)
#define YELLOW      RGB888(255,255,0)
#define ORANGE      RGB888(255,127,0)
#define RED         RGB888(255,0,0)
#define WHITE       RGB888(255,255,255)
#define BLACK       RGB888(0,0,0)


#define BSP_LCD_HSW 		10
#define BSP_LCD_HBP			20
#define BSP_LCD_HFP			10
#define BSP_LCD_VSW			2
#define BSP_LCD_VBP			2
#define BSP_LCD_VFP			4


/*Select pixel format */
#define	BSP_LCD_PIXEL_FMT_L8 		1
#define	BSP_LCD_PIXEL_FMT_RGB565	2
#define BSP_LCD_PIXEL_FMT_RGB666    3
#define	BSP_LCD_PIXEL_FMT_RGB888	4
#define BSP_LCD_PIXEL_FMT 			BSP_LCD_PIXEL_FMT_RGB565


/*Select orientation*/
#define PORTRAIT  0
#define LANDSCAPE 1
#define BSP_LCD_ORIENTATION   PORTRAIT

#if(BSP_LCD_ORIENTATION == PORTRAIT)
	#define  BSP_LCD_ACTIVE_WIDTH 			BSP_LCD_WIDTH
	#define  BSP_LCD_ACTIVE_HEIGHT  		BSP_LCD_HEIGHT
#elif(BSP_LCD_ORIENTATION == LANDSCAPE)
	#define  BSP_LCD_ACTIVE_WIDTH 			BSP_LCD_HEIGHT
	#define  BSP_LCD_ACTIVE_HEIGHT 			BSP_LCD_WIDTH
#endif


#define AUTO				 1
#define MANUAL				 0
#define BSP_LCD_CS_MANAGE    AUTO

#define USE_DMA 1

 typedef struct{
 	uint16_t x1;
 	uint16_t x2;
 	uint16_t y1;
 	uint16_t y2;
 }lcd_area_t;

 struct bsp_lcd;

 typedef void (*bsp_lcd_dma_cplt_cb_t)(struct bsp_lcd*);
 typedef void (*bsp_lcd_dma_err_cb_t)(struct bsp_lcd*);

 typedef struct{
 	uint8_t orientation;
 	uint8_t pixel_format;
 	uint8_t * draw_buffer1;
 	uint8_t * draw_buffer2;
 	uint32_t write_length;
 	uint8_t *buff_to_draw;
 	uint8_t *buff_to_flush;
 	lcd_area_t area;
 	bsp_lcd_dma_cplt_cb_t dma_cplt_cb;
 	bsp_lcd_dma_err_cb_t dma_err_cb;
 }bsp_lcd_t;




void lcd_set_orientation(uint8_t orientation);
void bsp_lcd_set_background_color(uint32_t rgb888);
void bsp_lcd_fill_rect(uint32_t rgb888, uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height);
void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1 , uint16_t y2);
void bsp_lcd_send_cmd_mem_write(void);
uint16_t bsp_lcd_convert_rgb888_to_rgb565(uint32_t rgb888);
void *bsp_lcd_get_draw_buffer1_addr(void);
void *bsp_lcd_get_draw_buffer2_addr(void);


extern void ILI9341_Init();
extern void ILI9341_DrawCrop(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
extern void ILI9341_DrawFrame(const uint8_t *fb_addr, uint32_t nbytes);
extern void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
extern void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#endif /* BSP_LCD_H_ */
