/*
 * bsp_lcd.c
 *
 *  Created on: May 12, 2024
 *      Author: ashen
 */

#include "main.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_dma.h"
#include "ili9341_reg.h"
#include "reg_util.h"
#include "bsp_lcd.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim5;

uint8_t volatile dataKOS[1000];
uint8_t volatile dataKOS_2[1000];
uint8_t* buffersPtr[2] = {(uint8_t*)dataKOS, (uint8_t*)dataKOS_2};
uint8_t volatile cnt = 0;

volatile uint32_t dma_spi_cnt;
volatile uint32_t scr_address;
volatile uint32_t n_chunks;
volatile uint32_t g_nitems;

/* Define all the LCD signals */
#define SPI                      SPI2

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

bsp_lcd_t lcd_handle;

bsp_lcd_t *hlcd = &lcd_handle;

/*private helper functions*/
void lcd_reset(void);
void lcd_config(void);
void lcd_write_cmd(uint8_t cmd);
void lcd_write_data(uint8_t *buffer,uint32_t len);
void lcd_set_orientation(uint8_t orientation);
void lcd_set_display_area(lcd_area_t *area);
void lcd_buffer_init(bsp_lcd_t *lcd);
void lcd_dma_init(bsp_lcd_t *lcd);
void lcd_flush(bsp_lcd_t *lcd);
void lcd_write_dma(uint32_t src_addr, uint32_t nbytes);
uint32_t get_total_bytes(bsp_lcd_t *lcd,uint32_t w , uint32_t h);
void make_area(lcd_area_t *area,uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height);
uint32_t bytes_to_pixels(uint32_t nbytes, uint8_t pixel_format);
uint32_t copy_to_draw_buffer( bsp_lcd_t *lcd,uint32_t nbytes,uint32_t rgb888);
uint32_t pixels_to_bytes(uint32_t pixels, uint8_t pixel_format);
static uint8_t is_lcd_write_allowed(bsp_lcd_t *lcd);
void initialize_lcd_write_dma(uint32_t src_addr, uint32_t dst_addr);
void initialize_memory_write_dma(uint32_t src_addr, uint32_t dst_addr);
uint16_t convert_rgb888_to_rgb565(uint32_t rgb888);

#define DB_SIZE 					(10UL * 1024UL)
uint8_t bsp_db[DB_SIZE];
uint8_t bsp_wb[DB_SIZE];

enum {FALSE,TRUE};

#ifndef UNUSED
#define UNUSED(x)    (void)x
#endif

#define __enable_dma(stream) 			REG_SET_BIT(stream->CR,DMA_SxCR_EN_Pos);
#define __disable_dma(stream)			REG_CLR_BIT(stream->CR,DMA_SxCR_EN_Pos);

#define HIGH_16(x)     					((((uint16_t)x) >> 8U) & 0xFFU)
#define LOW_16(x)      					((((uint16_t)x) >> 0U) & 0xFFU)

#define LCD_RESX_HIGH()				SET_BIT(LCD_RESX_GPIO_Port->ODR,LCD_RESX_Pin)
#define LCD_RESX_LOW()				CLEAR_BIT(LCD_RESX_GPIO_Port->ODR,LCD_RESX_Pin)

#if (BSP_LCD_CS_MANAGE == AUTO)
#define LCD_CSX_HIGH()
#define LCD_CSX_LOW()
#else
#define LCD_CSX_HIGH()				SET_BIT(LCD_CSX_GPIO_Port->ODR,LCD_CSX_Pin)
#define LCD_CSX_LOW()				CLEAR_BIT(LCD_CSX_GPIO_Port->ODR,LCD_CSX_Pin)
#endif

#define LCD_DCX_HIGH()				SET_BIT(LCD_DCX_GPIO_Port->ODR,LCD_DCX_Pin)
#define LCD_DCX_LOW()				CLEAR_BIT(LCD_DCX_GPIO_Port->ODR,LCD_DCX_Pin)



void bsp_lcd_init(void)
{
    //TODO: make orientation as runtime option
	__HAL_SPI_ENABLE(&hspi2);
	lcd_handle.orientation = BSP_LCD_ORIENTATION;
	lcd_handle.pixel_format = BSP_LCD_PIXEL_FMT;
	lcd_reset();
	lcd_config();
	hlcd->area.x1 = 0;
	hlcd->area.x2 = BSP_LCD_ACTIVE_WIDTH-1;
	hlcd->area.y1 = 0;
	hlcd->area.y2 = BSP_LCD_ACTIVE_HEIGHT-1;
	lcd_set_display_area(&hlcd->area);
	lcd_set_orientation(hlcd->orientation);
	lcd_buffer_init(hlcd);
	lcd_dma_init(hlcd);
}

void bsp_lcd_set_orientation(int orientation)
{
	lcd_set_orientation(orientation);
}

void bsp_lcd_send_cmd_mem_write(void)
{
	lcd_write_cmd(ILI9341_GRAM);
}



void *bsp_lcd_get_draw_buffer1_addr(void)
{
    return (void*)hlcd->draw_buffer1;
}
void *bsp_lcd_get_draw_buffer2_addr(void)
{
	return (void*)hlcd->draw_buffer2;
}

void bsp_lcd_write(uint8_t *buffer, uint32_t nbytes)
{
//	uint16_t *buff_ptr;

	__HAL_SPI_DISABLE(&hspi2);
	LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_16BIT);
	__HAL_SPI_ENABLE(&hspi2);

	LCD_CSX_LOW();

	HAL_SPI_Transmit(&hspi2, buffer, nbytes/2, HAL_MAX_DELAY);

//	buff_ptr = (uint16_t*)buffer;
//	while(nbytes){
//		while(!REG_READ_BIT(SPI->SR,SPI_SR_TXE_Pos));
//		LL_SPI_TransmitData16(SPI2, *buff_ptr++);
//		nbytes -= 2;
//	}

	__HAL_SPI_DISABLE(&hspi2);
	LCD_CSX_HIGH();
	LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_8BIT);
	__HAL_SPI_ENABLE(&hspi2);

}

void bsp_lcd_write_dma(uint32_t src_addr, uint32_t nbytes)
{
	lcd_write_dma(src_addr,nbytes);
}


void bsp_lcd_set_background_color(uint32_t rgb888)
{
	bsp_lcd_fill_rect(rgb888,0,(BSP_LCD_ACTIVE_WIDTH),0,(BSP_LCD_ACTIVE_HEIGHT));
}

uint16_t bsp_lcd_convert_rgb888_to_rgb565(uint32_t rgb888)
{
	return convert_rgb888_to_rgb565(rgb888);
}

/*
 * Disc: Creates a rectangle and fills color
 * rgb888: Color value in RGB888 format
 * x_start : Horizontal start position of the rectangle ( 0 <= x_start < BSP_FB_WIDTH)
 * x_width : Width of the rectangle in number of pixels ( 1 <= x_width <= BSP_FB_WIDTH )
 * y_start : Vertical start position of the rectangle ( 0 <= y_start < BSP_FB_HEIGHT)
 * y_height : Height of the rectangle in number of pixels ( 1 <= y_height <= BSP_FB_HEIGHT )
 */
void bsp_lcd_fill_rect(uint32_t rgb888, uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height)
{

	uint32_t total_bytes_to_write = 0;
	uint32_t bytes_sent_so_far = 0;
	uint32_t remaining_bytes = 0;
	uint32_t npix;
	uint32_t pixels_sent = 0;
	uint32_t x1, y1;
	uint32_t pixel_per_line = x_width;

	if ((x_start + x_width) > BSP_LCD_ACTIVE_WIDTH)
		return;
	if ((y_start + y_height) > BSP_LCD_ACTIVE_HEIGHT)
		return;

	//1. calculate total number of bytes written in to DB
	total_bytes_to_write = get_total_bytes(hlcd, x_width, y_height);
	remaining_bytes = total_bytes_to_write;
	while (remaining_bytes)
	{
		x1 = x_start + (pixels_sent % pixel_per_line);
		y1 = y_start + (pixels_sent / pixel_per_line);

		make_area(&hlcd->area, x1, x_width, y1, y_height);

		if (x1 != x_start)
		{
			npix = x_start + x_width - x1;
		}
		else
		{
			npix = bytes_to_pixels(remaining_bytes, hlcd->pixel_format);
		}

		bytes_sent_so_far += copy_to_draw_buffer(hlcd,
				pixels_to_bytes(npix, hlcd->pixel_format), rgb888);
		pixels_sent = bytes_to_pixels(bytes_sent_so_far, hlcd->pixel_format);
		remaining_bytes = total_bytes_to_write - bytes_sent_so_far;
	}
}

void lcd_reset(void)
{
	LCD_RESX_LOW();
	HAL_Delay(50);
	LCD_RESX_HIGH();
	HAL_Delay(50);
}

void lcd_config(void)
{
	uint8_t params[15];
	lcd_write_cmd(ILI9341_SWRESET);
	lcd_write_cmd(ILI9341_POWERB);
	params[0] = 0x00;
	params[1] = 0xD9;
	params[2] = 0x30;
	lcd_write_data(params, 3);

	lcd_write_cmd(ILI9341_POWER_SEQ);
	params[0]= 0x64;
	params[1]= 0x03;
	params[2]= 0X12;
	params[3]= 0X81;
	lcd_write_data(params, 4);

	lcd_write_cmd(ILI9341_DTCA);
	params[0]= 0x85;
	params[1]= 0x10;
	params[2]= 0x7A;
	lcd_write_data(params, 3);

	lcd_write_cmd(ILI9341_POWERA);
	params[0]= 0x39;
	params[1]= 0x2C;
	params[2]= 0x00;
	params[3]= 0x34;
	params[4]= 0x02;
	lcd_write_data(params, 5);

	lcd_write_cmd(ILI9341_PRC);
	params[0]= 0x20;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_DTCB);
	params[0]= 0x00;
	params[1]= 0x00;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_POWER1);
	params[0]= 0x1B;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_POWER2);
	params[0]= 0x12;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_VCOM1);
	params[0]= 0x08;
	params[1]= 0x26;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_VCOM2);
	params[0]= 0XB7;
	lcd_write_data(params, 1);


	lcd_write_cmd(ILI9341_PIXEL_FORMAT);
	params[0]= 0x55; //select RGB565
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_FRMCTR1);
	params[0]= 0x00;
	params[1]= 0x1B;//frame rate = 70
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_DFC);    // Display Function Control
	params[0]= 0x0A;
	params[1]= 0xA2;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_3GAMMA_EN);    // 3Gamma Function Disable
	params[0]= 0x02; //LCD_WR_DATA(0x00);
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_GAMMA);
	params[0]= 0x01;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_PGAMMA);    //Set Gamma
	params[0]= 0x0F;
	params[1]= 0x1D;
	params[2]= 0x1A;
	params[3]= 0x0A;
	params[4]= 0x0D;
	params[5]= 0x07;
	params[6]= 0x49;
	params[7]= 0X66;
	params[8]= 0x3B;
	params[9]= 0x07;
	params[10]= 0x11;
	params[11]= 0x01;
	params[12]= 0x09;
	params[13]= 0x05;
	params[14]= 0x04;
	lcd_write_data(params, 15);

	lcd_write_cmd(ILI9341_NGAMMA);
	params[0]= 0x00;
	params[1]= 0x18;
	params[2]= 0x1D;
	params[3]= 0x02;
	params[4]= 0x0F;
	params[5]= 0x04;
	params[6]= 0x36;
	params[7]= 0x13;
	params[8]= 0x4C;
	params[9]= 0x07;
	params[10]= 0x13;
	params[11]= 0x0F;
	params[12]= 0x2E;
	params[13]= 0x2F;
	params[14]= 0x05;
	lcd_write_data(params, 15);

	lcd_write_cmd(ILI9341_SLEEP_OUT); //Exit Sleep
	HAL_Delay(100);
	lcd_write_cmd(ILI9341_DISPLAY_ON); //display on
}

void lcd_set_orientation(uint8_t orientation)
{
	uint8_t param;

	if(orientation == LANDSCAPE){
		param = MADCTL_MV | MADCTL_MY | MADCTL_BGR; /*Memory Access Control <Landscape setting>*/
	}else if(orientation == PORTRAIT){
		param = MADCTL_MY| MADCTL_MX| MADCTL_BGR;  /* Memory Access Control <portrait setting> */
	}

	lcd_write_cmd(ILI9341_MAC);    // Memory Access Control command
	lcd_write_data(&param, 1);
}


void lcd_write_cmd(uint8_t cmd)
{
	LCD_CSX_LOW();
	LCD_DCX_LOW(); // DCX = 0, for commands

	HAL_SPI_Transmit(&hspi2, &cmd, 1U, HAL_MAX_DELAY);

	// wait till TXE becomes 1 (buffer empty)
	//while (!REG_READ_BIT(SPI->SR, SPI_SR_TXE_Pos));
	//REG_WRITE(SPI->DR, cmd);
	//while (!REG_READ_BIT(SPI->SR, SPI_SR_TXE_Pos));
	// make sure that command has been sent - check BUSY bit, wait if it is 1
	//while (REG_READ_BIT(SPI->SR, SPI_SR_BSY_Pos));

	// to avoid OVR error - do dummy read
	//uint8_t tmp = SPI->DR;
	//(void)tmp;

	LCD_DCX_HIGH();
	LCD_CSX_HIGH();
}

void lcd_write_data(uint8_t *buffer, uint32_t len)
{
	LCD_CSX_LOW();

	HAL_SPI_Transmit(&hspi2, buffer, len, HAL_MAX_DELAY);

	//for (uint32_t i = 0; i < len; i++)
	//{
	//	while (!REG_READ_BIT(SPI->SR, SPI_SR_TXE_Pos));
	//	REG_WRITE(SPI->DR, buffer[i]);
	//}

	//while (!REG_READ_BIT(SPI->SR, SPI_SR_TXE_Pos));
	// make sure that data byte has been sent - check BUSY bit, wait if it is 1
	//while (REG_READ_BIT(SPI->SR, SPI_SR_BSY_Pos));

	// to avoid OVR error - do dummy read
	//uint8_t tmp = SPI->DR;
	//(void)tmp;

	LCD_CSX_HIGH();
}

void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
 {
	 lcd_area_t area;
	 area.x1 = x1;
	 area.x2 = x2;
	 area.y1 = y1;
	 area.y2 = y2;
	 lcd_set_display_area(&area);
 }

void lcd_set_display_area(lcd_area_t *area)
{
	uint8_t params[4];
	/*Column address set(2Ah) */
	params[0] = HIGH_16(area->x1);
	params[1] = LOW_16(area->x1);
	params[2] = HIGH_16(area->x2);
	params[3] = LOW_16(area->x2);
	lcd_write_cmd(ILI9341_CASET);
	lcd_write_data(params, 4);

	params[0] = HIGH_16(area->y1);
	params[1] = LOW_16(area->y1);
	params[2] = HIGH_16(area->y2);
	params[3] = LOW_16(area->y2);
	lcd_write_cmd(ILI9341_RASET);
	lcd_write_data(params, 4);

}

void lcd_buffer_init(bsp_lcd_t *lcd)
{
	lcd->draw_buffer1 = bsp_db;
	lcd->draw_buffer2 = bsp_wb;
	lcd->buff_to_draw = NULL;
	lcd->buff_to_flush = NULL;
}

void lcd_dma_init(bsp_lcd_t *lcd)
{
#if (USE_DMA == 1)
	initialize_lcd_write_dma((uint32_t)bsp_wb,(uint32_t)&SPI2->DR);
#endif
}

void lcd_write_dma(uint32_t src_addr, uint32_t nbytes)
{
	__HAL_SPI_DISABLE(&hspi2);
	LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_16BIT);
	__HAL_SPI_ENABLE(&hspi2);
	LCD_CSX_LOW();
	uint32_t nitems = nbytes /2;

    // determine how many chunks of input buffer need to be sent
    n_chunks = (nitems % 0xFFFFU != 0U) ? (nitems / 0xFFFFU + 1UL) : (nitems / 0xFFFFU);
    // assign DMA iteration counter
    dma_spi_cnt = n_chunks;
    // src offset
    uint32_t src_offset;
    // chunk size
    uint32_t chunk_size;

    /* Send all chunks via DMA */

    if (nitems > 0xFFFFU)
    {
        chunk_size = 0xFFFFU;
        nitems -= 0xFFFFU;
    }
    else
    {
        chunk_size = nitems;
    }

    g_nitems = nitems;
    src_offset = 0xFFFFU * (n_chunks - dma_spi_cnt) * 2;
    src_addr += src_offset;
    scr_address = src_addr;
    dma_spi_cnt--;

    //__disable_dma(DMA1_Stream4);
    /*Address configuration */
    //REG_SET_VAL(DMA1_Stream4->PAR,dst_addr,0xFFFFFFFFU,DMA_SxPAR_PA_Pos);
    //REG_SET_VAL(DMA1_Stream4->M0AR, scr_address, 0xFFFFFFFFU,DMA_SxM0AR_M0A_Pos);
    /*Transfer length */
    //REG_SET_VAL(DMA1_Stream4->NDTR, chunk_size,0xFFFFU,DMA_SxNDT_Pos);
    //__enable_dma(DMA1_Stream4);
    //REG_SET_BIT(SPI->CR2,SPI_CR2_TXDMAEN_Pos);
    HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*) src_addr, chunk_size);
}

uint16_t convert_rgb888_to_rgb565(uint32_t rgb888)
{
    uint16_t r,g,b;
	r = (rgb888 >> 19) & 0x1FU;
	g = (rgb888 >> 10) & 0x3FU;
	b = (rgb888 >> 3)  & 0x1FU;
	return (uint16_t)((r << 11) | (g << 5) | b);
}

uint32_t get_total_bytes(bsp_lcd_t *hlcd,uint32_t w , uint32_t h)
{
	uint8_t bytes_per_pixel = 2;
	if(hlcd->pixel_format == BSP_LCD_PIXEL_FMT_RGB565){
		bytes_per_pixel = 2;
	}
	return (w * h * bytes_per_pixel);
}


void make_area(lcd_area_t *area,uint32_t x_start, uint32_t x_width,uint32_t y_start,uint32_t y_height){

	uint16_t lcd_total_width,lcd_total_height;

	lcd_total_width =  BSP_LCD_ACTIVE_WIDTH-1;
	lcd_total_height = BSP_LCD_ACTIVE_HEIGHT -1;

	area->x1 = x_start;
	area->x2 = x_start + x_width -1;
	area->y1 = y_start;
	area->y2 = y_start + y_height -1;

	area->x2 = (area->x2 > lcd_total_width) ? lcd_total_width :area->x2;
	area->y2 = (area->y2 > lcd_total_height) ? lcd_total_height : area->y2;

}

uint8_t *get_buff(bsp_lcd_t *hlcd)
{
	uint32_t buf1 = (uint32_t)hlcd->draw_buffer1;
	uint32_t buf2 = (uint32_t)hlcd->draw_buffer2;
	uint8_t* retPtr = NULL;

	__disable_irq();

	if(hlcd->buff_to_draw == NULL && hlcd->buff_to_flush == NULL)
	{
		retPtr = hlcd->draw_buffer1;
	}
	else if((uint32_t)hlcd->buff_to_flush == buf1 && hlcd->buff_to_draw == NULL )
	{
		retPtr = hlcd->draw_buffer2;
	}
	else if ((uint32_t)hlcd->buff_to_flush == buf2 && hlcd->buff_to_draw == NULL)
	{
		retPtr = hlcd->draw_buffer1;
	}
	__enable_irq();

	return retPtr;
}


uint32_t copy_to_draw_buffer( bsp_lcd_t *hlcd,uint32_t nbytes,uint32_t rgb888)
{
	uint16_t *fb_ptr = NULL;
	uint32_t npixels;
	hlcd->buff_to_draw = get_buff(hlcd);
	fb_ptr = (uint16_t*)hlcd->buff_to_draw;
	nbytes =  ((nbytes > DB_SIZE)?DB_SIZE:nbytes);
	npixels= bytes_to_pixels(nbytes,hlcd->pixel_format);
	if(hlcd->buff_to_draw != NULL){
		for(uint32_t i = 0 ; i < npixels ;i++){
			*fb_ptr = convert_rgb888_to_rgb565(rgb888);
			fb_ptr++;
		}
		hlcd->write_length = pixels_to_bytes(npixels,hlcd->pixel_format);
		while(!is_lcd_write_allowed(hlcd));
		hlcd->buff_to_flush = hlcd->buff_to_draw;
		hlcd->buff_to_draw = NULL;
		lcd_flush(hlcd);
		return pixels_to_bytes(npixels,hlcd->pixel_format);
	}

	return 0;
}


static uint8_t is_lcd_write_allowed(bsp_lcd_t *hlcd)
{
	uint8_t retVal;
	__disable_irq();
	retVal = (hlcd->buff_to_flush) ? FALSE: TRUE;
	__enable_irq();
	return retVal;
}




 void lcd_flush(bsp_lcd_t *hlcd)
{
	lcd_set_display_area(&hlcd->area);
	bsp_lcd_send_cmd_mem_write();
#if (USE_DMA == 0)
	bsp_lcd_write(hlcd->buff_to_flush,hlcd->write_length);
	hlcd->buff_to_flush = NULL;
#else
	lcd_write_dma((uint32_t)hlcd->buff_to_flush,hlcd->write_length);
#endif
}


uint32_t bytes_to_pixels(uint32_t nbytes, uint8_t pixel_format)
{
	UNUSED(pixel_format);
	return nbytes/2;

}

uint32_t pixels_to_bytes(uint32_t pixels, uint8_t pixel_format)
{
	UNUSED(pixel_format);
	return pixels * 2UL;
}

/////////////////////////////////////////////////////DMA functions///////////////////////////////////////////
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	// TODO: add here external application Error Callback
	while(1);
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    //__HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    lcd_handle.buff_to_flush = NULL;
    // scr offset
    uint32_t src_offset;
    // chunk size
    uint32_t chunk_size;

    if (dma_spi_cnt == 0U)
    {
#if (USE_DMA == 1)
        /* All data chunks are already sent via DMA */

        /* Release SC pin */
        LCD_CSX_HIGH();
        /* Restore SPI data width to 8 bits */
        __HAL_SPI_DISABLE(&hspi2);
        LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_8BIT);
        __HAL_SPI_ENABLE(&hspi2);
        /* Reset global counters */
        scr_address = 0;
        src_offset = 0;
        g_nitems = 0;
        chunk_size = 0;
#endif
        // TODO: add here external application TC Callback
    }
    else
    {
        /* Send next chunk via DMA */

        if (g_nitems > 0xFFFFU)
        {
            chunk_size = 0xFFFFU;
            g_nitems -= 0xFFFFU;
        }
        else
        {
            chunk_size = g_nitems;
        }

        src_offset = 0xFFFFU * (n_chunks - dma_spi_cnt) * 2;
        scr_address += src_offset;
        dma_spi_cnt--;

        //__disable_dma(DMA1_Stream4);
        /*Address configuration */
        //REG_SET_VAL(DMA1_Stream4->M0AR, scr_address, 0xFFFFFFFFU,DMA_SxM0AR_M0A_Pos);
        /*Transfer length */
        //REG_SET_VAL(DMA1_Stream4->NDTR, chunk_size,0xFFFFU,DMA_SxNDT_Pos);
        //__enable_dma(DMA1_Stream4);
        //REG_SET_BIT(SPI->CR2,SPI_CR2_TXDMAEN_Pos);
        HAL_SPI_Transmit_DMA(hspi, (uint8_t*) scr_address, chunk_size);
    }

    HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_3);
}

void initialize_lcd_write_dma(uint32_t src_addr, uint32_t dst_addr)
{
//	DMA_TypeDef *pDMA = DMA1;
//	RCC_TypeDef *pRCC = RCC;
//	DMA_Stream_TypeDef *pStream = DMA1_Stream4;
//
//	//Enable clock for DMA1
//	REG_SET_BIT(pRCC->AHB1ENR,RCC_AHB1ENR_DMA1EN_Pos);
//
//	/*Stream configuration */
//	REG_CLR_BIT(pStream->CR,DMA_SxCR_EN_Pos); /*Make sure that stream is disabled */
//	REG_SET_VAL(pStream->CR,0X00,0x7U,DMA_SxCR_CHSEL_Pos); /* SPI2_TX is on channel 0 */
//	REG_SET_VAL(pStream->CR,0x00,0x3U,DMA_SxCR_MBURST_Pos); /*Single transfer*/
//	REG_SET_VAL(pStream->CR,0x3U,0x3U,DMA_SxCR_PL_Pos); /*Priority very high*/
//	REG_CLR_BIT(pStream->CR,DMA_SxCR_PINCOS_Pos);
//	REG_SET_VAL(pStream->CR,0x01U,0x3U,DMA_SxCR_MSIZE_Pos); /* MSIZE = hw */
//	REG_SET_VAL(pStream->CR,0x01U,0x3U,DMA_SxCR_PSIZE_Pos); /* PSIZE = hw */
//	REG_SET_BIT(pStream->CR,DMA_SxCR_MINC_Pos); /* Increment memory address*/
//	REG_CLR_BIT(pStream->CR,DMA_SxCR_PINC_Pos); /* Fixed peripheral address */
//	REG_SET_VAL(pStream->CR,0x1U,0x3U,DMA_SxCR_DIR_Pos); /* Direction : Memory to peripheral */
//	REG_CLR_BIT(pStream->CR,DMA_SxCR_PFCTRL_Pos); /* Flow controller = DMA */
//
//	/*Address configuration */
//	REG_SET_VAL(pStream->PAR,dst_addr,0xFFFFFFFFU,DMA_SxPAR_PA_Pos);
//	REG_SET_VAL(pStream->M0AR,src_addr,0xFFFFFFFFU,DMA_SxM0AR_M0A_Pos);
//
//	/*FIFO control*/
//	REG_CLR_BIT(pStream->FCR,DMA_SxFCR_DMDIS_Pos); /* Direct mode enabled */
	/*Stream interrupt configuration */
    __HAL_SPI_DISABLE(&hspi2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4); /* Enable Transfer complete interrupt */
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_4); /* Enable transfer error interrupt */
	LL_DMA_EnableIT_DME(DMA1, LL_DMA_STREAM_4); /* Enable direct mode error interrupt */
	__HAL_SPI_ENABLE(&hspi2);
}

void write_frame(uint8_t *fb_addr, uint32_t nbytes)
{
	bsp_lcd_set_display_area(0, BSP_LCD_ACTIVE_WIDTH-1, 0, BSP_LCD_ACTIVE_HEIGHT-1);
	bsp_lcd_send_cmd_mem_write();
	//bsp_lcd_write(fb_addr, nbytes);
	bsp_lcd_write_dma((uint32_t)fb_addr, nbytes);
}
