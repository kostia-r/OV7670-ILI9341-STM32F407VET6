
#include "ILI9341.h"
#include "ILI9341_reg.h"

// TODO: update this structure and move it to separate config file
typedef struct
{
    uint16_t x1;
    uint16_t x2;
    uint16_t y1;
    uint16_t y2;
} lcd_area_t;

typedef struct
{
    uint32_t chunk_cnt;
    uint32_t src_address;
    uint32_t n_chunks;
    uint32_t nitems;
    uint8_t datatype;
} spi_dma_runtime_t;

static volatile struct
{
    uint8_t orientation;
    uint8_t pixel_format;
    uint8_t *draw_buffer1;
    uint8_t *draw_buffer2;
    uint32_t write_length;
    const uint8_t *buff_to_draw;
    const uint8_t *buff_to_flush;
    lcd_area_t area;
    spi_dma_runtime_t spi_dma;
    void (*dma_cplt_cb)(void);
    void (*dma_err_cb)(void);
    SPI_HandleTypeDef *hspi;
} hlcd;

#define DB_SIZE                    (10UL * 1024UL)
uint8_t bsp_db[DB_SIZE];
uint8_t bsp_wb[DB_SIZE];

/************************* LOCAL FUNCTIONS PROTOTYPES ************************/
static void lcd_Reset(void);
static void lcd_Config(void);
static void lcd_SetOrientation(uint8_t orientation);
static void lcd_BufferInit(void);
static uint8_t lcd_IsWriteAllowed(void);
static void lcd_SetDisplArea(void);
static uint32_t lcd_GetTotalBytes(uint32_t w, uint32_t h);
static uint8_t* lcd_GetBuff(void);
static void lcd_Flush(void);
static void lcd_WriteCmd(uint8_t cmd, const uint8_t *params, uint32_t param_len);
static void lcd_WriteDma(uint32_t src_addr, uint32_t nbytes);
static uint32_t lcd_CpyToDrawBuffer(uint32_t nbytes, uint32_t rgb888);
static void lcd_MakeArea(uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height);
static uint16_t lcd_RGB888toRGB565(uint32_t rgb888);

enum
{
    FALSE, TRUE
};

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

#define BYTES_TO_PIXELS(bytes)      ((bytes) / BSP_LCD_PIXEL_FMT)
#define PIXELS_TO_BYTES(pixels)     ((pixels) * BSP_LCD_PIXEL_FMT)

#ifndef UNUSED
#define UNUSED(x)                  (void)x
#endif

//TODO: move it to config
#define __enable_dma(stream)        SET_BIT(stream->CR,DMA_SxCR_EN_Pos);
#define __disable_dma(stream)       CLEAR_BIT(stream->CR,DMA_SxCR_EN_Pos);

#define HIGH_16(x)                  ((((uint16_t)x) >> 8U) & 0xFFU)
#define LOW_16(x)                   ((((uint16_t)x) >> 0U) & 0xFFU)

//TODO: move GPIO parameters to config file
#define LCD_RESX_HIGH()				SET_BIT(LCD_RESX_GPIO_Port->ODR,LCD_RESX_Pin)
#define LCD_RESX_LOW()				CLEAR_BIT(LCD_RESX_GPIO_Port->ODR,LCD_RESX_Pin)

//TODO: move it to config
#if (ILI9341_SPI_CS_HW_MANAGE == 1)
#define LCD_CSX_HIGH()
#define LCD_CSX_LOW()
#else
#define LCD_CSX_HIGH()				SET_BIT(LCD_CSX_GPIO_Port->ODR,LCD_CSX_Pin)
#define LCD_CSX_LOW()				CLEAR_BIT(LCD_CSX_GPIO_Port->ODR,LCD_CSX_Pin)
#endif

//TODO: move it to config
#define LCD_DCX_HIGH()				SET_BIT(LCD_DCX_GPIO_Port->ODR,LCD_DCX_Pin)
#define LCD_DCX_LOW()				CLEAR_BIT(LCD_DCX_GPIO_Port->ODR,LCD_DCX_Pin)

//TODO: move it to config
/* Set SPI data width to 8 bits if 16 bits has been already set: FOR STM32F4 */
#define ILI9341_SPI_SET_8_BIT()     do{if(READ_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_DFF))\
    {CLEAR_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_SPE);/* Disable SPI */\
    CLEAR_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_DFF);/* Set 8-bit data width */\
    SET_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_SPE);/* Enable SPI */}}while(0U)

/* Set SPI data width to 16 bits if 8 bits has been already set: FOR STM32F4 */
#define ILI9341_SPI_SET_16_BIT()    do{if(!READ_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_DFF))\
    {CLEAR_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_SPE);/* Disable SPI */\
    SET_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_DFF);/* Set 16-bit data width */\
    SET_BIT(hlcd.hspi->Instance->CR1, SPI_CR1_SPE);/* Enable SPI */}}while(0U)

// TODO: add timeouts
#define ILI9341_CHECK_SPI(spi_handler_ptr)         do{\
    while(HAL_SPI_STATE_READY != HAL_SPI_GetState(spi_handler_ptr));\
    }while(0)

#define ILI9341_DATAWIDTH_8BIT             (1U)
#define ILI9341_DATAWIDTH_16BIT            (2U)
#define ILI9341_DATAWIDTH                  ILI9341_DATAWIDTH_16BIT

#define ILI9341_WRITE_CMD                  (0U)
#define ILI9341_WRITE_DATA                 (1U)

#define ILI9341_DMA_MAX_ITEMS              (0xFFFFU)


void ILI9341_Init(SPI_HandleTypeDef *spi_handle)
{
    hlcd.hspi = spi_handle;
    hlcd.orientation = BSP_LCD_ORIENTATION;
    hlcd.pixel_format = BSP_LCD_PIXEL_FMT;
    lcd_Reset();
    lcd_Config();
    hlcd.area.x1 = 0;
    hlcd.area.x2 = BSP_LCD_ACTIVE_WIDTH - 1;
    hlcd.area.y1 = 0;
    hlcd.area.y2 = BSP_LCD_ACTIVE_HEIGHT - 1;
    lcd_SetDisplArea();
    lcd_SetOrientation(hlcd.orientation);
    lcd_BufferInit();
}

void ILI9341_RegisterCallback(uint8_t cb_type, void (*fnc_ptr)(void))
{
    switch (cb_type)
    {
        case ILI9341_TC_CALLBACK:
            hlcd.dma_cplt_cb = fnc_ptr;
            break;

        case INI9341_ERR_CALLBACK:
            hlcd.dma_err_cb = fnc_ptr;
            break;

        default:
            break;
    }
}

void ILI9341_DrawFrame(const uint8_t *fb_addr, uint32_t nbytes)
{
    hlcd.area.x1 = 0U;
    hlcd.area.x2 = BSP_LCD_ACTIVE_WIDTH - 1;
    hlcd.area.y1 = 0U;
    hlcd.area.y2 = BSP_LCD_ACTIVE_HEIGHT - 1;
    hlcd.buff_to_flush = fb_addr;
    hlcd.write_length = nbytes;
    lcd_Flush();
}

void ILI9341_DrawCrop(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    hlcd.area.x1 = x1;
    hlcd.area.x2 = x2;
    hlcd.area.y1 = y1;
    hlcd.area.y2 = y2;
    hlcd.buff_to_flush = buffer;
    hlcd.write_length = nbytes;
    lcd_Flush();
}

void* ILI9341_GetDrawBuffer1Addr(void)
{
    return (void*) hlcd.draw_buffer1;
}

void* ILI9341_GetDrawBuffer2Addr(void)
{
    return (void*) hlcd.draw_buffer2;
}

void ILI9341_SetBackgroundColor(uint32_t rgb888)
{
    ILI9341_FillRect(rgb888, 0, BSP_LCD_ACTIVE_WIDTH, 0, BSP_LCD_ACTIVE_HEIGHT);
}

/*
 * Disc: Creates a rectangle and fills color
 * rgb888: Color value in RGB888 format
 * x_start : Horizontal start position of the rectangle ( 0 <= x_start < BSP_FB_WIDTH)
 * x_width : Width of the rectangle in number of pixels ( 1 <= x_width <= BSP_FB_WIDTH )
 * y_start : Vertical start position of the rectangle ( 0 <= y_start < BSP_FB_HEIGHT)
 * y_height : Height of the rectangle in number of pixels ( 1 <= y_height <= BSP_FB_HEIGHT )
 */
void ILI9341_FillRect(uint32_t rgb888, uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height)
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
    total_bytes_to_write = lcd_GetTotalBytes(x_width, y_height);
    remaining_bytes = total_bytes_to_write;

    while (remaining_bytes)
    {
        x1 = x_start + (pixels_sent % pixel_per_line);
        y1 = y_start + (pixels_sent / pixel_per_line);

        lcd_MakeArea(x1, x_width, y1, y_height);

        if (x1 != x_start)
        {
            npix = x_start + x_width - x1;
        }
        else
        {
            npix = BYTES_TO_PIXELS(remaining_bytes);
        }

        bytes_sent_so_far += lcd_CpyToDrawBuffer(PIXELS_TO_BYTES(npix), rgb888);
        pixels_sent = BYTES_TO_PIXELS(bytes_sent_so_far);
        remaining_bytes = total_bytes_to_write - bytes_sent_so_far;
    }
}

static void lcd_SetOrientation(uint8_t orientation)
{
    uint8_t param;

    switch (orientation)
    {
        case LANDSCAPE:
        {
            /*Memory Access Control <Landscape setting>*/
            param = MADCTL_MV | MADCTL_MY | MADCTL_BGR;
            break;
        }

        case PORTRAIT:
        {
            /* Memory Access Control <portrait setting> */
            param = MADCTL_MY | MADCTL_MX | MADCTL_BGR;
            break;
        }

        default:
        {
            break;
        }
    }

    /* Memory Access Control command */
    lcd_WriteCmd(ILI9341_MAC, &param, 1U);
}

static void lcd_Reset(void)
{
    LCD_RESX_LOW();
    HAL_Delay(50);
    LCD_RESX_HIGH();
    HAL_Delay(50);
}

static void lcd_Config(void)
{
    uint8_t params[15] = { 0U };

    params[0] = 0x00;
    params[1] = 0xD9;
    params[2] = 0x30;
    lcd_WriteCmd(ILI9341_SWRESET, NULL, 0U);
    lcd_WriteCmd(ILI9341_POWERB, params, 3U);

    params[0] = 0x64;
    params[1] = 0x03;
    params[2] = 0X12;
    params[3] = 0X81;
    lcd_WriteCmd(ILI9341_POWER_SEQ, params, 4U);

    params[0] = 0x85;
    params[1] = 0x10;
    params[2] = 0x7A;
    lcd_WriteCmd(ILI9341_DTCA, params, 3U);

    params[0] = 0x39;
    params[1] = 0x2C;
    params[2] = 0x00;
    params[3] = 0x34;
    params[4] = 0x02;
    lcd_WriteCmd(ILI9341_POWERA, params, 5U);

    params[0] = 0x20;
    lcd_WriteCmd(ILI9341_PRC, params, 1U);

    params[0] = 0x00;
    params[1] = 0x00;
    lcd_WriteCmd(ILI9341_DTCB, params, 2U);

    params[0] = 0x1B;
    lcd_WriteCmd(ILI9341_POWER1, params, 1U);

    params[0] = 0x12;
    lcd_WriteCmd(ILI9341_POWER2, params, 1U);

    params[0] = 0x08;
    params[1] = 0x26;
    lcd_WriteCmd(ILI9341_VCOM1, params, 2);

    params[0] = 0XB7;
    lcd_WriteCmd(ILI9341_VCOM2, params, 1);

    /* Select RGB565 */
    params[0] = 0x55;
    lcd_WriteCmd(ILI9341_PIXEL_FORMAT, params, 1);

    /* frame rate = 70 */
    params[0] = 0x00;
    params[1] = 0x1B;
    lcd_WriteCmd(ILI9341_FRMCTR1, params, 2);

    /* Display Function Control */
    params[0] = 0x0A;
    params[1] = 0xA2;
    lcd_WriteCmd(ILI9341_DFC, params, 2);

    /* 3Gamma Function Disable */
    params[0] = 0x02; //LCD_WR_DATA(0x00);
    lcd_WriteCmd(ILI9341_3GAMMA_EN, params, 1);

    params[0] = 0x01;
    lcd_WriteCmd(ILI9341_GAMMA, params, 1);

    /* Set Gamma */
    params[0] = 0x0F;
    params[1] = 0x1D;
    params[2] = 0x1A;
    params[3] = 0x0A;
    params[4] = 0x0D;
    params[5] = 0x07;
    params[6] = 0x49;
    params[7] = 0X66;
    params[8] = 0x3B;
    params[9] = 0x07;
    params[10] = 0x11;
    params[11] = 0x01;
    params[12] = 0x09;
    params[13] = 0x05;
    params[14] = 0x04;
    lcd_WriteCmd(ILI9341_PGAMMA, params, 15);

    params[0] = 0x00;
    params[1] = 0x18;
    params[2] = 0x1D;
    params[3] = 0x02;
    params[4] = 0x0F;
    params[5] = 0x04;
    params[6] = 0x36;
    params[7] = 0x13;
    params[8] = 0x4C;
    params[9] = 0x07;
    params[10] = 0x13;
    params[11] = 0x0F;
    params[12] = 0x2E;
    params[13] = 0x2F;
    params[14] = 0x05;
    lcd_WriteCmd(ILI9341_NGAMMA, params, 15);

    /* Exit Sleep */
    lcd_WriteCmd(ILI9341_SLEEP_OUT, NULL, 0U);

    HAL_Delay(100);

    /* Display on */
    lcd_WriteCmd(ILI9341_DISPLAY_ON, NULL, 0U);
}

static void lcd_WriteCmd(uint8_t cmd, const uint8_t *params, uint32_t param_len)
{
    ILI9341_CHECK_SPI(hlcd.hspi);

    /* Set SPI data width as 8 bits */
    ILI9341_SPI_SET_8_BIT();

    LCD_CSX_LOW();
    LCD_DCX_LOW(); // for commands
    hlcd.spi_dma.datatype = ILI9341_WRITE_CMD;

    HAL_SPI_Transmit(hlcd.hspi, &cmd, 1U, HAL_MAX_DELAY);

    LCD_DCX_HIGH();// for commands
    LCD_CSX_HIGH();

    //TODO: check if its possible to use one CSX cycle in case without NSS?
    if (params != NULL)
    {
        ILI9341_CHECK_SPI(hlcd.hspi); LCD_CSX_LOW();
        hlcd.spi_dma.datatype = ILI9341_WRITE_DATA;
        HAL_SPI_Transmit(hlcd.hspi, (uint8_t*) params, param_len, HAL_MAX_DELAY);
        LCD_CSX_HIGH();
    }
}

static void lcd_WriteDma(uint32_t src_addr, uint32_t nbytes)
{
    ILI9341_CHECK_SPI(hlcd.hspi);

    /* Set SPI data width as 16 bits */
    ILI9341_SPI_SET_16_BIT();

    LCD_CSX_LOW();
    hlcd.spi_dma.datatype = ILI9341_WRITE_DATA;

    uint32_t nitems = nbytes / ILI9341_DATAWIDTH;

    // determine how many chunks of input buffer need to be sent
    hlcd.spi_dma.n_chunks = (nitems % ILI9341_DMA_MAX_ITEMS != 0U) ?
                    (nitems / ILI9341_DMA_MAX_ITEMS + 1UL) :
                    (nitems / ILI9341_DMA_MAX_ITEMS);
    // assign DMA iteration counter
    hlcd.spi_dma.chunk_cnt = hlcd.spi_dma.n_chunks;
    // chunk size
    uint32_t chunk_size;

    /* Send all chunks via DMA */

    if (nitems > ILI9341_DMA_MAX_ITEMS)
    {
        chunk_size = ILI9341_DMA_MAX_ITEMS;
        nitems -= ILI9341_DMA_MAX_ITEMS;
    }
    else
    {
        chunk_size = nitems;
    }

    hlcd.spi_dma.nitems = nitems;
    /* Shift source address */
    hlcd.spi_dma.src_address = src_addr + ILI9341_DMA_MAX_ITEMS * (hlcd.spi_dma.n_chunks - hlcd.spi_dma.chunk_cnt) * ILI9341_DATAWIDTH;
    hlcd.spi_dma.chunk_cnt--;

    HAL_SPI_Transmit_DMA(hlcd.hspi, (uint8_t*) hlcd.spi_dma.src_address, chunk_size);
}

static void lcd_SetDisplArea(void)
{
    uint8_t params[4];
    /*Column address set(2Ah) */
    params[0] = HIGH_16(hlcd.area.x1);
    params[1] = LOW_16(hlcd.area.x1);
    params[2] = HIGH_16(hlcd.area.x2);
    params[3] = LOW_16(hlcd.area.x2);
    lcd_WriteCmd(ILI9341_CASET, params, 4U);

    params[0] = HIGH_16(hlcd.area.y1);
    params[1] = LOW_16(hlcd.area.y1);
    params[2] = HIGH_16(hlcd.area.y2);
    params[3] = LOW_16(hlcd.area.y2);
    lcd_WriteCmd(ILI9341_RASET, params, 4U);
}

static void lcd_BufferInit(void)
{
    hlcd.draw_buffer1 = bsp_db;
    hlcd.draw_buffer2 = bsp_wb;
    hlcd.buff_to_draw = NULL;
    hlcd.buff_to_flush = NULL;
}

static uint16_t lcd_RGB888toRGB565(uint32_t rgb888)
{
    uint16_t r, g, b;
    r = (rgb888 >> 19) & 0x1FU;
    g = (rgb888 >> 10) & 0x3FU;
    b = (rgb888 >> 3) & 0x1FU;
    return (uint16_t) ((r << 11) | (g << 5) | b);
}

static uint32_t lcd_GetTotalBytes(uint32_t w, uint32_t h)
{
    uint8_t bytes_per_pixel = 2;

    if (hlcd.pixel_format == BSP_LCD_PIXEL_FMT_RGB565)
    {
        bytes_per_pixel = 2;
    }

    return (w * h * bytes_per_pixel);
}


static void lcd_MakeArea(uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height)
{

    uint16_t lcd_total_width, lcd_total_height;

    lcd_total_width = BSP_LCD_ACTIVE_WIDTH - 1;
    lcd_total_height = BSP_LCD_ACTIVE_HEIGHT - 1;

    hlcd.area.x1 = x_start;
    hlcd.area.x2 = x_start + x_width - 1;
    hlcd.area.y1 = y_start;
    hlcd.area.y2 = y_start + y_height - 1;

    hlcd.area.x2 = (hlcd.area.x2 > lcd_total_width) ? lcd_total_width : hlcd.area.x2;
    hlcd.area.y2 = (hlcd.area.y2 > lcd_total_height) ? lcd_total_height : hlcd.area.y2;
}

static uint8_t* lcd_GetBuff(void)
{
    uint32_t buf1 = (uint32_t) hlcd.draw_buffer1;
    uint32_t buf2 = (uint32_t) hlcd.draw_buffer2;
    uint8_t *retPtr = NULL;

    __disable_irq();

    if (hlcd.buff_to_draw == NULL && hlcd.buff_to_flush == NULL)
    {
        retPtr = hlcd.draw_buffer1;
    }
    else if ((uint32_t) hlcd.buff_to_flush == buf1 && hlcd.buff_to_draw == NULL)
    {
        retPtr = hlcd.draw_buffer2;
    }
    else if ((uint32_t) hlcd.buff_to_flush == buf2 && hlcd.buff_to_draw == NULL)
    {
        retPtr = hlcd.draw_buffer1;
    }
    __enable_irq();

    return retPtr;
}


static uint32_t lcd_CpyToDrawBuffer(uint32_t nbytes, uint32_t rgb888)
{
    uint16_t *fb_ptr = NULL;
    uint32_t npixels;
    hlcd.buff_to_draw = lcd_GetBuff();
    fb_ptr = (uint16_t*) hlcd.buff_to_draw;
    nbytes = ((nbytes > DB_SIZE) ? DB_SIZE : nbytes);
    npixels = BYTES_TO_PIXELS(nbytes);

    if (hlcd.buff_to_draw != NULL)
    {
        for (uint32_t i = 0; i < npixels; i++)
        {
            *fb_ptr = lcd_RGB888toRGB565(rgb888);
            fb_ptr++;
        }

        hlcd.write_length = PIXELS_TO_BYTES(npixels);

        while (!lcd_IsWriteAllowed());

        hlcd.buff_to_flush = hlcd.buff_to_draw;
        hlcd.buff_to_draw = NULL;
        lcd_Flush();

        return PIXELS_TO_BYTES(npixels);
    }

    return 0;
}


static uint8_t lcd_IsWriteAllowed(void)
{
    uint8_t retVal;
    __disable_irq();
    retVal = (hlcd.buff_to_flush) ? FALSE : TRUE;
    __enable_irq();
    return retVal;
}

static void lcd_Flush(void)
{
    lcd_SetDisplArea();
    /* Send command lcd memory write */
    lcd_WriteCmd(ILI9341_GRAM, NULL, 0);
    lcd_WriteDma((uint32_t) hlcd.buff_to_flush, hlcd.write_length);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    UNUSED(hspi);
    if (hlcd.dma_err_cb != NULL)
    {
        hlcd.dma_err_cb();
    }
    else
    {
        while (1);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    hlcd.buff_to_flush = NULL;
    // chunk size
    uint32_t chunk_size;

    /* if current transaction is cmd */
    if (hlcd.spi_dma.datatype == ILI9341_WRITE_CMD)
    {
        //LCD_DCX_HIGH();
        //LCD_CSX_HIGH();
    }

    if (hlcd.spi_dma.chunk_cnt == 0U)
    {
        /* All data chunks are already sent via DMA */

        /* Release CSX pin */
        LCD_CSX_HIGH();

        /* Reset global counters */
        hlcd.spi_dma.src_address = 0U;
        hlcd.spi_dma.nitems = 0U;
    }
    else
    {
        /* Send next chunk of 16-bit data via DMA */

        if (hlcd.spi_dma.nitems > ILI9341_DMA_MAX_ITEMS)
        {
            chunk_size = ILI9341_DMA_MAX_ITEMS;
            hlcd.spi_dma.nitems -= ILI9341_DMA_MAX_ITEMS;
        }
        else
        {
            chunk_size = hlcd.spi_dma.nitems;
        }

        /* Shift source address for the next data chunk */
        hlcd.spi_dma.src_address += ILI9341_DMA_MAX_ITEMS * (hlcd.spi_dma.n_chunks - hlcd.spi_dma.chunk_cnt) * ILI9341_DATAWIDTH;
        hlcd.spi_dma.chunk_cnt--;

        HAL_SPI_Transmit_DMA(hspi, (uint8_t*) hlcd.spi_dma.src_address, chunk_size);
    }

    if (hlcd.dma_cplt_cb != NULL)
    {
        hlcd.dma_cplt_cb();
    }
}
