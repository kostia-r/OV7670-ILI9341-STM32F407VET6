/*
 * ILI9341.c
 * ILI9341 SPI DMA Driver
 * Created on: Aug 5, 2024
 *     Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "ILI9341.h"

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define ILI9341_MADCTL_MY               0x80 // Bottom to top
#define ILI9341_MADCTL_MX               0x40 // Right to left
#define ILI9341_MADCTL_MV               0x20 // Reverse Mode
#define ILI9341_MADCTL_ML               0x10 // LCD refresh Bottom to top
#define ILI9341_MADCTL_RGB              0x00 // Red-Green-Blue pixel order
#define ILI9341_MADCTL_BGR              0x08 // Blue-Green-Red pixel order
#define ILI9341_MADCTL_MH               0x04 // LCD refresh right to left

#define ILI9341_BYTES_TO_PIXELS(bytes)  ((bytes) / ILI9341.pixel_format)
#define ILI9341_PIXELS_TO_BYTES(pixels) ((pixels) * ILI9341.pixel_format)
#define ILI9341_FRAME_SIZE(w, h)        ((w) * (h) * ILI9341.pixel_format)

#ifndef UNUSED
#define UNUSED(x)                       (void)x
#endif

#define HIGH_16(x)                      ((((uint16_t)x) >> 8U) & 0xFFU)
#define LOW_16(x)                       ((((uint16_t)x) >> 0U) & 0xFFU)

#define ILI9341_RESX_HIGH()             SET_BIT(ILI9341_GPIO_PORT_RESX->BSRR,ILI9341_GPIO_PIN_RESX)
#define ILI9341_RESX_LOW()              SET_BIT(ILI9341_GPIO_PORT_RESX->BSRR,((uint32_t)ILI9341_GPIO_PIN_RESX << 16U))

#if (ILI9341_SPI_CS_HW_MANAGE == 1)
#define ILI9341_CSX_HIGH()
#define ILI9341_CSX_LOW()
#else
#define ILI9341_CSX_HIGH()              SET_BIT(ILI9341_GPIO_PORT_CSX->BSRR,ILI9341_GPIO_PIN_CSX)
#define ILI9341_CSX_LOW()               SET_BIT(ILI9341_GPIO_PORT_CSX->BSRR,((uint32_t)ILI9341_GPIO_PIN_CSX << 16U))
#endif

#define ILI9341_DCX_HIGH()              SET_BIT(ILI9341_GPIO_PORT_DCX->BSRR,ILI9341_GPIO_PIN_DCX)
#define ILI9341_DCX_LOW()               SET_BIT(ILI9341_GPIO_PORT_DCX->BSRR,((uint32_t)ILI9341_GPIO_PIN_DCX << 16U))

/* Set SPI data width to 8 bits if 16 bits has been already set: FOR STM32F4 */
#define ILI9341_SPI_SET_8_BIT()         do{if(READ_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_DFF))\
    {CLEAR_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_SPE);/* Disable SPI */\
    CLEAR_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_DFF);/* Set 8-bit data width */\
    SET_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_SPE);/* Enable SPI */}}while(0U)

/* Set SPI data width to 16 bits if 8 bits has been already set: FOR STM32F4 */
#define ILI9341_SPI_SET_16_BIT()        do{if(!READ_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_DFF))\
    {CLEAR_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_SPE);/* Disable SPI */\
    SET_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_DFF);/* Set 16-bit data width */\
    SET_BIT(ILI9341.hspi->Instance->CR1, SPI_CR1_SPE);/* Enable SPI */}}while(0U)

// TODO: add timeouts
#define ILI9341_CHECK_SPI(spi_handler_ptr)         do{\
    while(HAL_SPI_STATE_READY != HAL_SPI_GetState(spi_handler_ptr));\
    }while(0)

#define ILI9341_DATAWIDTH_8BIT          (1U)
#define ILI9341_DATAWIDTH_16BIT         (2U)
#define ILI9341_DATAWIDTH               ILI9341_DATAWIDTH_16BIT

#define ILI9341_DMA_MAX_ITEMS           (0xFFFFU)

/* Data buffer size */
#define DB_SIZE                         (10UL * 1024UL)

/******************************************************************************
 *                            ILI9341 REGISTERS                               *
 ******************************************************************************/

/* Level 1 Commands */
#define ILI9341_SWRESET                  0x01U   /* Software Reset */
#define ILI9341_READ_DISPLAY_ID          0x04U   /* Read display identification information */
#define ILI9341_RDDST                    0x09U   /* Read Display Status */
#define ILI9341_RDDPM                    0x0AU   /* Read Display Power Mode */
#define ILI9341_RDDMADCTL                0x0BU   /* Read Display MADCTL */
#define ILI9341_RDDCOLMOD                0x0CU   /* Read Display Pixel Format */
#define ILI9341_RDDIM                    0x0DU   /* Read Display Image Format */
#define ILI9341_RDDSM                    0x0EU   /* Read Display Signal Mode */
#define ILI9341_RDDSDR                   0x0FU   /* Read Display Self-Diagnostic Result */
#define ILI9341_SPLIN                    0x10U   /* Enter Sleep Mode */
#define ILI9341_SLEEP_OUT                0x11U   /* Sleep out register */
#define ILI9341_PTLON                    0x12U   /* Partial Mode ON */
#define ILI9341_NORMAL_MODE_ON           0x13U   /* Normal Display Mode ON */
#define ILI9341_DINVOFF                  0x20U   /* Display Inversion OFF */
#define ILI9341_DINVON                   0x21U   /* Display Inversion ON */
#define ILI9341_GAMMA                    0x26U   /* Gamma register */
#define ILI9341_DISPLAY_OFF              0x28U   /* Display off register */
#define ILI9341_DISPLAY_ON               0x29U   /* Display on register */
#define ILI9341_CASET                    0x2AU   /* Column address register */
#define ILI9341_RASET                    0x2BU   /* Page address register */
#define ILI9341_GRAM                     0x2CU   /* GRAM register */
#define ILI9341_RGBSET                   0x2DU   /* Color SET */
#define ILI9341_RAMRD                    0x2EU   /* Memory Read */
#define ILI9341_PLTAR                    0x30U   /* Partial Area */
#define ILI9341_VSCRDEF                  0x33U   /* Vertical Scrolling Definition */
#define ILI9341_TEOFF                    0x34U   /* Tearing Effect Line OFF */
#define ILI9341_TEON                     0x35U   /* Tearing Effect Line ON */
#define ILI9341_MAC                      0x36U   /* Memory Access Control register*/
#define ILI9341_VSCRSADD                 0x37U   /* Vertical Scrolling Start Address */
#define ILI9341_IDMOFF                   0x38U   /* Idle Mode OFF */
#define ILI9341_IDMON                    0x39U   /* Idle Mode ON */
#define ILI9341_PIXEL_FORMAT             0x3AU   /* Pixel Format register */
#define ILI9341_WRITE_MEM_CONTINUE       0x3CU   /* Write Memory Continue */
#define ILI9341_READ_MEM_CONTINUE        0x3EU   /* Read Memory Continue */
#define ILI9341_SET_TEAR_SCANLINE        0x44U   /* Set Tear Scanline */
#define ILI9341_GET_SCANLINE             0x45U   /* Get Scanline */
#define ILI9341_WDB                      0x51U   /* Write Brightness Display register */
#define ILI9341_RDDISBV                  0x52U   /* Read Display Brightness */
#define ILI9341_WCD                      0x53U   /* Write Control Display register*/
#define ILI9341_RDCTRLD                  0x54U   /* Read CTRL Display */
#define ILI9341_WRCABC                   0x55U   /* Write Content Adaptive Brightness Control */
#define ILI9341_RDCABC                   0x56U   /* Read Content Adaptive Brightness Control */
#define ILI9341_WRITE_CABC               0x5EU   /* Write CABC Minimum Brightness */
#define ILI9341_READ_CABC                0x5FU   /* Read CABC Minimum Brightness */
#define ILI9341_READ_ID1                 0xDAU   /* Read ID1 */
#define ILI9341_READ_ID2                 0xDBU   /* Read ID2 */
#define ILI9341_READ_ID3                 0xDCU   /* Read ID3 */

/* Level 2 Commands */
#define ILI9341_RGB_INTERFACE            0xB0U   /* RGB Interface Signal Control */
#define ILI9341_FRMCTR1                  0xB1U   /* Frame Rate Control (In Normal Mode) */
#define ILI9341_FRMCTR2                  0xB2U   /* Frame Rate Control (In Idle Mode) */
#define ILI9341_FRMCTR3                  0xB3U   /* Frame Rate Control (In Partial Mode) */
#define ILI9341_INVTR                    0xB4U   /* Display Inversion Control */
#define ILI9341_BPC                      0xB5U   /* Blanking Porch Control register */
#define ILI9341_DFC                      0xB6U   /* Display Function Control register */
#define ILI9341_ETMOD                    0xB7U   /* Entry Mode Set */
#define ILI9341_BACKLIGHT1               0xB8U   /* Backlight Control 1 */
#define ILI9341_BACKLIGHT2               0xB9U   /* Backlight Control 2 */
#define ILI9341_BACKLIGHT3               0xBAU   /* Backlight Control 3 */
#define ILI9341_BACKLIGHT4               0xBBU   /* Backlight Control 4 */
#define ILI9341_BACKLIGHT5               0xBCU   /* Backlight Control 5 */
#define ILI9341_BACKLIGHT7               0xBEU   /* Backlight Control 7 */
#define ILI9341_BACKLIGHT8               0xBFU   /* Backlight Control 8 */
#define ILI9341_POWER1                   0xC0U   /* Power Control 1 register */
#define ILI9341_POWER2                   0xC1U   /* Power Control 2 register */
#define ILI9341_VCOM1                    0xC5U   /* VCOM Control 1 register */
#define ILI9341_VCOM2                    0xC7U   /* VCOM Control 2 register */
#define ILI9341_NVMWR                    0xD0U   /* NV Memory Write */
#define ILI9341_NVMPKEY                  0xD1U   /* NV Memory Protection Key */
#define ILI9341_RDNVM                    0xD2U   /* NV Memory Status Read */
#define ILI9341_READ_ID4                 0xD3U   /* Read ID4 */
#define ILI9341_PGAMMA                   0xE0U   /* Positive Gamma Correction register */
#define ILI9341_NGAMMA                   0xE1U   /* Negative Gamma Correction register */
#define ILI9341_DGAMCTRL1                0xE2U   /* Digital Gamma Control 1 */
#define ILI9341_DGAMCTRL2                0xE3U   /* Digital Gamma Control 2 */
#define ILI9341_INTERFACE                0xF6U   /* Interface control register */

/* Extend register commands */
#define ILI9341_POWERA                   0xCBU   /* Power control A register */
#define ILI9341_POWERB                   0xCFU   /* Power control B register */
#define ILI9341_DTCA                     0xE8U   /* Driver timing control A */
#define ILI9341_DTCB                     0xEAU   /* Driver timing control B */
#define ILI9341_POWER_SEQ                0xEDU   /* Power on sequence register */
#define ILI9341_3GAMMA_EN                0xF2U   /* 3 Gamma enable register */
#define ILI9341_PRC                      0xF7U   /* Pump ratio control register */

/******************************************************************************
 *                           LOCAL DATA TYPES                                 *
 ******************************************************************************/

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
} spi_dma_runtime_t;

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static volatile struct
{
    uint8_t           orientation;
    uint8_t           pixel_format;
    uint8_t           *draw_buffer1;
    uint8_t           *draw_buffer2;
    uint32_t          write_length;
    const uint8_t     *buff_to_draw;
    const uint8_t     *buff_to_flush;
    lcd_area_t        area;
    spi_dma_runtime_t spi_dma;
    ILI9341_FncPtr_t  dma_cplt_cb;
    ILI9341_FncPtr_t  dma_err_cb;
    SPI_HandleTypeDef *hspi;
} ILI9341;

enum
{
    FALSE, TRUE
};


uint8_t bsp_db[DB_SIZE];
uint8_t bsp_wb[DB_SIZE];

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static void lcd_Reset(void);
static void lcd_Config(void);
static void lcd_SetOrientation(uint8_t orientation);
static void lcd_BufferInit(void);
static uint8_t lcd_IsWriteAllowed(void);
static void lcd_SetDisplArea(void);
static uint8_t* lcd_GetBuff(void);
static void lcd_Flush(void);
static void lcd_WriteCmd(uint8_t cmd, const uint8_t *params, uint32_t param_len);
static void lcd_WriteDma(uint32_t src_addr, uint32_t nbytes);
static uint32_t lcd_CpyToDrawBuffer(uint32_t nbytes, uint32_t rgb888);
static void lcd_MakeArea(uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height);
static uint16_t lcd_RGB888toRGB565(uint32_t rgb888);
static HAL_StatusTypeDef lcd_SPI_Send(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len, uint32_t timeout);
//static HAL_StatusTypeDef lcd_SPI_Read(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len, uint32_t timeout);
static HAL_StatusTypeDef lcd_SPI_SendRead(SPI_HandleTypeDef *hspi, uint8_t *rxData, uint8_t *txData, uint8_t len, uint32_t timeout);
/******************************************************************************
 *                              GLOBAL FUNCTIONS                              *
 ******************************************************************************/

void ILI9341_Init(SPI_HandleTypeDef *spi_handle, ILI9341_PixFormat_t pixFormat)
{
    ILI9341.hspi = spi_handle;
    ILI9341.orientation = ILI9341_ORIENTATION;
    ILI9341.pixel_format = pixFormat;
    lcd_Reset();
    lcd_Config();
    ILI9341.area.x1 = 0;
    ILI9341.area.x2 = ILI9341_ACTIVE_WIDTH - 1;
    ILI9341.area.y1 = 0;
    ILI9341.area.y2 = ILI9341_ACTIVE_HEIGHT - 1;
    lcd_SetDisplArea();
    lcd_SetOrientation(ILI9341.orientation);
    lcd_BufferInit();
}

void ILI9341_RegisterCallback(ILI9341_CB_t cb_type, ILI9341_FncPtr_t fnc_ptr)
{
    switch (cb_type)
    {
        case ILI9341_TC_CBK:
            ILI9341.dma_cplt_cb = fnc_ptr;
            break;

        case ILI9341_ERR_CBK:
            ILI9341.dma_err_cb = fnc_ptr;
            break;

        default:
            break;
    }
}

void ILI9341_DrawFrame(const uint8_t *fb_addr, uint32_t nbytes)
{
    ILI9341.area.x1 = 0U;
    ILI9341.area.x2 = ILI9341_ACTIVE_WIDTH - 1;
    ILI9341.area.y1 = 0U;
    ILI9341.area.y2 = ILI9341_ACTIVE_HEIGHT - 1;
    ILI9341.buff_to_flush = fb_addr;
    ILI9341.write_length = nbytes;
    lcd_Flush();
}

void ILI9341_DrawCrop(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
    ILI9341.area.x1 = x1;
    ILI9341.area.x2 = x2;
    ILI9341.area.y1 = y1;
    ILI9341.area.y2 = y2;
    ILI9341.buff_to_flush = buffer;
    ILI9341.write_length = nbytes;
    lcd_Flush();
}

void* ILI9341_GetDrawBuffer1Addr(void)
{
    return (void*) ILI9341.draw_buffer1;
}

void* ILI9341_GetDrawBuffer2Addr(void)
{
    return (void*) ILI9341.draw_buffer2;
}

void ILI9341_SetBackgroundColor(uint32_t rgb888)
{
    ILI9341_FillRect(rgb888, 0, ILI9341_ACTIVE_WIDTH, 0, ILI9341_ACTIVE_HEIGHT);
}

void ILI9341_FillRect(uint32_t rgb888, uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height)
{

    uint32_t total_bytes_to_write = 0;
    uint32_t bytes_sent_so_far = 0;
    uint32_t remaining_bytes = 0;
    uint32_t npix;
    uint32_t pixels_sent = 0;
    uint32_t x1, y1;
    uint32_t pixel_per_line = x_width;

    if ((x_start + x_width) > ILI9341_ACTIVE_WIDTH)
        return;

    if ((y_start + y_height) > ILI9341_ACTIVE_HEIGHT)
        return;

    // calculate total number of bytes written in to DB
    total_bytes_to_write = ILI9341_FRAME_SIZE(x_width, y_height);
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
            npix = ILI9341_BYTES_TO_PIXELS(remaining_bytes);
        }

        bytes_sent_so_far += lcd_CpyToDrawBuffer(ILI9341_PIXELS_TO_BYTES(npix), rgb888);
        pixels_sent = ILI9341_BYTES_TO_PIXELS(bytes_sent_so_far);
        remaining_bytes = total_bytes_to_write - bytes_sent_so_far;
    }
}



void ILI9341_Read_GRAM(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t *buffer)
{
    // Calculate the total number of pixels
    uint32_t num_pixels = (x2 - x1 + 1) * (y2 - y1 + 1);
    // placeholder for 2 18-bits RGB666 pixels, since ILI9341 is able to return only in this format
    uint8_t pixels_data[6] = { 0x00U };

    // Set the address window for the rectangle
    ILI9341.area.x1 = x1;
    ILI9341.area.x2 = x2;
    ILI9341.area.y1 = y1;
    ILI9341.area.y2 = y2;
    lcd_SetDisplArea();

    ILI9341_SPI_SET_8_BIT();

    // Pull CSX low before issuing the command
    ILI9341_CSX_LOW();
    // Switch to Command Mode
    ILI9341_DCX_LOW();

    uint8_t cmd = ILI9341_RAMRD;
    lcd_SPI_Send(ILI9341.hspi, &cmd, sizeof(cmd), HAL_MAX_DELAY);

    // Switch to Data Mode
    ILI9341_DCX_HIGH();

    // Generate dummy byte
    uint8_t dummy = 0x00U;
    lcd_SPI_Send(ILI9341.hspi, &dummy, sizeof(dummy), HAL_MAX_DELAY);

    while (num_pixels > 0U)
    {
        lcd_SPI_SendRead(ILI9341.hspi, pixels_data, pixels_data, sizeof(pixels_data), HAL_MAX_DELAY);
        /* Convert two pixels RGB666 to RGB888 */
        for (uint8_t i = 0U; i < sizeof(pixels_data); i += 3)
        {
            /* RGB666 -> RGB888 */
            *buffer++ = (pixels_data[i] & 0xFC) | (pixels_data[i] >> 6);  // Red
            *buffer++ = (pixels_data[i + 1] & 0xFC) | (pixels_data[i + 1] >> 6); // Green
            *buffer++ = (pixels_data[i + 2] & 0xFC) | (pixels_data[i + 2] >> 6); // Blue
        }

        num_pixels -= 2U;
    }

    // Pull CSX high manually after the read
    ILI9341_CSX_HIGH();
}

/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/

static void lcd_SetOrientation(uint8_t orientation)
{
    uint8_t param;

    switch (orientation)
    {
        case ILI9341_LANDSCAPE:
        {
            /*Memory Access Control LANDSCAPE setting */
            param = ILI9341_MADCTL_MV | ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
            break;
        }

        case ILI9341_PORTRAIT:
        {
            /* Memory Access Control PORTRAIT setting */
            param = ILI9341_MADCTL_MY | ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
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
    ILI9341_RESX_LOW();
    ILI9341_DELAY(50);
    ILI9341_RESX_HIGH();
    ILI9341_DELAY(50);
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

    ILI9341_DELAY(100);

    /* Display on */
    lcd_WriteCmd(ILI9341_DISPLAY_ON, NULL, 0U);
}

static void lcd_WriteCmd(uint8_t cmd, const uint8_t *params, uint32_t param_len)
{
    //ILI9341_CHECK_SPI(ILI9341.hspi);

    /* Set SPI data width as 8 bits */
    ILI9341_SPI_SET_8_BIT();

    ILI9341_CSX_LOW(); // SPI CS
    ILI9341_DCX_LOW(); // for commands

    /* HAL implementation: */
    //HAL_SPI_Transmit(ILI9341.hspi, &cmd, 1U, HAL_MAX_DELAY);

    /* HAL/CMSIS implementation: */

    /* Check if the SPI is already enabled */
    if ((ILI9341.hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(ILI9341.hspi);
    }
    // wait till TXE becomes 1 (buffer empty)
    while (!__HAL_SPI_GET_FLAG(ILI9341.hspi, SPI_FLAG_TXE));
    WRITE_REG(ILI9341.hspi->Instance->DR, cmd);
    while (!__HAL_SPI_GET_FLAG(ILI9341.hspi, SPI_FLAG_TXE));
    // make sure that command has been sent - check BUSY bit, wait if it is 1
    while (READ_BIT(ILI9341.hspi->Instance->SR, SPI_SR_BSY));

    // clear OVR flag
    __HAL_SPI_CLEAR_OVRFLAG(ILI9341.hspi);

    ILI9341_DCX_HIGH();// for commands

    if (params != NULL)
    {
        //ILI9341_CHECK_SPI(ILI9341.hspi);
        //HAL_SPI_Transmit(ILI9341.hspi, (uint8_t*) params, param_len, HAL_MAX_DELAY);

        for (uint32_t i = 0; i < param_len; i++)
        {
            while (!__HAL_SPI_GET_FLAG(ILI9341.hspi, SPI_FLAG_TXE));
            WRITE_REG(ILI9341.hspi->Instance->DR, params[i]);
        }

        while (!__HAL_SPI_GET_FLAG(ILI9341.hspi, SPI_FLAG_TXE));
        while (READ_BIT(ILI9341.hspi->Instance->SR, SPI_SR_BSY));
        // clear OVR flag
       __HAL_SPI_CLEAR_OVRFLAG(ILI9341.hspi);
    }

    ILI9341_CSX_HIGH(); // SPI CS
}

static void lcd_WriteDma(uint32_t src_addr, uint32_t nbytes)
{
    ILI9341_CHECK_SPI(ILI9341.hspi);

    /* Set SPI data width as 16 bits */
    ILI9341_SPI_SET_16_BIT();

    uint32_t nitems = nbytes / ILI9341_DATAWIDTH;

    // determine how many chunks of input buffer need to be sent
    ILI9341.spi_dma.n_chunks = (nitems % ILI9341_DMA_MAX_ITEMS != 0U) ?
                    (nitems / ILI9341_DMA_MAX_ITEMS + 1UL) :
                    (nitems / ILI9341_DMA_MAX_ITEMS);
    // assign DMA iteration counter
    ILI9341.spi_dma.chunk_cnt = ILI9341.spi_dma.n_chunks;
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

    ILI9341.spi_dma.nitems = nitems;
    /* Shift source address */
    ILI9341.spi_dma.src_address = src_addr + ILI9341_DMA_MAX_ITEMS * (ILI9341.spi_dma.n_chunks - ILI9341.spi_dma.chunk_cnt) * ILI9341_DATAWIDTH;
    ILI9341.spi_dma.chunk_cnt--;

    ILI9341_CSX_LOW(); // SPI CS
    HAL_SPI_Transmit_DMA(ILI9341.hspi, (uint8_t*) ILI9341.spi_dma.src_address, chunk_size);
}

static void lcd_SetDisplArea(void)
{
    uint8_t params[4];
    /*Column address set(2Ah) */
    params[0] = HIGH_16(ILI9341.area.x1);
    params[1] = LOW_16(ILI9341.area.x1);
    params[2] = HIGH_16(ILI9341.area.x2);
    params[3] = LOW_16(ILI9341.area.x2);
    lcd_WriteCmd(ILI9341_CASET, params, 4U);

    params[0] = HIGH_16(ILI9341.area.y1);
    params[1] = LOW_16(ILI9341.area.y1);
    params[2] = HIGH_16(ILI9341.area.y2);
    params[3] = LOW_16(ILI9341.area.y2);
    lcd_WriteCmd(ILI9341_RASET, params, 4U);
}

static void lcd_BufferInit(void)
{
    ILI9341.draw_buffer1 = bsp_db;
    ILI9341.draw_buffer2 = bsp_wb;
    ILI9341.buff_to_draw = NULL;
    ILI9341.buff_to_flush = NULL;
}

static uint16_t lcd_RGB888toRGB565(uint32_t rgb888)
{
    uint16_t r, g, b;
    r = (rgb888 >> 19) & 0x1FU;
    g = (rgb888 >> 10) & 0x3FU;
    b = (rgb888 >> 3) & 0x1FU;
    return (uint16_t) ((r << 11) | (g << 5) | b);
}

static void lcd_MakeArea(uint32_t x_start, uint32_t x_width, uint32_t y_start, uint32_t y_height)
{

    uint16_t lcd_total_width, lcd_total_height;

    lcd_total_width = ILI9341_ACTIVE_WIDTH - 1;
    lcd_total_height = ILI9341_ACTIVE_HEIGHT - 1;

    ILI9341.area.x1 = x_start;
    ILI9341.area.x2 = x_start + x_width - 1;
    ILI9341.area.y1 = y_start;
    ILI9341.area.y2 = y_start + y_height - 1;

    ILI9341.area.x2 = (ILI9341.area.x2 > lcd_total_width) ? lcd_total_width : ILI9341.area.x2;
    ILI9341.area.y2 = (ILI9341.area.y2 > lcd_total_height) ? lcd_total_height : ILI9341.area.y2;
}

static uint8_t* lcd_GetBuff(void)
{
    uint32_t buf1 = (uint32_t) ILI9341.draw_buffer1;
    uint32_t buf2 = (uint32_t) ILI9341.draw_buffer2;
    uint8_t *retPtr = NULL;

    __disable_irq();

    if (ILI9341.buff_to_draw == NULL && ILI9341.buff_to_flush == NULL)
    {
        retPtr = ILI9341.draw_buffer1;
    }
    else if ((uint32_t) ILI9341.buff_to_flush == buf1 && ILI9341.buff_to_draw == NULL)
    {
        retPtr = ILI9341.draw_buffer2;
    }
    else if ((uint32_t) ILI9341.buff_to_flush == buf2 && ILI9341.buff_to_draw == NULL)
    {
        retPtr = ILI9341.draw_buffer1;
    }
    __enable_irq();

    return retPtr;
}


static uint32_t lcd_CpyToDrawBuffer(uint32_t nbytes, uint32_t rgb888)
{
    uint16_t *fb_ptr = NULL;
    uint32_t npixels;
    ILI9341.buff_to_draw = lcd_GetBuff();
    fb_ptr = (uint16_t*) ILI9341.buff_to_draw;
    nbytes = ((nbytes > DB_SIZE) ? DB_SIZE : nbytes);
    npixels = ILI9341_BYTES_TO_PIXELS(nbytes);

    if (ILI9341.buff_to_draw != NULL)
    {
        for (uint32_t i = 0; i < npixels; i++)
        {
            *fb_ptr = lcd_RGB888toRGB565(rgb888);
            fb_ptr++;
        }

        ILI9341.write_length = ILI9341_PIXELS_TO_BYTES(npixels);

        while (!lcd_IsWriteAllowed());

        ILI9341.buff_to_flush = ILI9341.buff_to_draw;
        ILI9341.buff_to_draw = NULL;
        lcd_Flush();

        return ILI9341_PIXELS_TO_BYTES(npixels);
    }

    return 0;
}


static uint8_t lcd_IsWriteAllowed(void)
{
    uint8_t retVal;
    __disable_irq();
    retVal = (ILI9341.buff_to_flush) ? FALSE : TRUE;
    __enable_irq();
    return retVal;
}

static void lcd_Flush(void)
{
    lcd_SetDisplArea();
    /* Send command lcd memory write */
    lcd_WriteCmd(ILI9341_GRAM, NULL, 0);
    lcd_WriteDma((uint32_t) ILI9341.buff_to_flush, ILI9341.write_length);
}

static HAL_StatusTypeDef lcd_SPI_Send(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len, uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    /* Check if the SPI is already enabled */
    if (!READ_BIT(hspi->Instance->CR1, SPI_CR1_SPE))
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

    while (len > 0U)
    {
        while (!__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) && timeout--);

        if (timeout == 0U)
        {
            ret = HAL_TIMEOUT;
            break;
        }
        else
        {
            if (READ_BIT(hspi->Instance->CR1, SPI_CR1_DFF))
            {
                /* 16-bit mode */
                WRITE_REG(hspi->Instance->DR, *((uint16_t*) data++));
                len--;
                len--;
            }
            else
            {
                /* 8-bit mode */
                WRITE_REG(hspi->Instance->DR, *data++);
                len--;
            }

            ret = HAL_OK;
        }
    }

    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) && timeout--);
    __HAL_SPI_CLEAR_OVRFLAG(hspi);

    ret = (timeout == 0U) ? HAL_TIMEOUT: HAL_OK;

    return ret;
}

//static HAL_StatusTypeDef lcd_SPI_Read(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len, uint32_t timeout)
//{
//    HAL_StatusTypeDef ret = HAL_ERROR;
//
//    /* Check if the SPI is already enabled */
//    if (!READ_BIT(hspi->Instance->CR1, SPI_CR1_SPE))
//    {
//        /* Enable SPI peripheral */
//        __HAL_SPI_ENABLE(hspi);
//    }
//
//    while (len > 0U)
//    {
//        while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) == 0U && timeout--);
//
//        if (timeout == 0U)
//        {
//            ret = HAL_TIMEOUT;
//            break;
//        }
//        else
//        {
//            if (READ_BIT(hspi->Instance->CR1, SPI_CR1_DFF))
//            {
//                /* 16-bit mode */
//                *((uint16_t*) data++) = READ_REG(hspi->Instance->DR);
//                len--;
//                len--;
//            }
//            else
//            {
//                /* 8-bit mode */
//                *data++ = READ_REG(hspi->Instance->DR);
//                len--;
//            }
//
//            ret = HAL_OK;
//        }
//    }
//
//    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) && timeout--);
//    __HAL_SPI_CLEAR_OVRFLAG(hspi);
//
//    ret = (timeout == 0U) ? HAL_TIMEOUT: HAL_OK;
//
//    return ret;
//}

static HAL_StatusTypeDef lcd_SPI_SendRead(SPI_HandleTypeDef *hspi, uint8_t *rxData, uint8_t *txData, uint8_t len, uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;
    uint32_t local_timeout;

    /* Enable SPI peripheral if not already enabled */
    if (!READ_BIT(hspi->Instance->CR1, SPI_CR1_SPE))
    {
        __HAL_SPI_ENABLE(hspi);
    }

    while (len > 0U)
    {
        local_timeout = timeout;

        /* Wait for TXE flag (ready to transmit) */
        while (!__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) && local_timeout--);

        if (local_timeout == 0U)
        {
            ret = HAL_TIMEOUT;
            break;
        }

        /* Write data to SPI Data Register */
        if (READ_BIT(hspi->Instance->CR1, SPI_CR1_DFF))
        {
            /* 16-bit mode */
            *((__IO uint16_t*) &hspi->Instance->DR) = *((uint16_t*) txData);
            txData += 2U;
            len -= 2U;
        }
        else
        {
            /* 8-bit mode */
            *((__IO uint8_t*) &hspi->Instance->DR) = *txData++;
            len--;
        }

        /* Wait for RXNE flag (data available) */
        local_timeout = timeout;
        while (!__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) && local_timeout--);

        if (local_timeout == 0U)
        {
            ret = HAL_TIMEOUT;
            break;
        }

        /* Read received data */
        if (READ_BIT(hspi->Instance->CR1, SPI_CR1_DFF))
        {
            *((uint16_t*) rxData) = *((__IO uint16_t*) &hspi->Instance->DR);
            rxData += 2U;
        }
        else
        {
            *rxData++ = *((__IO uint8_t*) &hspi->Instance->DR);
        }
    }

    /* Check BSY flag (SPI busy) */
    local_timeout = timeout;
    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) && local_timeout--);

    if (local_timeout == 0U)
    {
        ret = HAL_TIMEOUT;
    }

    /* Clear any overrun flag that may have occurred */
    __HAL_SPI_CLEAR_OVRFLAG(hspi);

    return ret;
}



/******************************************************************************
 *                               HAL CALLBACKS                                *
 ******************************************************************************/

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    UNUSED(hspi);
    if (ILI9341.dma_err_cb != NULL)
    {
        ILI9341.dma_err_cb();
    }
    else
    {
        while (1);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    //DEBUG_TIMEMEAS_START();
    ILI9341.buff_to_flush = NULL;
    uint32_t chunk_size, chunk_cnt, src_address, nitems, n_chunks;
    uint8_t needToCont;

    /* Read shared data */
    __disable_irq();
    src_address = ILI9341.spi_dma.src_address;
    nitems = ILI9341.spi_dma.nitems;
    chunk_cnt = ILI9341.spi_dma.chunk_cnt;
    n_chunks = ILI9341.spi_dma.n_chunks;
    __enable_irq();

    if (chunk_cnt == 0U)
    {
        /* All data chunks are already sent via DMA */

        /* Release CSX pin */
        ILI9341_CSX_HIGH();  // SPI CS

        /* Reset global counters */
        src_address = 0U;
        nitems = 0U;
        needToCont = FALSE;

        DEBUG_MEAS_FPS(ILI9341.area.x1 == 0U && ILI9341.area.y1 == 1U);
    }
    else
    {
        /* Send next chunk of 16-bit data via DMA */
        if (nitems > ILI9341_DMA_MAX_ITEMS)
        {
            chunk_size = ILI9341_DMA_MAX_ITEMS;
            nitems -= ILI9341_DMA_MAX_ITEMS;
        }
        else
        {
            chunk_size = nitems;
        }

        /* Shift source address for the next data chunk */
        src_address += ILI9341_DMA_MAX_ITEMS * (n_chunks - chunk_cnt) * ILI9341_DATAWIDTH;
        chunk_cnt--;

        needToCont = TRUE; // needs to call HAL_SPI_Transmit_DMA()
    }

    /* Write shared data */
    __disable_irq();
    ILI9341.spi_dma.src_address = src_address;
    ILI9341.spi_dma.nitems = nitems;
    ILI9341.spi_dma.chunk_cnt = chunk_cnt;
    ILI9341.spi_dma.n_chunks = n_chunks;
    __enable_irq();

    if (ILI9341.dma_cplt_cb != NULL)
    {
        /* Call external callback */
        ILI9341.dma_cplt_cb();
    }

    if (needToCont == TRUE)
    {
        /* Draw next data chunk */
        HAL_SPI_Transmit_DMA(hspi, (uint8_t*) src_address, chunk_size);
    }

    //DEBUG_TIMEMEAS_END();
}
