
/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "OV7670.h"
#include "OV7670_Reg.h"

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define OV7670_WIDTH_SIZE_BYTES    (OV7670_QVGA_WIDTH * 2U)
#define OV7670_WIDTH_SIZE_WORDS    (OV7670_WIDTH_SIZE_BYTES / 4U)
#define OV7670_HEIGHT_SIZE_BYTES   (OV7670_QVGA_HEIGHT * 2U)
#define OV7670_HEIGHT_SIZE_WORDS   (OV7670_HEIGHT_SIZE_BYTES / 4U)
#define OV7670_FRAME_SIZE_BYTES    (OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT * 2U)
#define OV7670_FRAME_SIZE_WORDS    (OV7670_FRAME_SIZE_BYTES / 4U)

#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
/* For double stream-line buffer */
#define OV7670_BUFFER_SIZE         (OV7670_WIDTH_SIZE_BYTES * 2U)
#define OV7670_DMA_DATA_LEN        (OV7670_WIDTH_SIZE_WORDS)
/* Macro for update address to second half of double-line buffer */
#define OV7670_SWITCH_BUFFER()     ((OV7670.buffer_addr != (uint32_t)buffer) ?\
        (OV7670.buffer_addr + (OV7670_BUFFER_SIZE)/ 2U) : (uint32_t)buffer)
#define OV7670_RESET_BUFFER_ADDR() (uint32_t)buffer

#else
/* For whole-size snapshot buffer */
#define OV7670_BUFFER_SIZE         (OV7670_FRAME_SIZE_BYTES)
#define OV7670_DMA_DATA_LEN        (OV7670_FRAME_SIZE_WORDS)
#endif

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static struct
{
    /* HAL peripheral handlers */
    DCMI_HandleTypeDef  *hdcmi;
    I2C_HandleTypeDef   *hi2c;
    TIM_HandleTypeDef   *htim;
    uint32_t            tim_ch;
    /* Requested mode */
    uint32_t            mode;
    /* Address of the buffer */
    volatile uint32_t   buffer_addr;
    /* Image line counter */
    volatile uint32_t   lineCnt;
    /* Draw line callback prototypes */
} OV7670;

/* Image buffer */
static uint8_t buffer[OV7670_BUFFER_SIZE];

/* Draw callback prototypes */
static void (*drawLine_cb)(const uint8_t *buffer, uint32_t nbytes, uint16_t x1,
        uint16_t x2, uint16_t y);
static void (*drawFrame_cb)(const uint8_t *buffer, uint32_t nbytes);

/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static HAL_StatusTypeDef SCCB_Write(uint8_t regAddr, uint8_t data);
static HAL_StatusTypeDef SCCB_Read(uint8_t regAddr, uint8_t *data);

/******************************************************************************
 *                              GLOBAL FUNCTIONS                              *
 ******************************************************************************/

void OV7670_Init(DCMI_HandleTypeDef *hdcmi, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim, uint32_t tim_ch)
{
    OV7670.hdcmi = hdcmi;
    OV7670.hi2c = hi2c;
    OV7670.htim = htim;
    OV7670.tim_ch = tim_ch;

    /* PWDN to LOW */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_PWDN, OV7670_GPIO_PIN_PWDN, GPIO_PIN_RESET);
    /* RET pin to LOW */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_RET, OV7670_GPIO_PIN_RET, GPIO_PIN_RESET);
    HAL_Delay(100);
    /* RET pin to HIGH */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_RET, OV7670_GPIO_PIN_RET, GPIO_PIN_SET);
    HAL_Delay(100);

    /* Do camera reset */
    SCCB_Write(0x12, 0x80);
    HAL_Delay(30);

    /* Get camera ID */
    uint8_t buf[4];
    SCCB_Read(0x0b, buf);
    DEBUG_LOG("[OV7670] dev id = 0x%02X", buf[0]);

    /* Stop DCMI periphery */
    HAL_DCMI_Stop(OV7670.hdcmi);

    /* Do camera reset */
    SCCB_Write(0x12, 0x80);
    HAL_Delay(30);

    /* TODO: OV7670_COLOR_MODE type is not considering
     * - fix this in the code right below */
    /* Do camera configuration */
    for (int i = 0; OV7670_reg[i][0] != REG_BATT; i++)
    {
        SCCB_Write(OV7670_reg[i][0], OV7670_reg[i][1]);
        HAL_Delay(1);
    }

    /* Initialize buffer address */
    OV7670.buffer_addr = (uint32_t) buffer;
}

void OV7670_Start(uint32_t capMode)
{
    /* Update requested mode */
    OV7670.mode = capMode;
#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
    /* Reset buffer address */
    OV7670.buffer_addr = OV7670_RESET_BUFFER_ADDR();
#endif
    /* Start camera XLK signal to capture the image data */
    HAL_TIM_OC_Start(OV7670.htim, OV7670.tim_ch);
    /* Start DCMI capturing. TODO: check for full-size QVGA buffer mode */
    HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_CONTINUOUS, OV7670.buffer_addr,
            OV7670_DMA_DATA_LEN);
}

void OV7670_Stop(void)
{
    HAL_DCMI_Stop(OV7670.hdcmi);
}

void OV7670_RegisterCallback(OV7670_CB_t cb_type, OV7670_FncPtr_t fnc_ptr)
{
    switch (cb_type)
    {
        case OV7670_DRAWLINE_CALLBACK:
        {
            drawLine_cb = fnc_ptr;
            break;
        }

        case OV7670_DRAWFRAME_CALLBACK:
        {
            drawFrame_cb = fnc_ptr;
            break;
        }

        default:
            break;
    }
}


/******************************************************************************
 *                               HAL CALLBACKS                                *
 ******************************************************************************/

#if (OV7670_STREAM_MODE == OV7670_SRTEAM_MODE_BY_FRAME)

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    /* Disable DCMI Camera interface */
    HAL_DCMI_Stop(OV7670.hdcmi);

    /* Stop camera XLK signal until captured image data is drawn */
    HAL_TIM_OC_Stop(OV7670.htim, OV7670.tim_ch);

    /* Call Display flush function */
    if (drawFrame_cb != NULL)
    {
        drawFrame_cb((uint8_t*) OV7670.buffer_addr, OV7670_FRAME_SIZE_BYTES);
    }

    /* Reset line counter */
    OV7670.lineCnt = 0U;
    //TODO: check for full-size QVGA buffer mode
    HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_CONTINUOUS, OV7670.buffer_addr,
            OV7670_FRAME_SIZE_WORDS);
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
}

#else

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    /* If this line is the last line of the frame */
    if (OV7670.lineCnt == OV7670_QVGA_HEIGHT)
    {
        /* Disable DCMI Camera interface */
        HAL_DCMI_Stop(OV7670.hdcmi);

        /* Stop camera XLK signal until captured image data is drawn */
        HAL_TIM_OC_Stop(OV7670.htim, OV7670.tim_ch);

        /* Reset line counter */
        OV7670.lineCnt = 0U;
    }

    /* Call Display flush function */
    if (drawLine_cb != NULL)
    {
        drawLine_cb((uint8_t*) OV7670.buffer_addr, OV7670_WIDTH_SIZE_BYTES, 0U,
                (OV7670_QVGA_WIDTH - 1U), OV7670.lineCnt);
    }

    /* Increment line counter */
    OV7670.lineCnt++;

    /* If snapshot is not fully captured (DCMI is not disabled above) or
     * we're in Continuous mode */
    if (READ_BIT(hdcmi->Instance->CR, DCMI_CR_CAPTURE) ||
    OV7670.mode == DCMI_MODE_CONTINUOUS)
    {
        /* Update buffer address with the next half-part */
        OV7670.buffer_addr = OV7670_SWITCH_BUFFER();

        /* Capture next line from the snapshot/stream */
        HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_CONTINUOUS,
                OV7670.buffer_addr, OV7670_WIDTH_SIZE_WORDS);
    }
}

#endif /* (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE) */


/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/

static HAL_StatusTypeDef SCCB_Write(uint8_t regAddr, uint8_t data)
{
    HAL_StatusTypeDef ret;
    do
    {
        ret = HAL_I2C_Mem_Write(OV7670.hi2c, OV7670_SCCB_ADDR, regAddr,
                I2C_MEMADD_SIZE_8BIT, &data, 1U, 100U);
    }
    while (ret != HAL_OK && 0);

    return ret;
}

static HAL_StatusTypeDef SCCB_Read(uint8_t regAddr, uint8_t *data)
{
    HAL_StatusTypeDef ret;

    do
    {
        /* HAL_I2C_Mem_Read doesn't work because of SCCB protocol(doesn't have ACK) */
        ret = HAL_I2C_Master_Transmit(OV7670.hi2c, OV7670_SCCB_ADDR, &regAddr, 1U, 100U);
        ret |= HAL_I2C_Master_Receive(OV7670.hi2c, OV7670_SCCB_ADDR, data, 1U, 100U);
    }
    while (ret != HAL_OK && 0);

    return ret;
}
