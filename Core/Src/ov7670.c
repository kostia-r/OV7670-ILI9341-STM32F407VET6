
#include "main.h"
#include "common.h"
#include "OV7670.h"
#include "OV7670_Reg.h"
#include "OV7670_Cfg.h"


#include "ILI9341.h"

/*** Internal Const Values, Macros ***/
#define OV7670_QVGA_WIDTH               (320U)
#define OV7670_QVGA_HEIGHT              (240U)

//#define OV7670_QVGA_WIDTH               (176U)
//#define OV7670_QVGA_HEIGHT              (144U)


#define OV7670_SRTEAM_MODE_BY_FRAME     (0U)
#define OV7670_STREAM_MODE_BY_LINE      (1U)
//TODO
#define OV7670_STREAM_MODE              OV7670_STREAM_MODE_BY_LINE

#define OV7670_WIDTH_SIZE_BYTES         (OV7670_QVGA_WIDTH * 2U)
#define OV7670_WIDTH_SIZE_WORDS         (OV7670_WIDTH_SIZE_BYTES / 4U)
#define OV7670_HEIGHT_SIZE_BYTES        (OV7670_QVGA_HEIGHT * 2U)
#define OV7670_HEIGHT_SIZE_WORDS        (OV7670_HEIGHT_SIZE_BYTES / 4U)
#define OV7670_FRAME_SIZE_BYTES         (OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT * 2U)
#define OV7670_FRAME_SIZE_WORDS         (OV7670_FRAME_SIZE_BYTES / 4U)

//TODO: add all callbacks via config structure
#define OV7670_DRAW_LINE_CALLBACK(buffer, size, x1, x2, y)\
    ILI9341_DrawCrop((buffer), (size), (x1), (x2), (y), (y))

#define OV7670_DRAW_FRAME_CALLBACK(buffer, size)\
    ILI9341_DrawFrame((buffer), (size))

/*** Internal Static Variables ***/
static DCMI_HandleTypeDef* sp_hdcmi;
static DMA_HandleTypeDef* sp_hdma_dcmi;
static I2C_HandleTypeDef* sp_hi2c;
static TIM_HandleTypeDef* htim_ptr;
static uint32_t tim_ocu_channel;

static volatile uint32_t s_currentH;
static volatile uint32_t s_currentV;
static volatile uint32_t mode;

// TODO: make it as 1 double stream-line buffer
#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
static uint8_t buffer1[OV7670_WIDTH_SIZE_BYTES];
static uint8_t buffer2[OV7670_WIDTH_SIZE_BYTES];
static volatile uint8_t* buffersPtr[2] = {(uint8_t*)buffer1, (uint8_t*)buffer2};
static volatile uint8_t cnt;
#else  /* OV7670_SRTEAM_MODE_BY_FRAME */
static uint8_t buffer[OV7670_FRAME_SIZE_BYTES];
#endif /* (OV7670_STREAM_MODE == OV7670_SRTEAM_MODE_BY_FRAME) */

/*** Internal Function Declarations ***/
static RET ov7670_write(uint8_t regAddr, uint8_t data);
static RET ov7670_read(uint8_t regAddr, uint8_t *data);

/*** External Function Defines ***/
void OV7670_Init(DCMI_HandleTypeDef *p_hdcmi, DMA_HandleTypeDef *p_hdma_dcmi,
                 I2C_HandleTypeDef *p_hi2c, TIM_HandleTypeDef* p_htim, uint32_t OCU_Tim_Channel)
{
  sp_hdcmi     = p_hdcmi;
  sp_hdma_dcmi = p_hdma_dcmi;
  sp_hi2c      = p_hi2c;
  htim_ptr     = p_htim;
  tim_ocu_channel = OCU_Tim_Channel;

  // TODO: manual GPIO make as macro
  /* Camera PWDN to GND */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  /* LCD Backlight to 3V3 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  ov7670_write(0x12, 0x80);  // RESET
  HAL_Delay(30);

  uint8_t buffer[4];
  ov7670_read(0x0b, buffer);
  DEBUG_LOG("[OV7670] dev id = %02X", buffer[0]);

  /* TODO: OV7670_MODE_QVGA_RGB565 type is not considering
   * - fix this in the code right below */
  HAL_DCMI_Stop(sp_hdcmi);
  ov7670_write(0x12, 0x80);  // RESET
  HAL_Delay(30);
  for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++)
  {
    ov7670_write(OV7670_reg[i][0], OV7670_reg[i][1]);
    HAL_Delay(1);
  }

  //TODO: start timer only in case of line-by line capturing
  HAL_TIM_OC_Start(htim_ptr, tim_ocu_channel);
}


void OV7670_Start(uint32_t capMode)
{
    HAL_DCMI_Stop(sp_hdcmi);

    mode = capMode;

    if (capMode == OV7670_CAP_CONTINUOUS)
    {
#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
        cnt = 0U;
        HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) buffersPtr[cnt], OV7670_WIDTH_SIZE_WORDS);
#else  /* OV7670_SRTEAM_MODE_BY_FRAME */
        HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) buffer, OV7670_FRAME_SIZE_WORDS);
#endif
    }
    else if (capMode == OV7670_CAP_SINGLE_FRAME)
    {
#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
        // TODO: complete for single frame mode
        cnt = 0U;
        HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) buffersPtr[cnt], OV7670_WIDTH_SIZE_WORDS);
#else  /* OV7670_SRTEAM_MODE_BY_FRAME */
        HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) buffer, OV7670_FRAME_SIZE_WORDS);
#endif
    }
}

void OV7670_Stop(void)
{
  HAL_DCMI_Stop(sp_hdcmi);
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    DEBUG_LOG("VSYNC %ld", HAL_GetTick());
#if (OV7670_STREAM_MODE == OV7670_SRTEAM_MODE_BY_FRAME)
    HAL_DCMI_Stop(sp_hdcmi);
    HAL_TIM_OC_Stop(htim_ptr, tim_ocu_channel);
    //OV7670_DRAW_FRAME_CALLBACK(buffer, OV7670_FRAME_SIZE_BYTES);
    ILI9341_DrawCrop(buffer, OV7670_FRAME_SIZE_BYTES, 0U, (OV7670_QVGA_WIDTH - 1U), 0,(OV7670_QVGA_HEIGHT - 1U));
    s_currentV++;
    s_currentH = 0;
    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) buffer, OV7670_FRAME_SIZE_WORDS);
#endif /* (OV7670_STREAM_MODE == OV7670_SRTEAM_MODE_BY_FRAME) */
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    DEBUG_LOG("FRAME %ld", HAL_GetTick());
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)

	if (s_currentH == OV7670_QVGA_HEIGHT)
	{
		s_currentH = 0;
		HAL_DCMI_Stop(sp_hdcmi);

		if (mode != OV7670_CAP_CONTINUOUS)
		    return;
	}

	OV7670_DRAW_LINE_CALLBACK((uint8_t*)buffersPtr[cnt], OV7670_WIDTH_SIZE_BYTES, 0U, (OV7670_QVGA_WIDTH - 1U), s_currentH);
    HAL_TIM_OC_Stop(htim_ptr, tim_ocu_channel);

	/* start reading new line */
	cnt = (cnt == 0) ? (cnt + 1) : 0;

	HAL_DCMI_Start_DMA(sp_hdcmi, mode, (uint32_t) buffersPtr[cnt], OV7670_WIDTH_SIZE_WORDS);
	s_currentH++;
#endif
}

/*** Internal Function Defines ***/
static RET ov7670_write(uint8_t regAddr, uint8_t data)
{
  HAL_StatusTypeDef ret;
  do {
    ret = HAL_I2C_Mem_Write(sp_hi2c, SLAVE_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  } while (ret != HAL_OK && 0);
  return ret;
}

static RET ov7670_read(uint8_t regAddr, uint8_t *data)
{
    HAL_StatusTypeDef ret;

    do
    {
        /* HAL_I2C_Mem_Read doesn't work (because of SCCB protocol(doesn't have ack))? */
//      ret = HAL_I2C_Mem_Read(sp_hi2c, SLAVE_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
        ret = HAL_I2C_Master_Transmit(sp_hi2c, SLAVE_ADDR, &regAddr, 1, 100);
        ret |= HAL_I2C_Master_Receive(sp_hi2c, SLAVE_ADDR, data, 1, 100);
    }
    while (ret != HAL_OK && 0);
    return ret;
}


