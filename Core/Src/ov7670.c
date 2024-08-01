/*
 * ov7670.c
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "common.h"
#include "ov7670.h"
#include "ov7670Config.h"
#include "ov7670Reg.h"

#include "bsp_lcd.h"

/*** Internal Const Values, Macros ***/
#define OV7670_QVGA_WIDTH  320
#define OV7670_QVGA_HEIGHT 240

//#define OV7670_QVGA_WIDTH  100
//#define OV7670_QVGA_HEIGHT 100

extern TIM_HandleTypeDef htim5;

extern volatile uint8_t* buffersPtr[2];
extern volatile uint8_t cnt;

/*** Internal Static Variables ***/
static DCMI_HandleTypeDef *sp_hdcmi;
static DMA_HandleTypeDef  *sp_hdma_dcmi;
static I2C_HandleTypeDef  *sp_hi2c;
static uint32_t    s_destAddressForContiuousMode;
static void (* s_cbHsync)(uint32_t h);
static void (* s_cbVsync)(uint32_t v);
static volatile uint32_t s_currentH;
static volatile uint32_t s_currentV;

/*** Internal Function Declarations ***/
static RET ov7670_write(uint8_t regAddr, uint8_t data);
static RET ov7670_read(uint8_t regAddr, uint8_t *data);

/*** External Function Defines ***/
RET ov7670_init(DCMI_HandleTypeDef *p_hdcmi, DMA_HandleTypeDef *p_hdma_dcmi, I2C_HandleTypeDef *p_hi2c)
{
  sp_hdcmi     = p_hdcmi;
  sp_hdma_dcmi = p_hdma_dcmi;
  sp_hi2c      = p_hi2c;
  s_destAddressForContiuousMode = 0;

  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  ov7670_write(0x12, 0x80);  // RESET
  HAL_Delay(30);

  uint8_t buffer[4];
  ov7670_read(0x0b, buffer);
  printf("[OV7670] dev id = %02X\n", buffer[0]);


  return RET_OK;
}

RET ov7670_config(uint32_t mode)
{
  ov7670_stopCap();
  ov7670_write(0x12, 0x80);  // RESET
  HAL_Delay(30);
  for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
    ov7670_write(OV7670_reg[i][0], OV7670_reg[i][1]);
    HAL_Delay(1);
  }

  return RET_OK;
}

RET ov7670_startCap(uint32_t capMode)
{
  ov7670_stopCap();
  if (capMode == OV7670_CAP_CONTINUOUS) {
    /* note: continuous mode automatically invokes DCMI, but DMA needs to be invoked manually */
    s_destAddressForContiuousMode = 0;//destAddress;
    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)buffersPtr[cnt], OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
  } else if (capMode == OV7670_CAP_SINGLE_FRAME) {
    s_destAddressForContiuousMode = 0;
    //HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
    //__HAL_DCMI_CLEAR_FLAG(sp_hdcmi, DCMI_FLAG_OVRRI);

    cnt = 0;
    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)buffersPtr[cnt], OV7670_QVGA_WIDTH /2);
    //
  }

  return RET_OK;
}

RET ov7670_stopCap()
{
  HAL_DCMI_Stop(sp_hdcmi);
//  HAL_Delay(30);
  return RET_OK;
}

void ov7670_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v))
{
  s_cbHsync = cbHsync;
  s_cbVsync = cbVsync;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
//  printf("FRAME %d\n", HAL_GetTick());
  if(s_cbVsync)s_cbVsync(s_currentV);
  if(s_destAddressForContiuousMode != 0) {
    HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, s_destAddressForContiuousMode, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
  }
  s_currentV++;
  //s_currentH = 0;
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
//  printf("VSYNC %d\n", HAL_GetTick());
//  HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, s_destAddressForContiuousMode, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
}



void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	if (s_currentH == OV7670_QVGA_HEIGHT)
	{
		s_currentH = 0;
		ov7670_stopCap();
	}

	bsp_lcd_set_display_area(0, OV7670_QVGA_WIDTH - 1, s_currentH, s_currentH);

	s_currentH++;

	HAL_TIM_OC_Stop(&htim5, TIM_CHANNEL_3);
	bsp_lcd_send_cmd_mem_write();
	//bsp_lcd_write((uint8_t*)buffersPtr[cnt], OV7670_QVGA_WIDTH * 2);
	//HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_3);
	bsp_lcd_write_dma((uint32_t) buffersPtr[cnt], OV7670_QVGA_WIDTH * 2);
	// start reading new line
	cnt = (cnt == 0) ? (cnt + 1) : 0;
	HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) buffersPtr[cnt], OV7670_QVGA_WIDTH / 2);
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
  do {
    // HAL_I2C_Mem_Read doesn't work (because of SCCB protocol(doesn't have ack))? */
//    ret = HAL_I2C_Mem_Read(sp_hi2c, SLAVE_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
    ret = HAL_I2C_Master_Transmit(sp_hi2c, SLAVE_ADDR, &regAddr, 1, 100);
    ret |= HAL_I2C_Master_Receive(sp_hi2c, SLAVE_ADDR, data, 1, 100);
  } while (ret != HAL_OK && 0);
  return ret;
}


