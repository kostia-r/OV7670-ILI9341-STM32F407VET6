
#ifndef OV7670_H_
#define OV7670_H_

#include "main.h"

#define OV7670_MODE_QVGA_RGB565 0
#define OV7670_MODE_QVGA_YUV    1

#define OV7670_CAP_CONTINUOUS   0
#define OV7670_CAP_SINGLE_FRAME 1

void OV7670_Init(DCMI_HandleTypeDef *p_hdcmi, DMA_HandleTypeDef *p_hdma_dcmi,
                 I2C_HandleTypeDef *p_hi2c, TIM_HandleTypeDef* p_htim,
                 uint32_t OCU_Tim_Channel);
void OV7670_Start(uint32_t capMode);
void OV7670_Stop(void);
void ov7670_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v));

#endif /* OV7670_H_ */
