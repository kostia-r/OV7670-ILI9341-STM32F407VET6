
#ifndef OV7670_H_
#define OV7670_H_

#include "main.h"

#define OV7670_MODE_QVGA_RGB565                  (0U)
#define OV7670_MODE_QVGA_YUV                     (1U)

#define OV7670_CAP_CONTINUOUS                    (0U)
#define OV7670_CAP_SINGLE_FRAME                  (1U)

/* TODO: make as enum */
#define OV7670_DRAWLINE_CALLBACK                 (0U)
#define OV7670_DRAWFRAME_CALLBACK                (1U)

typedef void (*OV7670_FncPtr_t)(void);

extern void OV7670_Init(DCMI_HandleTypeDef *p_hdcmi, DMA_HandleTypeDef *p_hdma_dcmi,
                 I2C_HandleTypeDef *p_hi2c, TIM_HandleTypeDef* p_htim,
                 uint32_t OCU_Tim_Channel);
extern void OV7670_RegisterCallback(uint8_t cb_type, OV7670_FncPtr_t fnc_ptr);
extern void OV7670_Start(uint32_t capMode);
extern void OV7670_Stop(void);

#endif /* OV7670_H_ */
