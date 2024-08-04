/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_lcd.h"
#include "ov7670.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Online converter image-to-C-array
// https://lvgl.io/tools/imageconverter
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;
extern TIM_HandleTypeDef htim5;

#if (PRINT_PICS == 1)
extern const uint8_t dasha[];
extern const uint32_t dashaSize;
extern const uint8_t sonia[];
extern const uint32_t soniaSize;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_DCMI_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  DEBUG_LOG(" Hello From STM32F407VET6");
  ILI9341_Init();
#if 0
  bsp_lcd_set_background_color(YELLOW);
  bsp_lcd_set_display_area(60, 259, 100,139);
  bsp_lcd_send_cmd_mem_write();
  uint16_t data[200UL * 40UL];
  for(uint32_t i = 0 ; i < (200UL * 40UL) ; i++)
  {
      data[i] = bsp_lcd_convert_rgb888_to_rgb565(RED);
  }
  bsp_lcd_write((uint8_t*)data, (200UL * 40UL * 2UL));
#endif
  uint32_t x_start, x_width, y_start, y_height;
  bsp_lcd_set_background_color(RED);
  x_start = 0;
  y_start = 0;
#if(BSP_LCD_ORIENTATION == LANDSCAPE)
  x_width = 320;
  y_height = 34;
#elif(BSP_LCD_ORIENTATION == PORTRAIT)
  x_width = 240;
  y_height = 45;
#endif
  lcd_set_orientation(PORTRAIT);
  bsp_lcd_fill_rect(VIOLET, x_start, x_width, y_height * 0, y_height);
  bsp_lcd_fill_rect(INDIGO, x_start, x_width, y_height * 1, y_height);
  bsp_lcd_fill_rect(BLUE, x_start, x_width, y_height * 2, y_height);
  bsp_lcd_fill_rect(GREEN, x_start, x_width, y_height * 3, y_height);
  bsp_lcd_fill_rect(YELLOW, x_start, x_width, y_height * 4, y_height);
  bsp_lcd_fill_rect(ORANGE, x_start, x_width, y_height * 5, y_height);
  bsp_lcd_fill_rect(RED, x_start, x_width, y_height * 6, y_height);
  HAL_Delay(5000);
#if (PRINT_PICS == 1)
  ILI9341_DrawFrame(sonia, soniaSize);
  HAL_Delay(5000);
  ILI9341_DrawFrame(dasha, dashaSize);
  HAL_Delay(5000);
#endif
  lcd_set_orientation(LANDSCAPE);
  ov7670_Init(&hdcmi, &hdma_dcmi, &hi2c2, &htim5, TIM_CHANNEL_3);
  ov7670_startCap(OV7670_CAP_CONTINUOUS);
  HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_Delay(2000);
	  //HAL_DCMI_DeInit(&hdcmi);
	  //HAL_DMA_DeInit(&hdcmi.DMA_Handle);
	  //MX_DCMI_Init();
	  ov7670_startCap(OV7670_CAP_SINGLE_FRAME);
	  HAL_Delay(1500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* This callback is invoked at the end of each SPI transaction */
void APP_SPI_TC_Callback(void)
{

}

/* This callback is invoked at the end after each drawn screen */
void APP_SPI_ScreenDrawComplete(void)
{

}

void APP_SPI_Error_Callback(void)
{
    while(1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
