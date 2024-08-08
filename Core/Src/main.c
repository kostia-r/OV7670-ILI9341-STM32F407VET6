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
#include "ILI9341.h"
#include "OV7670.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define PWM_SET_DUTY_CUCLE(htim, tim_ch, duty_cycle)\
    __HAL_TIM_SET_COMPARE(htim, tim_ch, ((__HAL_TIM_GET_AUTORELOAD(htim) * duty_cycle) / 100UL));

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim5;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim14;

/* LOGO BMP */
// Online converter image-to-C-array
// https://lvgl.io/tools/imageconverter
extern const uint8_t LOGO[];
extern const uint32_t LOGO_size;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/******************** APPLICATION FUNCTIONS PROTOTYPES ************************/

void APP_MakeSnaphot(void);
void APP_StartStream(void);
void APP_StopStream(void);
void APP_LedBrightness_Test(void);

/************************** CALLBACKS PROTOTYPES ******************************/

static void APP_SPI_TC_Callback(void);
static void APP_DCMI_DrawLine_Callback(const uint8_t *buffer, uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y);
static void APP_DCMI_DrawFrame_Callback(const uint8_t *buffer, uint32_t nbytes);

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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  DEBUG_LOG(" Hello From STM32F407VET6");

  /* Initialize ILI9341 SPI Display */
  ILI9341_Init(&hspi2, ILI9341_PIXEL_FMT_RGB565);
  ILI9341_RegisterCallback(ILI9341_TC_CALLBACK, APP_SPI_TC_Callback);
  ILI9341_RegisterCallback(ILI9341_ERR_CALLBACK, Error_Handler);
  ILI9341_SetBackgroundColor(BLACK);

  /* Initialize OV7670 DCMI Camera */
  OV7670_Init(&hdcmi, &hi2c2, &htim5, TIM_CHANNEL_3);
  OV7670_RegisterCallback(OV7670_DRAWLINE_CALLBACK, (OV7670_FncPtr_t) APP_DCMI_DrawLine_Callback);
  OV7670_RegisterCallback(OV7670_DRAWFRAME_CALLBACK, (OV7670_FncPtr_t) APP_DCMI_DrawFrame_Callback);

  /* Initialize backlight brightness PWM (PA7) */
  HAL_TIM_OC_Start(&htim14, TIM_CHANNEL_1);

  /* Draw LOGO test slide */
  ILI9341_DrawFrame(LOGO, LOGO_size);
  HAL_Delay(5000);

  /* Start video stream */
  APP_StartStream();
  HAL_Delay(10000);
  APP_StopStream();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(500);
    APP_MakeSnaphot();

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

/***************************** APPLICATION LOGIC ******************************/
void APP_MakeSnaphot(void)
{
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    HAL_Delay(50);
    OV7670_Start(DCMI_MODE_SNAPSHOT);
}

void APP_StartStream(void)
{
    OV7670_Start(DCMI_MODE_CONTINUOUS);
}

void APP_StopStream(void)
{
    OV7670_Stop();
    HAL_Delay(50);
    ILI9341_FillRect(BLACK, 0, 320, 0, 240);
    HAL_Delay(50);
}

/* Is not supported by the HW of the current ILI9341 */
/* Smooth blinking of the LED PA7 */
void APP_LedBrightness_Test(void)
{
    static uint8_t brightness = 0U;
    static uint8_t direction = 0U;

    if (!direction)
    {
        brightness++;
        direction = (brightness == 100U) ? 1U : direction;
    }

    if (direction)
    {
        brightness--;
        direction = (brightness == 0U) ? 0U : direction;
    }

    PWM_SET_DUTY_CUCLE(&htim14, TIM_CHANNEL_1, brightness);
    //HAL_Delay(10);
}

/************************ ILI9341 <-> OV7670 callbacks ************************/

/* This callback is invoked at the end of each SPI transaction */
static void APP_SPI_TC_Callback(void)
{
    /* Resume Camera XLK signal once captured image data is drawn */
    HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_3);
}


/* This callback is invoked at the end of each DCMI snapshot line reading */
static void APP_DCMI_DrawLine_Callback(const uint8_t *buffer,
        uint32_t nbytes, uint16_t x1, uint16_t x2, uint16_t y)
{
    ILI9341_DrawCrop(buffer, nbytes, x1, x2, y, y);
}


/* This callback is invoked at the end of each DCMI whole snapshot reading */
static void APP_DCMI_DrawFrame_Callback(const uint8_t *buffer, uint32_t nbytes)
{
    ILI9341_DrawFrame(buffer, nbytes);
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
