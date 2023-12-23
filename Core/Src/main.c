/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "./App/fatfs.h"
#include "rng.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "touch.h"
#include "delay.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_demo_music.h"
#include "gui_guider.h"
#include "events_init.h"
#include "src/lv_100ask_pinyin_ime/lv_100ask_pinyin_ime.h"
#include "widgets_init.h"
#include "Third_Party/FatFs/src/ff.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
lv_obj_t *scr;
lv_ui guider_ui;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float ADC1_NUM[1024];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FATFS fs;                       /* FatFs 文件系统对象 */
FIL file;                       /* 文件对象 */
FRESULT f_res;                  /* 文件操作结果 */
uint16_t fnum;                      /* 文件成功读写数量 */
BYTE ReadBuffer[1024] = {0};    /* 读缓冲区 */
BYTE WriteBuffer[] =            /* 写缓冲区 */
        "hello word\r\n";
uint16_t file_memory=0;
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
  MX_TIM3_Init();
  MX_FSMC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_RNG_Init();
  MX_USART1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_ADC_Start_IT(&hadc1);//初始化ADC采集
    delay_init(168);//初始化正电原子的延迟函数（参数是根据芯片的晶振）
    lcd_init();//屏幕初始化
    lcd_display_dir(1);//将屏幕设置为横屏
    tp_dev.init();//触屏初始化

/*    lv_init();//lvgl初始化
    lv_port_disp_init();//lvgl接口初始化
    lv_port_indev_init();//lvgl触摸初始化*/

    HAL_SD_CardInfoTypeDef sd_info;
    HAL_StatusTypeDef sd_state= HAL_SD_GetCardInfo(&hsd,&sd_info);
    HAL_SD_CardCIDTypeDef SD_CID;
    HAL_SD_GetCardCID(&hsd,&SD_CID);

    while(sd_state!=HAL_OK){
        lcd_show_string(10,10,200,40,32,"SD Get info Failed",RED);
        sd_state= HAL_SD_GetCardInfo(&hsd,&sd_info);
    }
    uint64_t cardcap=(uint64_t)(sd_info.LogBlockNbr)*(uint64_t)(sd_info.LogBlockSize)/1024/1024;

    int y=10;
    uint32_t m=10;
    lcd_show_num(200,y,sd_info.Class,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.Class:",RED);
    lcd_show_num(200,y+=30,sd_info.BlockNbr,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.BlockNbr:",RED);
    lcd_show_num(200,y+=30,sd_info.BlockSize,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.BlockSize:",RED);
    lcd_show_num(200,y+=30,sd_info.CardType,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.CardType:",RED);
    lcd_show_num(200,y+=30,sd_info.CardVersion,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.CardVersion:",RED);
    lcd_show_num(200,y+=30,sd_info.LogBlockNbr,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.LogBlockNbr:",RED);
    lcd_show_num(200,y+=30,sd_info.LogBlockSize,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.LogBlockSize:",RED);
    lcd_show_num(200,y+=30,sd_info.RelCardAdd,5,16,RED);
    lcd_show_string(10,y,200,40,16,"sd_info.RelCardAdd:",RED);

    lcd_show_num(200,y+=30,cardcap,5,16,RED);
    lcd_show_string(10,y,200,40,16,"cardcap:",RED);
    lcd_show_string(250,y,200,40,16,"MB",RED);

    lcd_show_num(200,y+=30,SD_CID.ManufacturerID,5,16,RED);
    lcd_show_string(10,y,200,40,16,"SD_CID.ManufacturerID", RED);

    lcd_show_num(200,y+=30,SD_CID.CID_CRC,5,16,RED);
    lcd_show_string(10,y,200,40,16,"SD_CID.CRC", RED);

    lcd_show_num(200,y+=30,SD_CID.ManufactDate,5,16,RED);
    lcd_show_string(10,y,200,40,16,"SD_CID.ManufactDate", RED);
    f_res = f_mount(&fs, "0:", 1);
    if(f_res == FR_OK)
    {
        lcd_show_string(500,0,200,40,16,"ok", RED);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
    while (1) {
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
