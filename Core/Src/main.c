/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR   (0x68 << 1)   /* HAL wants the 8-bit address */
#define ACC_SENS   8192.0f       /* LSB/g at +-4 g              */
#define GYR_SENS   65.5f         /* LSB/(deg/s) at +-500 dps    */
#define TS_MS      10U           /* 100 Hz sampling             */
#define WIN        100           /* 1 s analysis window         */
#define A_HP       0.841f        /* 3 Hz high-pass @ 100 Hz     */
#define A_LP       0.430f        /* 12 Hz low-pass @ 100 Hz     */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Globals for CubeIDE Live Expressions (watch these while debugging) */
volatile float g_rmsA = 0, g_freq = 0;
volatile char g_level[12] = "NONE";

typedef struct {
    float xh[2], yh[2], yl[2];
} Bandpass;

static Bandpass bpA[3], bpG[3];

static float bp_process(Bandpass *b, float x)
{
    float v = x;
    for (int i = 0; i < 2; i++) {            /* 2 cascaded high-pass */
        float y = A_HP * (b->yh[i] + v - b->xh[i]);
        b->xh[i] = v;
        b->yh[i] = y;
        v = y;
    }
    for (int i = 0; i < 2; i++) {            /* 2 cascaded low-pass */
        b->yl[i] += A_LP * (v - b->yl[i]);
        v = b->yl[i];
    }
    return v;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef mpu_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef mpu_read_all(int16_t *d)  /* d[7]: ax ay az T gx gy gz */
{
    uint8_t buf[14];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B,
                             I2C_MEMADD_SIZE_8BIT, buf, 14, 100);
    if (st != HAL_OK) return st;
    for (int i = 0; i < 7; i++)
        d[i] = (int16_t)((buf[2 * i] << 8) | buf[2 * i + 1]);
    return HAL_OK;
}

/* Classifies tremor severity from filtered RMS amplitude (g) and the
   dominant frequency (Hz) of the filtered signal.
   - Low energy              -> NONE
   - Energy present, but the dominant frequency is outside the 3-12 Hz
     tremor band -> likely voluntary/normal movement, not tremor -> MOTION
   - Energy present and frequency is within the tremor band -> graded
     by amplitude into MILD / MODERATE / SEVERE
   PLACEHOLDER thresholds - calibrate with your own test data! */
static const char *classify(float rmsA, float freqHz)
{
    if (rmsA < 0.02f) return "NONE";
    if (freqHz < 3.0f || freqHz > 12.0f) return "MOTION";
    if (rmsA < 0.05f) return "MILD";
    if (rmsA < 0.15f) return "MODERATE";
    return "SEVERE";
}

static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 100);
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  /* LED off (active low) */

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDR, 3, 100) != HAL_OK) {
      uart_print("MPU6050 NOT FOUND - check wiring\r\n");
      while (1) {                                           /* fast blink = error */
          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
          HAL_Delay(150);
      }
  }
  mpu_write(0x6B, 0x00);   /* wake up          */
  mpu_write(0x1A, 0x03);   /* DLPF ~44 Hz      */
  mpu_write(0x1B, 0x08);   /* gyro  +-500 dps  */
  mpu_write(0x1C, 0x08);   /* accel +-4 g      */
  uart_print("rms_acc_mg,rms_gyro_dps_x10,freq_hz_x10,level\r\n");

  SSD1306_Init(&hi2c1);
  SSD1306_DrawString(0, 0, "TREMOR MONITOR");
  SSD1306_UpdateScreen();
  HAL_Delay(1000);

  uint32_t next = HAL_GetTick() + TS_MS;
  int   n = 0;
  float sumA = 0, sumG = 0, axisE[3] = {0, 0, 0};
  int   cross[3] = {0, 0, 0};
  uint8_t pos[3] = {0, 0, 0};
  char  line[80];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((int32_t)(HAL_GetTick() - next) >= 0)
    {
        next += TS_MS;

        int16_t d[7];
        if (mpu_read_all(d) == HAL_OK)
        {
            float a[3] = { d[0] / ACC_SENS, d[1] / ACC_SENS, d[2] / ACC_SENS };
            float g[3] = { d[4] / GYR_SENS, d[5] / GYR_SENS, d[6] / GYR_SENS };

            float eA = 0, eG = 0;
            for (int i = 0; i < 3; i++) {
                float fa = bp_process(&bpA[i], a[i]);
                float fg = bp_process(&bpG[i], g[i]);
                eA += fa * fa;
                eG += fg * fg;
                axisE[i] += fa * fa;

                /* zero crossings with hysteresis (0.005 g) -> frequency */
                if (!pos[i] && fa > 0.005f)      { pos[i] = 1; cross[i]++; }
                else if (pos[i] && fa < -0.005f) { pos[i] = 0; }
            }
            sumA += eA;
            sumG += eG;

            if (++n >= WIN)
            {
                float rmsA = sqrtf(sumA / WIN);
                float rmsG = sqrtf(sumG / WIN);

                int dom = 0;                                  /* dominant axis */
                for (int i = 1; i < 3; i++)
                    if (axisE[i] > axisE[dom]) dom = i;
                float freq = cross[dom] * (100.0f / WIN);     /* Hz */

                /* Update globals for CubeIDE Live Expressions */
                g_rmsA = rmsA;
                g_freq = freq;
                strncpy((char*)g_level, classify(rmsA, freq), sizeof(g_level) - 1);

                snprintf(line, sizeof(line), "%d,%d,%d,%s\r\n",
                         (int)(rmsA * 1000.0f),
                         (int)(rmsG * 10.0f),
                         (int)(freq * 10.0f),
                         classify(rmsA, freq));
                uart_print(line);

                /* OLED update */
                char l1[20], l2[20], l3[20];
                snprintf(l1, sizeof(l1), "RMS:%d.%03dG",
                         (int)(rmsA * 1000) / 1000, (int)(rmsA * 1000) % 1000);
                snprintf(l2, sizeof(l2), "FREQ:%d.%dHZ",
                         (int)freq, (int)(freq * 10) % 10);
                snprintf(l3, sizeof(l3), "LEVEL:%s", (char *)g_level);

                SSD1306_Clear();
                SSD1306_DrawString(0, 0, "TREMOR MONITOR");
                SSD1306_DrawString(0, 2, l1);
                SSD1306_DrawString(0, 4, l2);
                SSD1306_DrawString(0, 6, l3);
                SSD1306_UpdateScreen();

                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                                  rmsA > 0.05f ? GPIO_PIN_RESET : GPIO_PIN_SET);

                n = 0; sumA = 0; sumG = 0;
                for (int i = 0; i < 3; i++) { axisE[i] = 0; cross[i] = 0; }
            }
        }
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
