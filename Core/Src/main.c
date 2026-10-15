/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Blue Pill + 6-Pin BME280 Mini Weather Station
  *                   Generated via Standalone STM32CubeMX -> STM32CubeIDE 2.x
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- LIVE WEATHER VARIABLES (Viewable in CubeIDE "Live Expressions") ---
volatile float Temperature_C = 0.0f;
volatile float Humidity_RH   = 0.0f;
volatile float Pressure_hPa  = 0.0f;
volatile uint8_t Sensor_OK   = 0;

// BME280 I2C Address (0x76 when SDO is connected to GND)
uint8_t bme280_addr = (0x76 << 1);

// Factory Calibration Registers inside the BME280
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t  dig_H1, dig_H3;
int16_t  dig_H2, dig_H4, dig_H5;
int8_t   dig_H6;
int32_t  t_fine;

char uart_buf[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t BME280_Init(void);
void BME280_Read_Calibration(void);
void BME280_Read_Weather(void);
void UART_Print(char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Print(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

void BME280_Read_Calibration(void) {
    uint8_t calib1[26] = {0};
    uint8_t calib2[7]  = {0};

    // Read factory trimming parameters from BME280 ROM
    HAL_I2C_Mem_Read(&hi2c1, bme280_addr, 0x88, 1, calib1, 26, 1000);
    HAL_I2C_Mem_Read(&hi2c1, bme280_addr, 0xE1, 1, calib2, 7, 1000);

    dig_T1 = (calib1[1] << 8) | calib1[0];
    dig_T2 = (calib1[3] << 8) | calib1[2];
    dig_T3 = (calib1[5] << 8) | calib1[4];

    dig_P1 = (calib1[7] << 8) | calib1[6];
    dig_P2 = (calib1[9] << 8) | calib1[8];
    dig_P3 = (calib1[11] << 8) | calib1[10];
    dig_P4 = (calib1[13] << 8) | calib1[12];
    dig_P5 = (calib1[15] << 8) | calib1[14];
    dig_P6 = (calib1[17] << 8) | calib1[16];
    dig_P7 = (calib1[19] << 8) | calib1[18];
    dig_P8 = (calib1[21] << 8) | calib1[20];
    dig_P9 = (calib1[23] << 8) | calib1[22];

    dig_H1 = calib1[25];
    dig_H2 = (calib2[1] << 8) | calib2[0];
    dig_H3 = calib2[2];
    dig_H4 = (calib2[3] << 4) | (calib2[4] & 0x0F);
    dig_H5 = (calib2[5] << 4) | (calib2[4] >> 4);
    dig_H6 = (int8_t)calib2[6];
}

uint8_t BME280_Init(void) {
    uint8_t chip_id = 0;

    // Auto-check 0x76 (SDO->GND) and 0x77 (SDO->3.3V)
    if (HAL_I2C_IsDeviceReady(&hi2c1, (0x76 << 1), 3, 100) == HAL_OK) {
        bme280_addr = (0x76 << 1);
    } else if (HAL_I2C_IsDeviceReady(&hi2c1, (0x77 << 1), 3, 100) == HAL_OK) {
        bme280_addr = (0x77 << 1);
    } else {
        return 0; // Sensor not responding on B6/B7
    }

    // Verify Chip ID register (0xD0): 0x60 = BME280, 0x58 = BMP280
    HAL_I2C_Mem_Read(&hi2c1, bme280_addr, 0xD0, 1, &chip_id, 1, 1000);
    if (chip_id != 0x60 && chip_id != 0x58) {
        return 0;
    }

    // Soft reset the sensor
    uint8_t reset_cmd = 0xB6;
    HAL_I2C_Mem_Write(&hi2c1, bme280_addr, 0xE0, 1, &reset_cmd, 1, 1000);
    HAL_Delay(100);

    // Load calibration constants
    BME280_Read_Calibration();

    // 1. Humidity oversampling x1 (Register 0xF2)
    uint8_t ctrl_hum = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, bme280_addr, 0xF2, 1, &ctrl_hum, 1, 1000);

    // 2. Standby time 1000ms, filter off (Register 0xF5)
    uint8_t config = (0x05 << 5);
    HAL_I2C_Mem_Write(&hi2c1, bme280_addr, 0xF5, 1, &config, 1, 1000);

    // 3. Temp & Pressure oversampling x1, Normal Mode (Register 0xF4)
    uint8_t ctrl_meas = (0x01 << 5) | (0x01 << 2) | 0x03;
    HAL_I2C_Mem_Write(&hi2c1, bme280_addr, 0xF4, 1, &ctrl_meas, 1, 1000);

    return 1;
}

void BME280_Read_Weather(void) {
    uint8_t raw[8];
    // Burst read 8 bytes starting from register 0xF7
    HAL_I2C_Mem_Read(&hi2c1, bme280_addr, 0xF7, 1, raw, 8, 1000);

    int32_t adc_P = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = (raw[6] << 8)  |  raw[7];

    // --- Temperature Compensation ---
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    Temperature_C = (float)T / 100.0f;

    // --- Pressure Compensation ---
    int64_t p_var1, p_var2, p;
    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int32_t)dig_P1) >> 33;
    if (p_var1 != 0) {
        p = 1048576 - adc_P;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
        Pressure_hPa = (float)p / 25600.0f;
    }

    // --- Humidity Compensation ---
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    Humidity_RH = (float)(v_x1_u32r >> 12) / 1024.0f;
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
  HAL_Delay(500);
  UART_Print("\r\n============================================\r\n");
  UART_Print("   BLUE PILL MINI WEATHER STATION (BME280)  \r\n");
  UART_Print("============================================\r\n");

  Sensor_OK = BME280_Init();
  if (Sensor_OK == 0) {
      UART_Print("[ERROR] BME280 not detected! Check B6, B7, CSB & SDO.\r\n");
      while (1) {
          // Rapid blink on PC13 LED indicates wiring error
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          HAL_Delay(150);
      }
  }
  UART_Print("[OK] BME280 Initialized at 0x76!\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Toggle Blue Pill onboard LED (PC13) on each reading
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // Read and compensate Temperature, Humidity, and Pressure
    BME280_Read_Weather();

    // Format and send over USART1 (PA9/PA10)
    snprintf(uart_buf, sizeof(uart_buf),
             "--------------------------------------------\r\n"
             " Temperature : %.2f C\r\n"
             " Humidity    : %.2f %%RH\r\n"
             " Pressure    : %.2f hPa\r\n"
             "--------------------------------------------\r\n",
             Temperature_C, Humidity_RH, Pressure_hPa);

    UART_Print(uart_buf);

    HAL_Delay(2000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
