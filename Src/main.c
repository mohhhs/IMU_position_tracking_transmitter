/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Set up for ARM Cortex M7 to communicate with BMI 160 
  *                   Data sent over UART
  *                   Data is avaraged by the predifined macro 'DATAAVG'
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include "string.h"
#include "bmi160.h"
#include "dwt_stm32_delay.h"
#define DEBUG_PRINT

/* Private typedef -----------------------------------------------------------*/

#ifdef DEBUG_PRINT

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#endif

/* Private define/macro ------------------------------------------------------*/

#define DEVADR UINT8_C(0x68 << 1)
#define DATALEN 7 
#define DATAAVG 5 

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

struct bmi160_dev sensor;

struct tm *timeinfo;

/* BEGIN Private Funcatoins */

// BMI160 write function
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t rslt = 5; /* Return 0 for Success, non-zero for failure */

  uint8_t data_size = sizeof(reg_data);
  uint8_t data[data_size]; // first byte is register address
  data[0] = reg_addr;
  for (int a = 1; a < data_size; a++) //filling the array
  {
    data[a] = reg_data[a - 1];
  }
  rslt = HAL_I2C_Master_Transmit(&hi2c1, dev_id, data, len + 1, 50);

  return rslt; // rslt is HAL_statustypedef
}

// BMI160 Read Function
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{

  int8_t rslt = 5;                                                   /* Return 0 for Success, non-zero for failure */
  HAL_I2C_Master_Transmit(&hi2c1, dev_id, &reg_addr, 1, 20);         //go to register adress
  rslt = HAL_I2C_Master_Receive(&hi2c1, dev_id, reg_data, len, 100); //check 3 para 3

  return rslt;
}

int8_t sensorsInit(void)
{

  int8_t rslt;

  /* BMI160 */
  sensor.id = DEVADR; //BMI160_I2C_ADDR;
  sensor.interface = BMI160_I2C_INTF;
  sensor.read = (bmi160_com_fptr_t)user_i2c_read; // I just put type data from an example?
  sensor.write = (bmi160_com_fptr_t)user_i2c_write;
  sensor.delay_ms = (bmi160_delay_fptr_t)DWT_Delay_ms;

  rslt = bmi160_init(&sensor); // initialize the device
  if (rslt == 0)
  {
    printf("BMI160 I2C connection [OK].\r\n");
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

    /* Select the power mode of Gyroscope sensor */
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&sensor);
    sensor.delay_ms(50);
  }
  else
  {
    printf("BMI160 I2C connection [FAIL].\r\n");
  }
  return rslt;
}

/* END Private Funcatoins */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
uint16_t count;
int16_t imu_data[DATALEN];
int16_t imu_debug[6]; // Store live variable

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

// configure FOC for accel
int8_t start_foc(struct bmi160_dev *dev)
{
  int8_t rslt = 0;
  /* FOC configuration structure */
  struct bmi160_foc_conf foc_conf;
  /* Structure to store the offsets */
  struct bmi160_offsets offsets;

  /* Enable FOC for accel with target values of z = 1g ; x,y as 0g */
  foc_conf.acc_off_en = BMI160_ENABLE;
  foc_conf.foc_acc_x = BMI160_FOC_ACCEL_0G;
  foc_conf.foc_acc_y = BMI160_FOC_ACCEL_0G;
  foc_conf.foc_acc_z = BMI160_FOC_ACCEL_POSITIVE_G;

  /* Enable FOC for gyro */
  //	foc_conf.foc_gyr_en = BMI160_ENABLE;
  //	foc_conf.gyro_off_en = BMI160_ENABLE;

  rslt = bmi160_start_foc(&foc_conf, &offsets, dev);

  if (rslt == BMI160_OK)
  {
    printf("\n FOC DONE SUCCESSFULLY ");
    printf("\n OFFSET VALUES AFTER FOC : ");
    printf("\n OFFSET VALUES ACCEL X : %d ", offsets.off_acc_x);
    printf("\n OFFSET VALUES ACCEL Y : %d ", offsets.off_acc_y);
    printf("\n OFFSET VALUES ACCEL Z : %d ", offsets.off_acc_z);
    //		printf("\n OFFSET VALUES GYRO  X : %d ",offsets.off_gyro_x);
    //		printf("\n OFFSET VALUES GYRO  Y : %d ",offsets.off_gyro_y);
    //		printf("\n OFFSET VALUES GYRO  Z : %d ",offsets.off_gyro_z);
  }

  /* After start of FOC offsets will be updated automatically and
	 * Data will be very similar to target measurment values*/

  return rslt;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  DWT_Delay_Init();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();

  /* Data system setup */
  int isd;

  uint8_t reg_addr = BMI160_CHIP_ID_ADDR; 
  uint8_t data;
  uint16_t len = 1;

  // divided 7 reading array (16 bit) into 14 element array (8 bit) array for UART transimition
  uint8_t imu_data8[DATALEN*2]; 
  uint8_t led_count = 0;

  int8_t rslt = BMI160_OK;
  rslt = sensorsInit();
  struct bmi160_sensor_data accel;
  struct bmi160_sensor_data gyro;

  start_foc(&sensor);

  //  clock_t t;

  /* Infinite loop */
  for (;;)
  {
    printf("---------------------------------------\r\n");
    //	  HAL_GPIO_TogglePin(GPIOB , LD1_Pin);

    isd = HAL_I2C_IsDeviceReady(&hi2c1, DEVADR, 2, 1000); /*- zero means sucess -*/
    printf("output of isd is  %d \r\n", isd);

    //	  DWT_Delay_ms (5); //with the two delays I have, reading 4 times a second

    count++;
    led_count++;

    //	  bmi160_get_regs(reg_addr, &data, len, &sensor);
    printf("BMI160 init result is  %d,  Chip-id =  %d \r\n", rslt, data);
    //	  t = clock(); // start measuring time taken

    // to flash led every 10 readings
    if (led_count == 10)
    {
      HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
      led_count = 0;
    }
    for (int i = 1; i < DATALEN; i++)
      imu_data[i] = 0; // to intialize all 6 axies

    // to avrage 5 readings
    for (int j = 0; j < DATAAVG; j++)
    { 
      bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor);
      imu_data[1] += accel.x / DATAAVG;
      imu_data[2] += accel.y / DATAAVG;
      imu_data[3] += accel.z / DATAAVG;
      imu_data[4] += gyro.x / DATAAVG;
      imu_data[5] += gyro.y / DATAAVG;
      imu_data[6] += gyro.z / DATAAVG;
    }
    //assign values to full array
    imu_data[0] = count;
    imu_debug[0] = imu_data[1];
    imu_debug[1] = imu_data[2];
    imu_debug[2] = imu_data[3];
    imu_debug[3] = imu_data[4];
    imu_debug[4] = imu_data[5];
    imu_debug[5] = imu_data[6];

    //tansfer 16bit data array into two 8bit, send msb first then lsb
    for (int i = 0; i < DATALEN; i++)
    {
      //lsb, and the low 8 by 1s give same, msb will be zero
      imu_data8[i * 2 + 1] = imu_data[i] & 0xff;
      //msb, shift high 8 by 8
      imu_data8[i * 2] = (imu_data[i] >> 8);
    }
    //		  HAL_UART_Transmit(&huart3, Test, sizeof(Test), 100); // this one works ...
    //  using python I am reading hex :) - sends ascii unless you specifie data format
    // reached 382 reading a second, at baud = 256000, transit timeout = 1 and no delay
    // saved 100,000 reading file size = 3.4M in 4Min 20sec

    HAL_UART_Transmit(&huart3, imu_data8, sizeof(imu_data8), 10);
    DWT_Delay_ms(12);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /*I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00808CD2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  // Check Pg1030 Fig(333)
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
#ifdef DEBUG_PRINT

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
