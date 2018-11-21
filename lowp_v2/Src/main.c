/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
//#include "usart.h"
#include "B3M.h"
#include "string.h"
#include "sd_hal_mpu6050.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
I2C_HandleTypeDef hi2c1;
SD_MPU6050 mpu1;

//WWDG_HandleTypeDef hwwdg;

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_WWDG_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);

/* Private function prototypes -----------------------------------------------*/

void setModeToNormal(/*unsigned char ID*/)
{
    uint8_t TxData[8];   // TransmitByteData [1 byte]
    //uint8_t CheckSum = 0; // CheckSum calculation

    TxData[0] = 0x08;   // All Byte
    TxData[1] = 0x04;   // Command
    TxData[2] = 0x00;   // Option/Status
    TxData[3] = 0x00;   // ID
    TxData[4] = 0x00;   // mode
    TxData[5] = 0x28;   // address
    TxData[6] = 0x01; // device number
    TxData[7] = 0x35;   // Sum

    HAL_UART_Transmit(&huart1, (uint8_t*)TxData, 8, HAL_MAX_DELAY);
    HAL_Delay(1);

}

void setAngle(int16_t angle)
{
    uint8_t TxData[9];   // TransmitByteData [9 bits]
    uint8_t CheckSum = 0; // CheckSum calculation
    uint8_t RxData[9];

    if(angle<0)
        angle = 65535+angle+1;

    TxData[0] = 0x09;                 // All Byte
    TxData[1] = 0x04;                       // Command
    TxData[2] = 0x00;                       // Option/Status
    TxData[3] = 0x00;                         // ID
    TxData[4] = (uint8_t)0x00FF & angle;              //  mode
    TxData[5] = (uint8_t)0x00FF & (angle >> 8);       // address
    TxData[6] = 0x2A;                         // device number
    TxData[7] = 0x01;                     // Count

    // CheckSum calculation
    for(unsigned char i=0; i<=7; i++){
        CheckSum = CheckSum + TxData[i];                // XOR from ID to Data
    }
    CheckSum = (uint8_t)0x00FF & CheckSum;

    TxData[8] = CheckSum;

    HAL_UART_Transmit(&huart1, (uint8_t*)TxData, 9, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_UART_Receive(&huart1, (uint8_t*)RxData, 9, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_UART_Transmit(&huart2, (uint8_t*)RxData, 9, HAL_MAX_DELAY);
    HAl_Delay(1);
    /*追加*/
    HAL_UART_Transmit(&huart2, (uint8_t*)TxData, 9, HAL_MAX_DELAY);
    HAL_Delay(1);
}

void setFree (/*unsigned char ID*/)
{
    uint8_t TxData[8];   // TransmitByteData [1byte]

    TxData[0] = 0x08;          // All Byte
    TxData[1] = 0x04;          // Command
    TxData[2] = 0x00;          // Option/Status
    TxData[3] = 0x00;          // ID
    TxData[4] = 0x02;          //  mode
    TxData[5] = 0x28;          // address
    TxData[6] = 0x01;          // device number
    TxData[7] = 0x37;          // Sum

    HAL_UART_Transmit(&huart1, (uint8_t*)TxData, 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)TxData, 8, HAL_MAX_DELAY);
    HAL_Delay(1);
}

//void setGain (/*unsigned char ID*/){
//  uint8_t TxData[8];   // TransmitByteData [10byte]
//
//    TxData[0] = 0x08;          // All Byte
//    TxData[1] = 0x04;          // Command
//    TxData[2] = 0x00;          // Option/Status
//    TxData[3] = 0x00;          // ID
//    TxData[4] = 0x02;          //  mode
//    TxData[5] = 0x5C;          // address
//    TxData[6] = 0x01;          // device number
//    TxData[7] = 0x69;          // Sum
//
//    HAL_UART_Transmit(&huart1, (uint8_t*)TxData, 8, HAL_MAX_DELAY);
//    HAL_UART_Transmit(&huart2, (uint8_t*)TxData, 8, HAL_MAX_DELAY);
//    HAL_Delay(1);               // Wait for transmission
//}


int main(void)
{
  SD_MPU6050_Result result ;
  uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
  uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};

  float diff = 0; //IMU出力角度
  int16_t dig_x = 0;
  float dig_y = 0;
  int16_t dig_z = 0;
  float lp;
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  //MX_WWDG_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);


  result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_16G,SD_MPU6050_Gyroscope_2000s );

  if(result == SD_MPU6050_Result_Ok)
  {
    HAL_UART_Transmit(&huart2,mpu_ok,(uint16_t)15, HAL_MAX_DELAY);
  }
  else
  {
    HAL_UART_Transmit(&huart2, mpu_not, (uint16_t)17, HAL_MAX_DELAY);
  }

  B3M_RunNormal(huart1, SERVO_ID_1);
  B3M_RunNormal(huart6, SERVO_ID_6);

  SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
  SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);

/*
  int16_t bg_x = mpu1.Gyroscope_X;
  int16_t bg_y = mpu1.Gyroscope_Y;
  int16_t bg_z = mpu1.Gyroscope_Z;
*/
  float bg_x = 0;
  float bg_y = 0;
  float bg_z = 0;

  float ba_x = 0;
  float ba_y = 0;
  float ba_z = 0;

  float g_x = 0;
  float g_y = 0;
  float g_z = 0;

  int16_t gy;

  bg_x = mpu1.Gyroscope_X;
  bg_y = mpu1.Gyroscope_Y;
  bg_z = mpu1.Gyroscope_Z;

  lp = 37;

  /* Infinite loop */
  while (1)
  {
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET){
      B3M_RunNormal(huart1, SERVO_ID_1);
      B3M_RunNormal(huart6, SERVO_ID_6);
      HAL_Delay(100);
    }
    //setModeToNormal();
    //HAL_Delay(1000);
     //HAL_Delay(1);
    //B3M_Write_1B(huart1, SERVO_ID_0, OPERATION_MODE_RunNormal, SERVO_TORQUE_ON);
    //HAL_Delay(100);

    /*サーボ回転角度指定 */

    /*
    B3M_SetDesirePostion(huart1, SERVO_ID_0, 1000);
    B3M_SetDesirePostion(huart6, SERVO_ID_0, 6000);
    HAL_Delay(1000);

    B3M_SetDesirePostion(huart1, SERVO_ID_0, -1000);
    B3M_SetDesirePostion(huart6, SERVO_ID_0, -6000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, RESET);;
    */
    char gyro[20], acc[20];

    SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
    g_x = mpu1.Gyroscope_X;
    g_y = mpu1.Gyroscope_Y;
    g_z = mpu1.Gyroscope_Z;

    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
    //default int16_t
    //HAL_UART_Transmit(&huart2, (uint16_t*)(&g_x), 16,HAL_MAX_DELAY);

    //SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);//1125-915 = 210μs

    /*
    int16_t a_x = mpu1.Accelerometer_X / 4095;
    int16_t a_y = mpu1.Accelerometer_Y / 4095;
    int16_t a_z = mpu1.Accelerometer_Z / 4095;

    int16_t acc_angle_x = atan2(a_x, a_z) * 360 / 2.0 / 3.1415;
    int16_t acc_angle_y = atan2(a_y, a_z) * 360 / 2.0 / 3.1415;
    int16_t acc_angle_z = atan2(a_x, a_y) * 360 / 2.0 / 3.1415;
    */

    //ローパスフィルタ
    //y = exp(-st/tc) * (x-1) + (1-exp(-st/tc)) * val
    //lp = 0.549 * mpu1.Accelerometer_X;

    //加速度角度表記
    //sprintf(acc, "%i\n\r", lp);
    //sprintf(acc, " acc x: %i y: %i z: %i\n\r\n\r", acc_angle_x, acc_angle_y, acc_angle_z);
    //HAL_UART_Transmit(&huart2, (uint8_t*)acc, strlen(acc), HAL_MAX_DELAY);
    //HAL_Delay(50);
/*
    diff   = 0.99 * before + (1 - 0.99) * mpu1.Accelerometer_X ;
    before = diff;
    diff   = diff * 0.549;

    sprintf(gyro, "%i\n\r", diff);
    HAL_UART_Transmit(&huart2, (uint8_t*)gyro, strlen(gyro), HAL_MAX_DELAY);

    if(diff > 9500)
      diff = 9500;

    if(diff < -9500)
      diff = -9500;

    B3M_SetDesirePostion(huart1, SERVO_ID_1, diff);
   //B3M_SetDesirePostion(huart6, SERVO_ID_6, diff);
*/
/*積算*/

  //dig_x += g_x * 0.001;
  //dig_y += (g_y - lp)* 0.00091;
  //dig_z += g_z * 0.001;

//台形近似
  //dig_x += (bg_x + g_x) * 0.0001 / 2 ;
  dig_y += (bg_y + g_y - lp) * 0.00091 / 2/108 ;//0.00091
  //dig_z += (bg_z + g_z) * 0.0001 / 2 ;

  //gy = 0.90 * gy + (1 - 0.90) * dig_y;

  //dig_x += dig_y * sin(g_z * 0.0001 * 3.1415/180);
  //dig_y += dig_x * sin(g_z * 0.0001 * 3.1415/180);

  //ローパスフィルタ
  //y = exp(-st/tc) * (x-1) + (1-exp(-st/tc)) * val
  //lp = 0.549 * mpu1.Accelerometer_X;

  bg_x = g_x;
  bg_y = g_y - lp;
  bg_z = g_z;
  //diff = bg_y*100000;
  //sprintf(gyro, "x = %i, y = %i, z = %i\n\r", dig_x, dig_y, dig_z);
  //sprintf(gyro, " y = %i\n\r", mpu1.Gyroscope_Y);

  //lp = mpu1.Gyroscope_Y;//あいだは2.5μで切り替わりが0.9ms
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);

  gy = dig_y;
  //sprintf(gyro, "%i\n\r", mpu1.Gyroscope_Y);
  sprintf(gyro, " y = %i\n\r", gy);
  HAL_UART_Transmit(&huart2, (uint8_t*)gyro, strlen(gyro), HAL_MAX_DELAY);

  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
  //HAL_Delay(1);

  //lp = mpu1.Gyroscope_Y;
  }

}

/** System Clock Configuration　
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);//voltage smooth

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  //hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */

static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* WWDG init function */
//static void MX_WWDG_Init(void)
//{
//
//  hwwdg.Instance = WWDG;
//  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
//  hwwdg.Init.Window = 64;
//  hwwdg.Init.Counter = 64;
//  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
//  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
