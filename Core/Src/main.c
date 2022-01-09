/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  insert Human Activity Recognition : 2022/01/06 , by DIHASYS, JinHo KANG
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dfsdm.h"
#include "i2c.h"
#include "octospi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsl.h"
#include "b_l4s5i_iot01a_bus.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void LCD_line1_clear (void);
void LCD_backlight_RGB(char r, char g, char b);   // 0 ~ 255
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTOF(__BUFFER__)  (sizeof(__BUFFER__)/sizeof(*(__BUFFER__)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  LSM6DSL_Object_t MotionSensor;
  volatile uint32_t dataRdyIntReceived;

  volatile uint16_t TIM15_1msec_uart2 = 0;
  volatile uint16_t TIM15_OP_LED_cnt = 0;

  ai_handle network;
  float aiInData[AI_NETWORK_IN_1_SIZE];
  float aiOutData[AI_NETWORK_OUT_1_SIZE];
  uint8_t activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
  const char* activities[AI_NETWORK_OUT_1_SIZE] = {
    "stationary", "walking", "running"
  };

  uint8_t  rx_data;

//  volatile char LCD_buffer[32+1];  // 16 * 2 line + 1 null
  char LCD_clear[] = "|-";
  char LCD_data[] = "                by DIHASYS JinHo\r\n";  // 1 & 2 Lines

  char LCD_line1[17];

  uint8_t LCD_color, LCD_color_old; // 0:white(stationary), 1:green(walking), 2:red(running)

  char tempDATA1[40];
  char tempDATA2[40];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
static void MEMS_Init(void);
static void AI_Init(ai_handle w_addr, ai_handle act_addr);
static void AI_Run(float *pIn, float *pOut);
static uint32_t argmax(const float * values, uint32_t len);
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

  LSM6DSL_Axes_t acc_axes;

  int acc_axes_x_old, acc_axes_y_old, acc_axes_z_old;  // temp
  uint8_t acc_xyz_change_f = 0;

//  int i;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_OCTOSPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_TIM15_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);  // initial
  HAL_TIM_Base_Start_IT(&htim15);

  LCD_line1_clear();

  printf("by programmed DIHASYS , JinHo KANG\r\nStart-->\r\n");

  // LCD_backlight_RGB(0, 0, 0);   // 0 ~ 255
  LCD_backlight_RGB(128, 128, 128);   // 0 ~ 255

  LCD_color = LCD_color_old = 0; // 0:white(stationary), 1:green(walking), 2:red(running)

  HAL_UART_Transmit(&huart2, (uint8_t *)LCD_clear, (COUNTOF(LCD_clear)-1), 50);

  bzero(tempDATA1, 40);
  bzero(tempDATA2, 40);

  HAL_UART_Transmit(&huart2, (uint8_t *)LCD_data, (COUNTOF(LCD_data)-1), 50);
  TIM15_1msec_uart2 = 0;

  dataRdyIntReceived = 0;
  MEMS_Init();

  AI_Init(ai_network_data_weights_get(), activations);

  uint32_t write_index = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (dataRdyIntReceived != 0)
    {
      if (dataRdyIntReceived != 1)
      {
        printf("Overrun error: new data available before reading previous data.\r\n");
        strcpy(tempDATA2," Overrun error: Error_Handler()");          Error_Handler();
      }

      dataRdyIntReceived = 0;
      LSM6DSL_ACC_GetAxes(&MotionSensor, &acc_axes);
      // printf("% 5d, % 5d, % 5d\r\n",  (int) acc_axes.x, (int) acc_axes.y, (int) acc_axes.z);

      /* Normalize data to [-1; 1] and accumulate into input buffer */
      /* Note: window overlapping can be managed here */
      aiInData[write_index + 0] = (float) acc_axes.x / 4000.0f;
      aiInData[write_index + 1] = (float) acc_axes.y / 4000.0f;
      aiInData[write_index + 2] = (float) acc_axes.z / 4000.0f;
      write_index += 3;

      if (write_index == AI_NETWORK_IN_1_SIZE) {
        write_index = 0;

        printf("Running inference\r\n");
        AI_Run(aiInData, aiOutData);

        /* Output results */
        for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
          printf("%8.6f ", aiOutData[i]);
        }
        uint32_t class = argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);
        printf(": %d - %s\r\n", (int) class, activities[class]);

        // for LCD (2 Line * 16 char ) backlight_RGB => uart2 , 9600bps
        strcpy(LCD_line1, activities[class]);  // "stationary", "walking", "running"
        acc_axes_x_old = acc_axes_y_old = acc_axes_z_old = 0; // for LCD display
        LCD_color = class; // 0:white(stationary), 1:green(walking), 2:red(running)
        if(LCD_color != LCD_color_old)
        {
          LCD_color_old = LCD_color;

          switch(LCD_color) // 0:white(stationary), 1:green(walking), 2:red(running)
          {
            case 0:
              LCD_backlight_RGB(80, 80, 80);   // 0 ~ 255
              break;
            case 1:
              LCD_backlight_RGB(0, 255, 0);   // 0 ~ 255
              break;
            case 2:
              LCD_backlight_RGB(255, 0, 0);   // 0 ~ 255
              break;
          }
        }
      }
    }


    // for LCD (2 Line * 16 char ) display => uart2 , 9600bps
    if(TIM15_1msec_uart2 > 70)  // 2 line * (23+2)char
    {
      acc_xyz_change_f = 0;
      if((int) acc_axes.x != acc_axes_x_old) acc_xyz_change_f++;
      if((int) acc_axes.y != acc_axes_y_old) acc_xyz_change_f++;
      if((int) acc_axes.z != acc_axes_z_old) acc_xyz_change_f++;
      if(acc_xyz_change_f != 0)
      {
       HAL_UART_Transmit(&huart2, (uint8_t *)LCD_clear, (COUNTOF(LCD_clear)-1), 50);
       // strncpy(LCD_line1, "      ", 6);
       strcpy(tempDATA1,LCD_line1);
       sprintf(tempDATA2,"%5d%5d%5d ",  (int) acc_axes.x, (int) acc_axes.y, (int) acc_axes.z);
       strcat(tempDATA2,tempDATA1);
       HAL_UART_Transmit(&huart2, (uint8_t *)tempDATA2, strlen(tempDATA2), 50);
       acc_axes_x_old = (int) acc_axes.x;
       acc_axes_y_old = (int) acc_axes.y;
       acc_axes_z_old = (int) acc_axes.z;
      }
      TIM15_1msec_uart2 = 0;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(TIM15_OP_LED_cnt > 500)  // 0.001s * 500 = per 0.5sec
    {
      TIM15_OP_LED_cnt = 0;
      HAL_GPIO_TogglePin(LD2_Green_GPIO_Port, LD2_Green_Pin);  // op_led
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    strcpy(tempDATA2,"HAL_PWREx_.... : Error_Handler()");          Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    strcpy(tempDATA2,"HAL_RCC_Osc....: Error_Handler()");          Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    strcpy(tempDATA2,"HAL_RCC_Clock..: Error_Handler()");          Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    strcpy(tempDATA2,"HAL_RCCEx_Peri.: Error_Handler()");          Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void MEMS_Init(void)
{
  LSM6DSL_IO_t io_ctx;
  uint8_t id;
  LSM6DSL_AxesRaw_t axes;

  /* Link I2C functions to the LSM6DSL driver */
  io_ctx.BusType     = LSM6DSL_I2C_BUS;
  io_ctx.Address     = LSM6DSL_I2C_ADD_L;
  io_ctx.Init        = BSP_I2C2_Init;
  io_ctx.DeInit      = BSP_I2C2_DeInit;
  io_ctx.ReadReg     = BSP_I2C2_ReadReg;
  io_ctx.WriteReg    = BSP_I2C2_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx);

  /* Read the LSM6DSL WHO_AM_I register */
  LSM6DSL_ReadID(&MotionSensor, &id);
  if (id != LSM6DSL_ID) {
    strcpy(tempDATA2,"No id LSM6DSL :  Error_Handler()");          Error_Handler();
  }

  /* Initialize the LSM6DSL sensor */
  LSM6DSL_Init(&MotionSensor);

  /* Configure the LSM6DSL accelerometer (ODR, scale and interrupt) */
  LSM6DSL_ACC_SetOutputDataRate(&MotionSensor, 26.0f); /* 26 Hz */
  LSM6DSL_ACC_SetFullScale(&MotionSensor, 4);          /* [-4000mg; +4000mg] */

  LSM6DSL_Set_DRDY_Mode(&MotionSensor, 1);             /* DRDY pulsed mode */

  LSM6DSL_ACC_Set_INT1_DRDY(&MotionSensor, ENABLE);    /* Enable DRDY */
  LSM6DSL_ACC_GetAxesRaw(&MotionSensor, &axes);        /* Clear DRDY */

  /* Start the LSM6DSL accelerometer */
  LSM6DSL_ACC_Enable(&MotionSensor);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (LSM6DSL_INT1_EXTI11_Pin) :
      dataRdyIntReceived++;
      break;

    case (BUTTON_EXTI13_Pin) :
      HAL_GPIO_TogglePin(LD1_Green_GPIO_Port, LD1_Green_Pin);
      break;

  default :
    break;
  }
}

static void AI_Init(ai_handle w_addr, ai_handle act_addr)
{
  ai_error err;

  /* 1 - Create an instance of the model */
  err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    printf("ai_network_create error - type=%d code=%d\r\n", err.type, err.code);
    strcpy(tempDATA2,"ai create err :  Error_Handler()");    Error_Handler();
  }

  /* 2 - Initialize the instance */
  const ai_network_params params = AI_NETWORK_PARAMS_INIT(
    AI_NETWORK_DATA_WEIGHTS(w_addr),
    AI_NETWORK_DATA_ACTIVATIONS(act_addr)
  );

  if (!ai_network_init(network, &params)) {
    err = ai_network_get_error(network);
    printf("ai_network_init error - type=%d code=%d\r\n", err.type, err.code);
    strcpy(tempDATA2,"ai init error :  Error_Handler()");    Error_Handler();
  }
}

static void AI_Run(float *pIn, float *pOut)
{
  ai_i32 batch;
  ai_error err;

  /* 1 - Create the AI buffer IO handlers with the default definition */
  ai_buffer ai_input[AI_NETWORK_IN_NUM] = AI_NETWORK_IN;
  ai_buffer ai_output[AI_NETWORK_OUT_NUM] = AI_NETWORK_OUT;

  /* 2 - Update IO handlers with the data payload */
  ai_input[0].n_batches = 1;
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].n_batches = 1;
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    err = ai_network_get_error(network);
    printf("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
    strcpy(tempDATA2,"ai run error :   Error_Handler()");    Error_Handler();
  }
}

static uint32_t argmax(const float * values, uint32_t len)
{
  float max_value = values[0];
  uint32_t max_index = 0;
  for (uint32_t i = 1; i < len; i++) {
    if (values[i] > max_value) {
      max_value = values[i];
      max_index = i;
    }
  }
  return max_index;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    HAL_UART_Transmit(&huart1, &rx_data, 1,10);
  }
}

// for printf()
int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM15)  // per 0.001sec
  {
    TIM15_1msec_uart2++;
    TIM15_OP_LED_cnt++;
  }
}

void LCD_line1_clear (void)
{
  uint8_t i;
  for(i=0; i < 16; i++) LCD_line1[i] = 0x20;
  LCD_line1[16] = 0;  //NULL
}

void LCD_backlight_RGB(char r, char g, char b)   // 0 ~ 255
{
  tempDATA1[0] = '|'; tempDATA1[1] = '+';
  tempDATA1[2] = r;
  tempDATA1[3] = g;
  tempDATA1[4] = b;
  HAL_UART_Transmit(&huart2, (uint8_t*)tempDATA1, 5,100);
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

  LCD_backlight_RGB(128, 128, 128);   // 0 ~ 255

  HAL_UART_Transmit(&huart2, (uint8_t *)LCD_clear, (COUNTOF(LCD_clear)-1), 50);

//  strcpy(tempDATA2, " Overrun error  Error_Handler() ");
  HAL_UART_Transmit(&huart2, (uint8_t *)tempDATA2, strlen(tempDATA2), 50);

  __disable_irq();
  while(1) {
    HAL_GPIO_TogglePin(LD2_Green_GPIO_Port, LD2_Green_Pin);
    HAL_Delay(50); /* wait 50 ms ( 0.05sec ) */
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

