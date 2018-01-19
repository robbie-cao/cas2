/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "lcd.h"
#include "icon.h"
//#include "lcd_font.h"
#if (WHITE_BKG==1)
#include "wlogo.h"
#include "wsymbol.h"
#else
#include "blogo.h"
#include "bsymbol.h"
#endif

#include "voc.h"
#include "sensair.h"
#include "pm25.h"
#include "hih6130.h"

#include "comm.h"

#include "led.h"

#define KEY_RIGHT        HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)
#define KEY_LEFT         HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)
#define KEY_CENTER       HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)

#define PSHOLD_HIGH      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define PSHOLD_LOW       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define LONG_PRESS       150

typedef enum Screen_Update_Mode
{
   FIXED_MODE = 0,
   SCROLL_MODE = 1
} Screen_Update_Mode_t;

extern Font_t bmp_font;

// Assign default value for all sensors for the first time use not to be empty
SensorData_t sensor_data_latest = {
  75.0,         // temperature
  70.0,         // humidity
  500,          // co2
  122,          // tvoc
  501,          // co2eq
  50,           // pm25
  50            // pm10
};
SensorData_t sensor_data_old;
SensorData_t sensor_data_display;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

SRAM_HandleTypeDef hsram1;

osThreadId defaultTaskHandle;
osThreadId sensorTaskHandle;
osThreadId commTaskHandle;
osThreadId displayTaskHandle;
osThreadId purifierTaskHandle;

uint32_t tim3_count = 0;

uint8_t one_byte = 'X';
uint8_t recv_comm_buf[COMM_RECV_BUF_MAX];
uint8_t send_comm_buf[256];
uint8_t recv_comm_idx;
uint8_t start_rcv_timer;
uint8_t rcv_tim_delay;
uint8_t comm_rcv_flag;

volatile uint8_t key_press_flag = 0;

xSemaphoreHandle xSensorDataMutex = NULL;
xSemaphoreHandle xScreenCtrlMutex = NULL;

typedef struct LCD_Screen
{
  uint8_t* cur_icon;
  union value
  {
    float       temp_val;
    float       humd_val;
    uint16_t    co2_val;
    uint16_t    tvoc_val;
    uint16_t    pm25_val;
  } sensor;
  uint8_t cur_index;
  uint8_t cur_symbol;
} LCD_Screen_t;

typedef struct LCD_Display
{
  Screen_Update_Mode_t  mode;
  uint8_t               index_curr;
  uint8_t               index_next;
  uint8_t               quality_level;      // 0 - blue, 1 - yellow, 2 - red
  uint8_t               quality_level_prev; // 0 - blue, 1 - yellow, 2 - red
  uint8_t               symbol_level;       // 0~14
  uint8_t               symbol_level_prev;  // 0~14
  SensorData_t          data_to_display;
  SensorData_t          data_on_screen;
} LCD_Display_t;

struct LCD_Screen screen[5];
struct LCD_Display display;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void SensorTask(void const * argument);
void CommTask(void const * argument);
void PurifierTask(void const * argument);
void DisplayTask(void const * argument);
void Init_Keypad(void);
void Keypad_handler(void);

#if PWRCTRL_MIMIC
volatile uint8_t shutdown_flag = 0;
void system_resume(void);
void system_shutdown(void);
void UpdateDisplay3(uint8_t mode, uint8_t index_curr, uint8_t index_next);
#endif

uint8_t UpdateSymbolLevel(uint8_t symbol_level);
uint8_t UpdateQualityLevel(uint8_t symbol_level);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
#if 0
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
#endif

  return ch;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Screen_Init(void)
{
  /* screen[0] for Temperature */
  screen[0].cur_icon = (uint8_t*)icon_newtemp;
  screen[0].cur_symbol = 0;
  screen[0].sensor.temp_val = sensor_data_latest.temperature;
  screen[0].cur_index = INDEX_0;

  /* screen[1] For Humidity */
  screen[1].cur_icon = (uint8_t*)icon_newhumd;
  screen[1].cur_symbol = 0;
  screen[1].sensor.humd_val = (uint16_t) sensor_data_latest.humidity;
  screen[1].cur_index = INDEX_1;

  /* screen[2] For CO2*/
  screen[2].cur_icon = (uint8_t*)icon_newco2;
  screen[2].cur_symbol = 1;
  screen[2].sensor.co2_val = sensor_data_latest.co2;
  screen[2].cur_index = INDEX_2;

  /* screen[3] For TVOC*/
  screen[3].cur_icon = (uint8_t*)icon_newtvoc;
  screen[3].cur_symbol = 1;
  screen[3].sensor.tvoc_val= sensor_data_latest.tvoc;
  screen[3].cur_index = INDEX_3;

  /* screen[4] For PM25*/
  screen[4].cur_icon = (uint8_t*)icon_newpm25;
  screen[4].cur_symbol = 1;
  screen[4].sensor.pm25_val = sensor_data_latest.pm25;
  screen[4].cur_index = INDEX_4;

  display.mode = FIXED_MODE;
  display.index_curr = 0xFF;
  display.index_next = 0;
  memcpy(&display.data_to_display, &sensor_data_latest, sizeof(SensorData_t));
  memset(&display.data_on_screen, 0, sizeof(SensorData_t));
}

#if PWRCTRL_MIMIC
void system_shutdown(void)
{
  if (xSemaphoreTake(xScreenCtrlMutex, (TickType_t)10) == pdTRUE )
  {
    LCD_Clear(BKG);
    LCD_ShowImage(LOGO_XPOS, LOGO_YPOS, LOGO_WIDTH, LOGO_HEIGHT, (uint8_t*)logo);
    HAL_Delay(1000);
    LCD_MaskImage(LOGO_XPOS, LOGO_YPOS, LOGO_WIDTH, LOGO_HEIGHT, BKG);
    LCD_Switch_Off();
    xSemaphoreGive( xScreenCtrlMutex );
  }
}

void system_resume(void)
{
  if (xSemaphoreTake(xScreenCtrlMutex, (TickType_t)10) == pdTRUE )
  {
    LCD_Clear(BKG);
    LCD_Switch_On();

    LCD_ShowImage(LOGO_XPOS, LOGO_YPOS, LOGO_WIDTH, LOGO_HEIGHT, (uint8_t*)logo);
    HAL_Delay(2000);
    LCD_Clear(BKG);

    display.mode=FIXED_MODE;
    display.index_next = 0;
    display.index_curr = 1;
    UpdateDisplay3(display.mode, display.index_curr, display.index_next);

    xSemaphoreGive( xScreenCtrlMutex );
  }
}
#endif

extern unsigned char g_ucaSeriNo[];
extern uint8_t temp_id_str[];

void Keypad_handler(void)
{
  uint8_t counter;
  osDelay(50);

  if (KEY_RIGHT == 0 && KEY_LEFT == 0) {
    char buf[32];

    for(int i = 0; i<12; i++)
    {
      HEX_2_ASCII(g_ucaSeriNo[i]);

      buf[2*i] = temp_id_str[0];
      buf[2*i+1] = temp_id_str[1];
    }
    buf[24] = 0;
    LCD_Clear(BKG);
    LCD_ShowString(0, 0, 32*24, 32, 32, buf);
    return;
  }
  if (KEY_RIGHT == 0)
  {
    if (xSemaphoreTake(xScreenCtrlMutex, (TickType_t)10) == pdTRUE )
    {
      if (display.index_next >= 0 && display.index_next < 4)
      {
        display.index_next += 1;
      }
      else
      {
        display.index_next = 0;
      }
      xSemaphoreGive( xScreenCtrlMutex );
    }
  }
  else if (KEY_LEFT == 0)
  {
    if (xSemaphoreTake(xScreenCtrlMutex, (TickType_t)10) == pdTRUE )
    {
      if (display.index_next > 0 && display.index_next <= 4)
      {
        display.index_next -= 1;
      }
      else
      {
        display.index_next = 4;
      }
      xSemaphoreGive( xScreenCtrlMutex );
    }
  }
  else if (KEY_CENTER == 0)
  {
    counter= 0;
    while((KEY_CENTER== 0)&& (counter< 0xFF))
    {
      counter++;
      osDelay(20);
    }
    if(counter>= LONG_PRESS)
    {
      PSHOLD_LOW;
#if PWRCTRL_MIMIC
      if(shutdown_flag == 0)
      {
        system_shutdown();
        shutdown_flag= 1;
      }
      else
      {
        system_resume();
        shutdown_flag = 0;
      }
#endif
    }
    else
    {
       if (xSemaphoreTake(xScreenCtrlMutex, (TickType_t)10) == pdTRUE )
       {
          display.mode = !display.mode;
          if (display.mode) {
              HAL_TIM_Base_Start_IT(&htim3);
          } else {
              HAL_TIM_Base_Stop_IT(&htim3);
          }
          xSemaphoreGive( xScreenCtrlMutex );
       }
    }
  }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  Screen_Init();
  Comm_Init();

  LCD_Init();
  LCD_BKL_RESET;

  LCD_Clear(BKG);
  POINT_COLOR = PEN;

  /* USER CODE BEGIN 2 */
  printf("Start...\r\n");


  POINT_COLOR=PEN;

  LCD_ShowImage(LOGO_XPOS, LOGO_YPOS, LOGO_WIDTH, LOGO_HEIGHT, (uint8_t*)logo);
  HAL_Delay(1000);
  LCD_Clear(BKG);

  PM25_StopAutoSend();
  PM25_StopAutoSend();
  PM25_StopAutoSend();
  HAL_Delay(10);
  PM25_StartMeasurement();
  HAL_Delay(100);



  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  xSensorDataMutex = xSemaphoreCreateMutex();
  xScreenCtrlMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(displayTask, DisplayTask, osPriorityNormal, 0, 256);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  osThreadDef(sensorTask, SensorTask, osPriorityNormal, 0, 256);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  osThreadDef(commTask, CommTask, osPriorityNormal, 0, 128);
  commTaskHandle = osThreadCreate(osThread(commTask), NULL);

  osThreadDef(purifierTask, PurifierTask, osPriorityNormal, 0, 128);
  purifierTaskHandle = osThreadCreate(osThread(purifierTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{
  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
    since APB1 prescaler is different from 1.
      TIM3CLK = 2 * PCLK1
      PCLK1 = HCLK / 4
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock /2) /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;

//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Period = 10000 - 1;
  htim3.Init.Prescaler = uwPrescalerValue;
  htim3.Init.ClockDivision = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 9600;
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_NORSRAM_TimingTypeDef Timing;
  FMC_NORSRAM_TimingTypeDef ExtTiming;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 120;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 15;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 120;
  ExtTiming.BusTurnAroundDuration = 15;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure the PS_Hold pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //PSHold Set to high to turn on ST6601 MOSFET
  PSHOLD_HIGH;

  /* Configure the LCD RST pin */
  GPIO_InitStruct.Pin = LCD_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(LCD_RST_PORT, &GPIO_InitStruct);

  /* Configure the LCD BackLight pin */
  GPIO_InitStruct.Pin = LCD_BKL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(LCD_BKL_PORT, &GPIO_InitStruct);

  /* Configure the LED pin */
  GPIO_InitStruct.Pin = LED_PIN_LEFT | LED_PIN_CENTER | LED_PIN_RIGHT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11~ PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
void CommTask(void const * argument)
{
  HAL_UART_Receive_IT(&WIFI_COMM_UART, &one_byte, 1);
  /* Infinite loop */
  for(;;)
  {
    if(comm_rcv_flag)
    {
      Comm_Process();
      comm_rcv_flag = 0;
      Comm_Response();
    } else {
      vTaskDelay(10);
    }
  }
}

void PurifierTask(void const * argument)
{
  uint8_t data[16];
  uint8_t count = 0;

  /* Infinite loop */
  for(;;)
  {
    count++;
    data[0] = (sensor_data_latest.pm25 >> 8) & 0xFF;
    data[1] = (sensor_data_latest.pm25) & 0xFF;
//#if 0
//    data[2] = (sensor_data_latest.tvoc >> 8) & 0xFF;
//    data[3] = (sensor_data_latest.tvoc) & 0xFF;
//#else
//    data[2] = (sensor_data_latest.co2 >> 8) & 0xFF;
//    data[3] = (sensor_data_latest.co2) & 0xFF;
//#endif

    if (count >= 3) {
    data[2] = (sensor_data_latest.tvoc >> 8) & 0xFF;
    data[3] = (sensor_data_latest.tvoc) & 0xFF;
    if (count > 6) count = 0;
    } else {
    data[2] = (sensor_data_latest.co2 >> 8) & 0xFF;
    data[3] = (sensor_data_latest.co2) & 0xFF;
    }
    data[4] = (((uint16_t)(sensor_data_latest.temperature * 10)) >> 8) & 0xFF;
    data[5] = (((uint16_t)(sensor_data_latest.temperature * 10))) & 0xFF;
    data[6] = (((uint16_t)(sensor_data_latest.humidity))) & 0xFF;

    if (sensor_data_latest.co2 > 1200 || sensor_data_latest.tvoc > 900 || sensor_data_latest.pm25 > 150) {
    data[7] = 3;  // 0 - off, 1 - low, 2 - mid, 3 - high
    } else if (sensor_data_latest.co2 > 900 || sensor_data_latest.tvoc > 600 || sensor_data_latest.pm25 > 100) {
    data[7] = 2;  // 0 - off, 1 - low, 2 - mid, 3 - high
    } else if (sensor_data_latest.co2 > 600 || sensor_data_latest.tvoc > 300 || sensor_data_latest.pm25 > 50) {
    data[7] = 1;  // 0 - off, 1 - low, 2 - mid, 3 - high
    } else {
    data[7] = 0;  // 0 - off, 1 - low, 2 - mid, 3 - high
    }

    // checksum
    data[8] = (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) % 256;

    HAL_UART_Transmit(&huart1, data, 9, 0xFFFF);
    HAL_UART_Transmit(&huart6, data, 9, 0xFFFF);
    vTaskDelay(3000);
  }
}

#define STABLE_TIME   (15 * 60)   // 15 minutes

void SensorTask(void const * argument)
{
  static uint32_t time_count = 0;
  static uint8_t init_flag = 0;
  static float init_temp;

  float t, h;
  uint16_t co2, voc, co2eq, pm25, pm10;

  /* Infinite loop */
  for(;;)
  {
    // Read sensor data
    Get_HumiTemp(&h, &t);
    S8_Read(&co2);
    Get_VocData(&co2eq, &voc);
    PM25_Read(&pm25, &pm10);
#if TEMPERATURE_COMPENSATE
    if (!init_flag) {
      init_temp = t;
      init_flag = 1;
    }
    if (init_flag) {
      if (time_count < STABLE_TIME / 2) {
        t = init_temp + (t - init_temp) * time_count * 2 / STABLE_TIME;
      } else {
        t = t - 2.0;
      }
      time_count++;
    }
#endif
#if FAHRENHEIT_DEGREE
    t = t * 1.8 + 32;
#endif

#if TEMPERATURE_NO_DOT
    t = t + 0.5;
#endif

    xSemaphoreTake(xSensorDataMutex, portMAX_DELAY);
    sensor_data_latest.temperature = t;
    sensor_data_latest.humidity = h;
    sensor_data_latest.tvoc = voc;
    sensor_data_latest.co2 = co2;
    sensor_data_latest.pm25 = pm25;
    xSemaphoreGive(xSensorDataMutex);

    printf("T: %.1f, H: %.1f, V: %d, CO2: %d, PM25: %d\r\n",
            sensor_data_latest.temperature,
            sensor_data_latest.humidity,
            sensor_data_latest.tvoc,
            sensor_data_latest.co2,
            sensor_data_latest.pm25);

    if (voc > TVOC_THRESHOLD || co2 > CO2_THRESHOLD || pm25 > PM25_THRESHOLD) {
      LED_LEFT_TOGGLE();
      LED_CENTER_TOGGLE();
      LED_RIGHT_TOGGLE();
    } else {
      LED_LEFT_OFF();
      LED_CENTER_OFF();
      LED_RIGHT_OFF();
    }

    vTaskDelay(2000);
  }
}

uint8_t DisplayDataChanged(void)
{
  uint8_t changed = 0;
  uint8_t screen_index_curr = 0;

  xSemaphoreTake(xScreenCtrlMutex, portMAX_DELAY);
  screen_index_curr = display.index_curr;
  xSemaphoreGive(xScreenCtrlMutex);

  switch (screen_index_curr) {
  case 0:
#if TEMPERATURE_NO_DOT
    changed = (display.data_to_display.temperature != display.data_on_screen.temperature);
#else
    changed = ((uint16_t)display.data_to_display.temperature != (uint16_t)display.data_on_screen.temperature);
#endif
    break;
  case 1:
    changed = ((uint16_t)display.data_to_display.humidity != (uint16_t)display.data_on_screen.humidity);
    break;
  case 2:
    changed = (display.data_to_display.co2 != display.data_on_screen.co2);
    break;
  case 3:
    changed = (display.data_to_display.tvoc != display.data_on_screen.tvoc);
    break;
  case 4:
    changed = (display.data_to_display.pm25 != display.data_on_screen.pm25);
    break;
  default:
    break;

  }

  return changed;
}

void UpdateDataToDisplay(uint8_t index)
{
  xSemaphoreTake(xSensorDataMutex, portMAX_DELAY);

  display.data_to_display.temperature = sensor_data_latest.temperature;
  display.data_to_display.humidity = sensor_data_latest.humidity;
  display.data_to_display.tvoc = sensor_data_latest.tvoc;
  display.data_to_display.co2 = sensor_data_latest.co2;
  display.data_to_display.pm25 = sensor_data_latest.pm25;

  display.symbol_level = UpdateSymbolLevel(index);
  display.quality_level = UpdateQualityLevel(display.symbol_level);

  screen[0].sensor.temp_val = sensor_data_latest.temperature;
  screen[1].sensor.humd_val = (uint16_t)sensor_data_latest.humidity;
  screen[2].sensor.co2_val = sensor_data_latest.co2;
  screen[3].sensor.tvoc_val= sensor_data_latest.tvoc;
  screen[4].sensor.pm25_val = sensor_data_latest.pm25;

  xSemaphoreGive(xSensorDataMutex);
}

uint8_t UpdateSymbolLevel(uint8_t cur_index)
{
  uint8_t level = 0;

#if 0
  switch (cur_index) {
  case 2: // co2
    if (display.data_to_display.co2 >= CO2_THRESHOLD_MAX) {
      level = 24;
    } else if (display.data_to_display.co2 >= CO2_LEVEL_4) {
      level = 20 + (display.data_to_display.co2 - CO2_LEVEL_4) * 4 / (CO2_THRESHOLD_MAX - CO2_LEVEL_4);
    } else if (display.data_to_display.co2 >= CO2_LEVEL_3) {
      level = 15 + (display.data_to_display.co2 - CO2_LEVEL_3) * 4 / (CO2_LEVEL_4 - CO2_LEVEL_3);
    } else if (display.data_to_display.co2 >= CO2_LEVEL_2) {
      level = 10 + (display.data_to_display.co2 - CO2_LEVEL_2) * 4 / (CO2_LEVEL_3 - CO2_LEVEL_2);
    } else if (display.data_to_display.co2 >= CO2_LEVEL_1) {
      level = 5 + (display.data_to_display.co2 - CO2_LEVEL_1) * 4 / (CO2_LEVEL_2 - CO2_LEVEL_1);
    } else {
      level = 0 + (display.data_to_display.co2 - CO2_LEVEL_0) * 4 / (CO2_LEVEL_1 - CO2_LEVEL_0);
    }
    break;
  case 3: // tvoc
    if (display.data_to_display.tvoc >= TVOC_THRESHOLD_MAX) {
      level = 24;
    } else if (display.data_to_display.tvoc >= TVOC_LEVEL_4) {
      level = 20 + (display.data_to_display.tvoc - TVOC_LEVEL_4) * 4 / (TVOC_THRESHOLD_MAX - TVOC_LEVEL_4);
    } else if (display.data_to_display.tvoc >= TVOC_LEVEL_3) {
      level = 15 + (display.data_to_display.tvoc - TVOC_LEVEL_3) * 4 / (TVOC_LEVEL_4 - TVOC_LEVEL_3);
    } else if (display.data_to_display.tvoc >= TVOC_LEVEL_2) {
      level = 10 + (display.data_to_display.tvoc - TVOC_LEVEL_2) * 4 / (TVOC_LEVEL_3 - TVOC_LEVEL_2);
    } else if (display.data_to_display.tvoc >= TVOC_LEVEL_1) {
      level = 5 + (display.data_to_display.tvoc - TVOC_LEVEL_1) * 4 / (TVOC_LEVEL_2 - TVOC_LEVEL_1);
    } else {
      level = 0 + (display.data_to_display.tvoc - TVOC_LEVEL_0) * 4 / (TVOC_LEVEL_1 - TVOC_LEVEL_0);
    }
    break;
  case 4: // pm25
    if (display.data_to_display.pm25 >= PM25_THRESHOLD_MAX) {
      level = 24;
    } else if (display.data_to_display.pm25 >= PM25_LEVEL_4) {
      level = 20 + (display.data_to_display.pm25 - PM25_LEVEL_4) * 4 / (PM25_THRESHOLD_MAX - PM25_LEVEL_4);
    } else if (display.data_to_display.pm25 >= PM25_LEVEL_3) {
      level = 15 + (display.data_to_display.pm25 - PM25_LEVEL_3) * 4 / (PM25_LEVEL_4 - PM25_LEVEL_3);
    } else if (display.data_to_display.pm25 >= PM25_LEVEL_2) {
      level = 10 + (display.data_to_display.pm25 - PM25_LEVEL_2) * 4 / (PM25_LEVEL_3 - PM25_LEVEL_2);
    } else if (display.data_to_display.pm25 >= PM25_LEVEL_1) {
      level = 5 + (display.data_to_display.pm25 - PM25_LEVEL_1) * 4 / (PM25_LEVEL_2 - PM25_LEVEL_1);
    } else {
      level = 0 + (display.data_to_display.pm25 - PM25_LEVEL_0) * 4 / (PM25_LEVEL_1 - PM25_LEVEL_0);
    }
    break;
  default:
    break;
  }

#else

  switch (cur_index) {
  case 0: // temperature
    if (display.data_to_display.temperature >= TEMP_LEVEL_BLUE_LOW && display.data_to_display.temperature <= TEMP_LEVEL_BLUE_HIGH)
    {
      level = 1;
    }
    else
    {
      level = 6;
    }
    break;
  case 1: // humidity
    if (display.data_to_display.humidity >= HUMI_LEVEL_BLUE_LOW && display.data_to_display.humidity <= HUMI_LEVEL_BLUE_HIGH)
    {
      level = 1;
    }
    else
    {
      level = 6;
    }
    break;
  case 2: // co2
    if (display.data_to_display.co2 >= CO2_LEVEL_MAX) {
      level = 14;
    } else if (display.data_to_display.co2 >= CO2_LEVEL_RED) {
      level = 10 + (display.data_to_display.co2 - CO2_LEVEL_RED) * 4 / (CO2_LEVEL_MAX - CO2_LEVEL_RED);
    } else if (display.data_to_display.co2 >= CO2_LEVEL_YELLOW) {
      level = 5 + (display.data_to_display.co2 - CO2_LEVEL_YELLOW) * 4 / (CO2_LEVEL_RED - CO2_LEVEL_YELLOW);
    } else {
      level = 0 + (display.data_to_display.co2) * 4 / CO2_LEVEL_YELLOW;
    }
    break;
  case 3: // tvoc
    if (display.data_to_display.tvoc >= TVOC_LEVEL_MAX) {
      level = 14;
    } else if (display.data_to_display.tvoc >= TVOC_LEVEL_RED) {
      level = 10 + (display.data_to_display.tvoc - TVOC_LEVEL_RED) * 4 / (TVOC_LEVEL_MAX - TVOC_LEVEL_RED);
    } else if (display.data_to_display.tvoc >= TVOC_LEVEL_YELLOW) {
      level = 5 + (display.data_to_display.tvoc - TVOC_LEVEL_YELLOW) * 4 / (TVOC_LEVEL_RED - TVOC_LEVEL_YELLOW);
    } else {
      level = 0 + (display.data_to_display.tvoc) * 4 / TVOC_LEVEL_YELLOW;
    }
    break;
  case 4: // pm25
    if (display.data_to_display.pm25 >= PM25_LEVEL_MAX) {
      level = 14;
    } else if (display.data_to_display.pm25 >= PM25_LEVEL_RED) {
      level = 10 + (display.data_to_display.pm25 - PM25_LEVEL_RED) * 4 / (PM25_LEVEL_MAX - PM25_LEVEL_RED);
    } else if (display.data_to_display.pm25 >= PM25_LEVEL_YELLOW) {
      level = 5 + (display.data_to_display.pm25 - PM25_LEVEL_YELLOW) * 4 / (PM25_LEVEL_RED - PM25_LEVEL_YELLOW);
    } else {
      level = 0 + (display.data_to_display.pm25) * 4 / PM25_LEVEL_YELLOW;
    }
    break;
  default:
    break;
  }

#endif

  return level;
}

uint8_t UpdateQualityLevel(uint8_t symbol_level)
{
  uint8_t level = 0;

  if (symbol_level >= 10) {
    level = 2;
  } else if (symbol_level >= 5) {
    level = 1;
  } else {
    level = 0;
  }

  return level;
}

void UpdateDataOnScreen(uint8_t cur_index)
{
  uint8_t level = UpdateSymbolLevel(cur_index);
  if(screen[cur_index].cur_symbol==1)
  {
    LCD_ShowSymbol(level);
  }

  display.quality_level_prev = display.quality_level;
  display.symbol_level_prev = display.symbol_level;
  memcpy(&display.data_on_screen, &display.data_to_display, sizeof(SensorData_t));
}


void UpdateDisplay(uint8_t mode, uint8_t index_curr, uint8_t index_next)
{
  float curval;
  uint16_t myval;
  uint8_t bit_width;
  char buf[4] = {0};

  POINT_COLOR = PEN;
  if (mode == SCROLL_MODE || index_curr != index_next) {
    // SCROLL mode or FIXED mode for the first time
    // Clear whole screen and redraw icon and bottom slides
    LCD_Clear(BKG);

    memset(buf, 0, sizeof(buf));
    LCD_ShowImage(ICON_SENSOR_XPOS, ICON_SENSOR_YPOS,
                  ICON_SENSOR_WIDTH, ICON_SENSOR_HEIGHT, (uint8_t*)screen[index_next].cur_icon);
    LCD_ShowSlide(screen[index_next].cur_index);

  } else {
    // FIXED mode
    // Only redraw data, not redraw icon and bottom slides
    LCD_Fill(DIGIT_XPOS, DIGIT_YPOS, 480, DIGIT_YPOS+DIGIT_HEIGHT, BKG);
  }

  memset(buf, 0, sizeof(buf));
  switch (index_next) {
  case 0:
    curval = display.data_to_display.temperature;
    sprintf(buf,"%3.1f", curval);
    if (curval < 0 ) //Negative value
    {
      LCD_ShowChar(DIGIT_XPOS, DIGIT_YPOS, '-', 32, 1);
    }
    else if (curval >= 0 && curval < 10)
    {
      bit_width = 2;
    } else if (curval >= 10 && curval < 100)
    {
      bit_width = 3;
    }
    LCD_ShowDigtStr(buf, 1, bit_width);
    break;
  case 1:
  case 2:
  case 3:
  case 4:
    if (index_next == 1) {
      myval = (uint16_t)display.data_to_display.humidity;
    } else if (index_next == 2) {
      myval = display.data_to_display.co2;
      if (myval > CO2_THRESHOLD) {
        POINT_COLOR = HON_RED;
      }
    } else if (index_next == 3) {
      myval = display.data_to_display.tvoc;
      if (myval > TVOC_THRESHOLD) {
        POINT_COLOR = HON_RED;
      }
    } else if (index_next == 4) {
      myval = display.data_to_display.pm25;
      if (myval > PM25_THRESHOLD) {
        POINT_COLOR = HON_RED;
      }
    } else {
      myval = 0;
    }
    sprintf(buf, "%d", myval);
    if (myval < 0) //Negative value
    {
      LCD_ShowChar(DIGIT_XPOS, DIGIT_YPOS, '-', 32, 1);
    }
    else if (myval >= 0 && myval < 10)
    {
      bit_width = 1;
    }
    else if (myval >= 10 && myval < 100)
    {
      bit_width = 2;
    }
    else if (myval >= 100 && myval < 1000)
    {
      bit_width = 3;
    }
    else if (myval >= 1000 && myval < 10000)
    {
      bit_width = 4;
    }
    LCD_ShowDigtStr(buf, 0, bit_width);
    break;
  default:
    break;
  }
}

void UpdateSensorDataDisplay(uint8_t index_next)
{
  uint16_t color = PEN;

  switch (display.quality_level) {
  case 0:
    color = BLUE;
    break;
  case 1:
    color = YELLOW;
    break;
  case 2:
    color = HON_RED;
    break;
  default:
    break;
  }

  switch (index_next) {
  case 0:
    if (display.data_to_display.temperature < 0 ) //Negative value
    {
      LCD_ShowChar(DIGIT_XPOS, DIGIT_YPOS, '-', 32, 1);
    }

    LCD_ShowNumCenterAlign((uint16_t)display.data_to_display.temperature, &bmp_font, color);

    break;
  case 1:
    LCD_ShowNumCenterAlign((uint16_t)display.data_to_display.humidity, &bmp_font, color);

    break;
  case 2:
    LCD_ShowNumCenterAlign(display.data_to_display.co2, &bmp_font, color);

    break;
  case 3:
    LCD_ShowNumCenterAlign(display.data_to_display.tvoc, &bmp_font, color);

    break;
  case 4:
    LCD_ShowNumCenterAlign(display.data_to_display.pm25, &bmp_font, color);

    break;

  default:
    break;
  }
}

void UpdateSensorDataDisplayPartial(uint8_t index_next)
{
  uint16_t color = PEN;

  switch (display.quality_level) {
  case 0:
    color = BLUE;
    break;
  case 1:
    color = YELLOW;
    break;
  case 2:
    color = HON_RED;
    break;
  default:
    break;
  }

  switch (index_next) {
  case 0:

    if (display.quality_level == display.quality_level_prev) {
      LCD_UpdateNumPartialCenterAlign((uint16_t)display.data_to_display.temperature, (uint16_t)display.data_on_screen.temperature, &bmp_font, color);
    } else {
      LCD_Fill(0, BMP_YPOS, 480, BMP_YPOS + BMP_HEIGHT, BKG);
      LCD_ShowNumCenterAlign((uint16_t)display.data_to_display.temperature, &bmp_font, color);
    }

    break;
  case 1:
    if (display.quality_level == display.quality_level_prev) {
      LCD_UpdateNumPartialCenterAlign((uint16_t)display.data_to_display.humidity, (uint16_t)display.data_on_screen.humidity, &bmp_font, color);
    } else {
      LCD_Fill(0, BMP_YPOS, 480, BMP_YPOS + BMP_HEIGHT, BKG);
      LCD_ShowNumCenterAlign((uint16_t)display.data_to_display.humidity, &bmp_font, color);
    }

    break;
  case 2:
    if (display.quality_level == display.quality_level_prev) {

      LCD_UpdateNumPartialCenterAlign(display.data_to_display.co2, display.data_on_screen.co2, &bmp_font, color);

    } else {

      LCD_Fill(0, BMP_YPOS, 480, BMP_YPOS+ BMP_HEIGHT, BKG);
      LCD_ShowNumCenterAlign(display.data_to_display.co2, &bmp_font, color);

    }
    break;
  case 3:
    if (display.quality_level == display.quality_level_prev) {

      LCD_UpdateNumPartialCenterAlign(display.data_to_display.tvoc, display.data_on_screen.tvoc, &bmp_font, color);

    } else {

      LCD_Fill(0, BMP_YPOS, 480, BMP_YPOS +BMP_HEIGHT, BKG);
      LCD_ShowNumCenterAlign(display.data_to_display.tvoc, &bmp_font, color);

    }
    break;
  case 4:
    if (display.quality_level == display.quality_level_prev) {

      LCD_UpdateNumPartialCenterAlign(display.data_to_display.pm25, display.data_on_screen.pm25, &bmp_font, color);

    } else {

      LCD_Fill(0, BMP_YPOS, 480, BMP_YPOS +BMP_HEIGHT, BKG);
      LCD_ShowNumCenterAlign(display.data_to_display.pm25, &bmp_font, color);
    }
    break;

  default:
    break;
  }
}

void UpdateDisplay2(uint8_t mode, uint8_t index_curr, uint8_t index_next)
{
  uint16_t color = PEN;

  POINT_COLOR = PEN;
  if (mode == SCROLL_MODE || index_curr != index_next) {
    // SCROLL mode or FIXED mode for the first time
    // Clear whole screen and redraw icon and bottom slides
    LCD_Clear(BKG);

    LCD_ShowImage(ICON_SENSOR_XPOS, ICON_SENSOR_YPOS,
                  ICON_SENSOR_WIDTH, ICON_SENSOR_HEIGHT, (uint8_t*)screen[index_next].cur_icon);
    LCD_ShowSlide(screen[index_next].cur_index);
  } else {
    // FIXED mode
    // Only redraw data, not redraw icon and bottom slides
    LCD_Fill(0, BMP_YPOS, 480, BMP_YPOS+ BMP_HEIGHT, BKG);
  }

  color = PEN;

  switch (display.quality_level) {
  case 0:
    color = BLUE;
    break;
  case 1:
    color = YELLOW;
    break;
  case 2:
    color = HON_RED;
    break;
  default:
    break;
  }

  switch (index_next) {
  case 0:
    if (display.data_to_display.temperature < 0 ) //Negative value
    {
      LCD_ShowChar(DIGIT_XPOS, DIGIT_YPOS, '-', 32, 1);
    }

    LCD_ShowNumCenterAlign((uint16_t)display.data_to_display.temperature, &bmp_font, color);

    break;
  case 1:

    LCD_ShowNumCenterAlign((uint16_t)display.data_to_display.humidity, &bmp_font, color);

    break;
  case 2:
    LCD_ShowNumCenterAlign(display.data_to_display.co2, &bmp_font, color);

    break;
  case 3:
    LCD_ShowNumCenterAlign(display.data_to_display.tvoc, &bmp_font, color);

    break;
  case 4:

    LCD_ShowNumCenterAlign(display.data_to_display.pm25, &bmp_font, color);
    break;

  default:
    break;
  }
}

void UpdateDisplay3(uint8_t mode, uint8_t index_curr, uint8_t index_next)
{
  uint16_t color = PEN;

  POINT_COLOR = PEN;
  if (mode == SCROLL_MODE || index_curr != index_next) {
    // SCROLL mode or FIXED mode for the first time
    // Clear whole screen and redraw icon and bottom slides

    LCD_Clear(BKG);

    LCD_ShowImage(ICON_SENSOR_XPOS, ICON_SENSOR_YPOS,
                  ICON_SENSOR_WIDTH, ICON_SENSOR_HEIGHT, (uint8_t*)screen[index_next].cur_icon);
    LCD_ShowSlide(screen[index_next].cur_index);
    UpdateSensorDataDisplay(index_next);
  } else {
    // FIXED mode
    // Only redraw data, not redraw icon and bottom slides
    UpdateSensorDataDisplayPartial(index_next);
  }

}


void DisplayTask(void const * argument)
{

  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
    uint8_t display_mode = 0;
    uint8_t screen_index_curr = 0;
    uint8_t screen_index_next = 0;

    xSemaphoreTake(xScreenCtrlMutex, portMAX_DELAY);
    display_mode = display.mode;
    screen_index_curr = display.index_curr;
    screen_index_next = display.index_next;
    xSemaphoreGive(xScreenCtrlMutex);

    if (screen_index_curr == screen_index_next) {
      // Stay at one sensor and update data if changed
      UpdateDataToDisplay(screen_index_next);
      if (DisplayDataChanged()) {
        UpdateDisplay3(display_mode, screen_index_curr, screen_index_next);
        UpdateDataOnScreen(screen_index_next);
      }
      vTaskDelay(50);
      continue ;
    }

    // Switch sensor and update whole screen
    UpdateDataToDisplay(screen_index_next);
    UpdateDisplay2(display_mode, screen_index_curr, screen_index_next);
    UpdateDataOnScreen(screen_index_next);

    xSemaphoreTake(xScreenCtrlMutex, portMAX_DELAY);
    display.index_curr = display.index_next;
    xSemaphoreGive(xScreenCtrlMutex);

    vTaskDelay(50);
  }
  /* USER CODE END 5 */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    if (key_press_flag)
    {
       Keypad_handler();
       key_press_flag = 0;
    }
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3) {
    printf(".");
    tim3_count++;
    if (tim3_count >= 3) {
      display.index_next += 1;
      if (display.index_next >= 5) {
        display.index_next = 0;
      }
      tim3_count = 0;
    }
  }

/* USER CODE END Callback 1 */
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if (UartHandle->Instance == WIFI_COMM_UART.Instance) {
    /* Set transmission flag: transfer complete*/
    printf("%c", one_byte);
    if(!comm_rcv_flag)
    {
      recv_comm_buf[recv_comm_idx++] = one_byte;
      if(recv_comm_idx == COMM_RECV_BUF_MAX)
      {
        recv_comm_idx = 0;
      }
    }
    start_rcv_timer = 1;
    rcv_tim_delay = 0;
    HAL_UART_Receive_IT(&WIFI_COMM_UART, &one_byte, 1);
  }

}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
  //BSP_LED_On(LED3);
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case GPIO_PIN_11:
  case GPIO_PIN_12:
  case GPIO_PIN_13:
    key_press_flag = 1;
    break;

  default:
    break;
  }
}

void Timer_1MS_ISR(void)
{
  /* UART end of receive check */
  if(start_rcv_timer)
  {
    rcv_tim_delay++;
    if(rcv_tim_delay >= REC_TIM_DELAY)
    {
      start_rcv_timer = 0;
      comm_rcv_flag = 1;
    }
  }
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
