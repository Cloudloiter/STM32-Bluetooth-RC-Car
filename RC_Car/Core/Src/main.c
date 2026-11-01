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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PWM_MAX          1000   /* ARR+1：PWM 满量程 */
#define SPEED            600    /* 默认速度（占空比 60%） */
#define CAR_TIMEOUT_MS   800    /* 超过这么久没收到指令 → 自动停车（断连保护） */
#define SPEED_STEP       111    /* 档位 1~9 每档对应的 PWM 值（9×111≈999） */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint8_t  rx_byte;                    /* 串口中断每收到 1 字节就存这里 */
volatile uint8_t  car_cmd = 'S';     /* 最新收到的指令（volatile：中断和主循环共享） */
volatile uint32_t last_cmd_tick = 0; /* 最后一次收到指令的时刻（ms） */

static uint16_t g_speed = SPEED;     /* 当前速度 */
static uint8_t  g_dir   = 0;         /* 当前方向：0停 1前 -1后 2左 3右 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void Motor_Left(int dir);
static void Motor_Right(int dir);
static void Car_Run(int dir, uint16_t speed);
static void Car_TurnLeft(uint16_t speed);
static void Car_TurnRight(uint16_t speed);
static void Car_Stop(void);

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
  Car_Stop(); 

  /* 打开串口中断接收：收满 1 字节会自动调用 HAL_UART_RxCpltCallback */
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  last_cmd_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
     
    /* USER CODE BEGIN 3 */
    uint8_t cmd = car_cmd;            /* 先拷贝一份，避免读的过程中被中断改掉 */

    /* ---- 1. 解析指令 ---- */
    if (cmd >= '0' && cmd <= '9')     /* 数字键 = 调速，不改变行驶方向 */
    {
      g_speed = (uint16_t)(cmd - '0') * SPEED_STEP;
      if (g_speed < 250) g_speed = 250;   /* 0 也保底给一点速度，否则等于停车 */
    }

    switch (cmd)
    {
      case 'F': g_dir = 1;  break;    /* 前进 */
      case 'B': g_dir = -1; break;    /* 后退 */
      case 'L': g_dir = 2;  break;    /* 原地左转 */
      case 'R': g_dir = 3;  break;    /* 原地右转 */
      case 'S': g_dir = 0;  break;    /* 停车 */
      default: break;                 /* 其他字符忽略 */
    }

    /* ---- 2. 断连保护：太久没收到指令就停车 ---- */
    if ((HAL_GetTick() - last_cmd_tick) > CAR_TIMEOUT_MS)
    {
      g_dir = 0;
    }

    /* ---- 3. 按当前方向和速度执行 ---- */
    switch (g_dir)
    {
      case 1:  Car_Run(1,  g_speed);    break;
      case -1: Car_Run(-1, g_speed);    break;
      case 2:  Car_TurnLeft(g_speed);   break;
      case 3:  Car_TurnRight(g_speed);  break;
      default: Car_Stop();              break;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Motor_Left(int dir)          /* 左轮方向：IN1/IN2 */
{
  if (dir > 0) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  } else if (dir < 0) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  }
}

static void Motor_Right(int dir)         /* 右轮方向：IN3/IN4 */
{
  if (dir > 0) {
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
  } else if (dir < 0) {
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
  }
}

static void Car_Run(int dir, uint16_t speed)
{
  Motor_Left(dir);
  Motor_Right(dir);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);  /* 改占空比=调速 */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

static void Car_TurnLeft(uint16_t speed)   /* 左轮后退 + 右轮前进 = 原地左转 */
{
  Motor_Left(-1);
  Motor_Right(1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

static void Car_TurnRight(uint16_t speed)  /* 左轮前进 + 右轮后退 = 原地右转 */
{
  Motor_Left(1);
  Motor_Right(-1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

static void Car_Stop(void)
{
  Motor_Left(0);
  Motor_Right(0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);      /* 占空比归零 */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

/**
  * @brief  串口接收完成回调（HAL 库在中断里调用它）
  *         收满 1 字节 → 存指令、记时间 → 立刻重新武装接收
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    car_cmd = rx_byte;                       /* 存下收到的字符 */
    last_cmd_tick = HAL_GetTick();           /* 记录时间，用于断连保护 */

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* 关键：重新开一次接收 */
  }
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
