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
#define SPEED            1000   /* 默认速度（占空比 100%）。排查阶段先拉满，确认电机能转
                                   再按需往下调；正式玩的时候建议 600~800 */
#define CAR_TIMEOUT_MS   10000  /* 指令保持时间 / 断连保护，单位 ms
                                   · 收到方向指令后车一直按该方向走，直到收到 S 或别的方向
                                   · 只有超过这么久一个合法指令都收不到，才自动停车当兜底
                                   · 手机 App 不支持连发时，这个值就等于"一次指令能走多久"：
                                     10 秒足够看清动作，手机真掉线了车也不会跑飞 */
#define SPEED_STEP       111    /* 档位 1~9 每档对应的 PWM 值（9×111≈999） */
#define SPEED_MIN        300    /* 最低运行速度。低于这个占空比电机维持不住转动
                                   （实测 250 即约 1V 只能空响，转不起来） */
#define KICK_MS          150    /* 起步 boost 持续时间，单位 ms
                                   从静止切换运动时先给满占空比冲刺这么久，
                                   帮电机突破静摩擦，之后回落到目标档位 */

/* 调试回显开关：1=手机发什么字符，单片机就原样发回去（用来确认蓝牙链路双向通）
   验收阶段保持 1；车能正常开了以后改成 0，模组就只会"听"不会"说" */
#define DEBUG_UART_ECHO  1

/* 每秒心跳：1=单片机每秒往手机发一次 "OK\r\n"
   注意这会和回显混在一起，手机屏幕上会一直刷。排查阶段够用了，现在关掉 */
#define DEBUG_UART_HEARTBEAT 0

/* ===== 广播率测试：改这一个数就能试不同波特率 =====
   HC-05 常见出厂值：9600。被改过的话常见 38400 / 115200 / 57600 / 4800
   判断标准：手机里能稳定看到"每秒一个 OK"，就是对的，其余都是错的 */
#define UART_BAUD_TEST   9600

/* ===== 静默诊断开关 =====
   置 1：单片机完全不发任何串口数据，只用板载 LED 快闪表示程序在跑。
         2026-11-01 已验证结论：静默时手机不再刷乱码 → 乱码确实是单片机发出的数据
   置 0：关闭静默（现在已定位完成，改回 0） */
#define UART_SILENT_TEST 0

/* ===== 波特率自动扫描（当前 = 1，正在找模块真正的波特率）=====
   置 1：固件每 5 秒自动换一个波特率，同时发 "OK"，用 LED 闪几下表示当前是第几个：
             1 = 9600     2 = 19200    3 = 38400
             4 = 57600    5 = 115200   6 = 4800
         手机里出现干干净净的 "OK" 时，回想 LED 刚才闪了几下，就知道模块是多少。
   置 0：关闭扫描（找到正确值后，把 UART_BAUD_TEST 改成它，再把这个改 0） */
#define UART_SCAN_MODE   0

/* ===== 单片机串口自检（当前 = 0，已完成使命）=====
   2026-11-01 结论：自检通过（PC13 每 2 秒快闪 3 下）→ PA9/PA10 收发正常。
   同日用 USB-TTL 直连模块跑 AT 指令，8 条全回 OK（固件 2.0-20100601，
   UART=9600,0,0，ROLE=0，PSWD=1234）→ 模块也是好的。
   所以现在改回 0，进入正常工作模式。 */
#define UART_SELFTEST_MODE 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t  rx_byte;                    /* 串口中断每收到 1 字节就存这里 */
volatile uint8_t  car_cmd = 'S';     /* 最新收到的指令（volatile：中断和主循环共享） */
volatile uint32_t last_cmd_tick = 0; /* 最后一次收到指令的时刻（ms） */

/* 收到"任意"字节的累计数（不管合不合法、不管是不是噪声）。
   主循环盯它变化与否，就能判断线上到底有没有数据进来 —— 这是排查链路的关键仪表 */
volatile uint32_t rx_any_cnt = 0;

static uint16_t g_speed = SPEED;     /* 当前速度 */
/* 注意必须是 int8_t（有符号）：要能存 -1。用 uint8_t 存 -1 会变成 255，后退永远失效 */
static int8_t   g_dir   = 0;         /* 当前方向：0停 1前 -1后 2左 3右 */

#if DEBUG_UART_ECHO
volatile uint8_t echo_pending = 0;   /* 中断置 1，主循环负责把收到的字符发回手机 */
volatile uint8_t echo_char    = 0;   /* 要回显的字符 */
#endif

#if UART_SELFTEST_MODE
volatile uint32_t selftest_rx_cnt = 0;   /* 自检时：成功收回来的 0x55 计数 */
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static void Motor_Left(int dir);
static void Motor_Right(int dir);
static void Car_Run(int dir, uint16_t speed);
static void Car_TurnLeft(uint16_t speed);
static void Car_TurnRight(uint16_t speed);
static void Car_Stop(void);
static uint8_t Cmd_IsValid(uint8_t c);
#if UART_SCAN_MODE
static void UART_ScanApply(uint8_t idx);
#endif

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
  Car_Stop(); 

  /* 打开串口中断接收：收满 1 字节会自动调用 HAL_UART_RxCpltCallback */

  /* ---- 临时覆盖波特率（在 USER CODE 区里，CubeMX 重新生成也不会被冲掉）---- */
  huart1.Init.BaudRate = UART_BAUD_TEST;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }

  /* ============ 关键修复：USART1_RX(PA10) 必须开内部上拉 ============
     CubeMX 默认把它生成成 GPIO_NOPULL（浮空输入），这是有问题的：
       · HC-05 模块的 TXD 输出串了限流电阻，驱动能力偏弱
       · 引脚浮空时，整条线（PA10 + 杜邦线 + 模块TXD）会被吊在 0.99V 左右
       · STM32 判不出高电平，UART 一个字节也解析不出来 → 看起来"完全没数据"
     佐证：同一根线上接 CH340 就能收到数据，因为 CH340 的 RXD 内部自带
           上拉电阻，把弱驱动信号拉起来了。换成 STM32 浮空输入就废了。
     注意：本段必须在 HAL_UART_Init() 之后执行，因为 MspInit 里会重新配 GPIO。
     ================================================================= */
  {
    GPIO_InitTypeDef rx_init = {0};
    rx_init.Pin  = GPIO_PIN_10;          /* USART1_RX */
    rx_init.Mode = GPIO_MODE_INPUT;
    rx_init.Pull = GPIO_PULLUP;          /* 空闲时被拉到 3.3V，才能识别起始位 */
    HAL_GPIO_Init(GPIOA, &rx_init);
  }

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  last_cmd_tick = HAL_GetTick();

  /* 诊断用：把板载 LED（PC13）配成输出，作为"程序还活着"的指示灯 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  {
    GPIO_InitTypeDef led_init = {0};
    led_init.Pin   = GPIO_PIN_13;
    led_init.Mode  = GPIO_MODE_OUTPUT_PP;
    led_init.Pull  = GPIO_NOPULL;
    led_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &led_init);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* PC13 低电平点亮，先灭掉 */
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t cmd = car_cmd;            /* 先拷贝一份，避免读的过程中被中断改掉 */

    /* ================= 板载 LED 诊断仪表（PC13，低电平点亮） =================
       灯默认灭，只有两种情况会亮 150ms 然后自己灭：

       ① 每 2 秒亮一次（心跳）→ 证明程序在跑、没卡死
          灯从来不亮 = 没供电 / 固件没烧进去 / 程序死在启动阶段

       ② 每收到一个串口字节就亮一次 → 证明单片机从 PA9 真收到数据了

       怎么读：
         没发数据时        → 每 2 秒稳稳闪一下         = 单片机活着
         手机发数据时      → 闪成一片、密集闪烁         = 收到了，链路通
         手机发数据时      → 还是每 2 秒一下、没变化    = 单片机压根没收到 → PA9 断了
         收到数据有闪、手机却无回显 → PA10 到模块 TXD 这条线断了
       ==================================================================== */
    static uint32_t beat_tick    = 0;   /* 心跳计时 */
    static uint32_t led_off_tick = 0;   /* 灭灯计时 */
    static uint32_t last_rx_any  = 0;   /* 上一次看到的收字节计数 */
    static uint8_t  led_on       = 0;   /* 灯当前是否亮着 */

    uint8_t want_on = 0;

    if ((HAL_GetTick() - beat_tick) >= 2000)        /* ① 心跳：每 2 秒 */
    {
      beat_tick = HAL_GetTick();
      want_on = 1;
    }

    if (rx_any_cnt != last_rx_any)                  /* ② 收到字节：来一个闪一次 */
    {
      last_rx_any = rx_any_cnt;
      want_on = 1;
    }

    if (want_on && !led_on)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   /* 低电平点亮 */
      led_on = 1;
      led_off_tick = HAL_GetTick();
    }
    else if (led_on && ((HAL_GetTick() - led_off_tick) >= 150))
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);     /* 高电平熄灭 */
      led_on = 0;
    }

#if UART_SCAN_MODE && !UART_SELFTEST_MODE
    /* ---- 波特率自动扫描 ----
       每 5 秒换一个波特率，换的时候 LED 先闪 N 下，N 就是当前编号：
       1=9600  2=19200  3=38400  4=57600  5=115200  6=4800
       手机上看到干净的 "OK" 时，回想 LED 闪了几下即可 */
    static uint8_t  scan_idx   = 0;
    static uint32_t scan_tick  = 0;
    static uint32_t send_tick  = 0;
    static uint8_t  blink_left = 0;
    static uint32_t blink_tick = 0;
    static uint8_t  first_run  = 1;

    if (first_run)                    /* 第一轮：9600，先闪 1 下 */
    {
      first_run  = 0;
      blink_left = 2;
      blink_tick = HAL_GetTick();
      scan_tick  = HAL_GetTick();
      send_tick  = HAL_GetTick();
    }

    if ((HAL_GetTick() - scan_tick) >= 5000)
    {
      scan_tick  = HAL_GetTick();
      scan_idx   = (uint8_t)((scan_idx + 1U) % 6U);
      UART_ScanApply(scan_idx);                       /* 切波特率 */
      blink_left = (uint8_t)((scan_idx + 1U) * 2U);   /* 闪 N 下 = 翻转 2N 次 */
      blink_tick = HAL_GetTick();
      send_tick  = HAL_GetTick();
    }

    if ((blink_left > 0U) && ((HAL_GetTick() - blink_tick) >= 300))
    {
      blink_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      blink_left--;
    }

    if ((HAL_GetTick() - send_tick) >= 1000)          /* 每秒发一次 OK */
    {
      send_tick = HAL_GetTick();
      if (HAL_UART_Transmit(&huart1, (uint8_t *)"OK\r\n", 4, 50) != HAL_OK)
      {
        huart1.gState = HAL_UART_STATE_READY;         /* 防卡死在 BUSY_TX */
      }
    }
#endif

#if UART_SELFTEST_MODE
    /* ---- 单片机串口自检：自己发 0x55，从 PA10 收回来 ----
       前提：PA9 与 PA10 用一根杜邦线短接，蓝牙模块的数据线全部拔掉 */
    static uint32_t st_tx_tick = 0, st_chk_tick = 0, st_blink_tick = 0;
    static uint8_t  st_blink = 0;

    if ((HAL_GetTick() - st_tx_tick) >= 300)
    {
      uint8_t u = 0x55;
      st_tx_tick = HAL_GetTick();
      if (HAL_UART_Transmit(&huart1, &u, 1, 5) != HAL_OK)
      {
        huart1.gState = HAL_UART_STATE_READY;
      }
    }

    if ((HAL_GetTick() - st_chk_tick) >= 2000)   /* 每 2 秒结算一次 */
    {
      st_chk_tick = HAL_GetTick();
      st_blink = (selftest_rx_cnt >= 3U) ? 6U : 2U;   /* 通：翻转6次(闪3下)；不通：翻转2次(闪1下) */
      selftest_rx_cnt = 0;
      st_blink_tick = HAL_GetTick();
    }

    if ((st_blink > 0U) && ((HAL_GetTick() - st_blink_tick) >= 120))
    {
      st_blink_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      st_blink--;
    }
#endif

#if UART_SILENT_TEST
    /* ---- 静默诊断：一个字节都不发，只用 LED 快闪表示程序在跑 ----
       如果这时候手机上还有乱码，那就跟单片机发送无关，是线路噪声/模块自己吐的 */
    static uint32_t led_tick = 0;
    if ((HAL_GetTick() - led_tick) >= 150)
    {
      led_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
#endif

#if DEBUG_UART_ECHO && !UART_SILENT_TEST && !UART_SCAN_MODE && !UART_SELFTEST_MODE
    /* ---- 0. 调试专用：回显 + 每秒心跳（链路通了以后把 DEBUG_UART_ECHO 改 0 即可关掉） ---- */
    static uint32_t dbg_tick = 0;

    if (echo_pending)                 /* 收到字符 → 原样发回手机 */
    {
      uint8_t ch = echo_char;
      echo_pending = 0;
      /* HAL 的已知坑：发送超时后 gState 会卡在 BUSY_TX，之后所有发送都被拒。
         所以失败后手动复位状态，否则会"突然彻底发不出去" */
      if (HAL_UART_Transmit(&huart1, &ch, 1, 5) != HAL_OK)
      {
        huart1.gState = HAL_UART_STATE_READY;
      }
    }

    if (DEBUG_UART_HEARTBEAT && (HAL_GetTick() - dbg_tick) >= 1000)   /* 每秒发一次 "OK"，验证单片机→蓝牙方向 */
    {
      dbg_tick = HAL_GetTick();
      if (HAL_UART_Transmit(&huart1, (uint8_t *)"OK\r\n", 4, 10) != HAL_OK)
      {
        huart1.gState = HAL_UART_STATE_READY;
      }
    }
#endif

    /* ---- 1. 解析指令 ---- */
    if (cmd >= '1' && cmd <= '9')     /* 1~9 = 调速，不改变行驶方向 */
    {
      g_speed = (uint16_t)(cmd - '0') * SPEED_STEP;
      /* 保底：占空比太低电机维持不住转动。有了下面的起步 boost，
         静止时低档也能起步，但运行中仍需要这个下限兜住 */
      if (g_speed < SPEED_MIN) g_speed = SPEED_MIN;
    }
    else if (cmd == '0')              /* 0 = 停车（比 1 还慢没有意义，直接当停车键用） */
    {
      g_dir = 0;
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
#if UART_SILENT_TEST || UART_SCAN_MODE || UART_SELFTEST_MODE
    Car_Stop();      /* 诊断模式：不管收到什么都停车，防止线上噪声把车开走 */
#else
    /* ================= 起步 boost（kick start） =================
       直流电机的启动扭矩（要克服静摩擦）远大于维持扭矩（动摩擦）。
       实测（电池 5.72V，电机端约 4V）：
         · 静止直接给 3~7 档 → 只有堵转嗡响，8 档才起得来
         · 但转起来后从 8 档降到 2 档 → 仍能维持慢速转动
       对策：从"停"切到"走"（或反向）的瞬间，先给满占空比冲刺 KICK_MS 毫秒，
             帮转子突破静摩擦，随后立即回落到用户设定的档位。
       效果：低速档从静止也能起步，同时不牺牲正常调速手感。
       ============================================================= */
    static int8_t   last_dir  = 0;      /* 上一轮的方向，用来检测"起步"这个时刻 */
    static uint8_t  kicking   = 0;      /* 是否正处于冲刺阶段 */
    static uint32_t kick_tick = 0;

    if ((g_dir != 0) && (g_dir != last_dir))   /* 起步 或 反向 → 触发冲刺 */
    {
      kicking   = 1;
      kick_tick = HAL_GetTick();
    }

    if (g_dir == 0)                            /* 停车：立刻取消冲刺 */
    {
      kicking = 0;
    }
    else if (kicking && ((HAL_GetTick() - kick_tick) >= KICK_MS))
    {
      kicking = 0;                             /* 冲刺结束，回落到目标速度 */
    }

    last_dir = g_dir;

    uint16_t out_speed = kicking ? PWM_MAX : g_speed;

    switch (g_dir)
    {
      case 1:  Car_Run(1,  out_speed);    break;
      case -1: Car_Run(-1, out_speed);    break;
      case 2:  Car_TurnLeft(out_speed);   break;
      case 3:  Car_TurnRight(out_speed);  break;
      default: Car_Stop();                break;
    }
#endif
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
  huart1.Init.BaudRate = 9600;
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
  * @brief  判断一个字节是不是合法指令（只认 F/B/L/R/S 和 0~9）
  *         作用：串口线上如果窜进乱码噪声，绝不能让它驱动电机
  */
static uint8_t Cmd_IsValid(uint8_t c)
{
  if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') return 1;
  if (c >= '0' && c <= '9') return 1;
  return 0;
}

/**
  * @brief  串口接收完成回调（HAL 库在中断里调用它）
  *         收满 1 字节 → 存指令、记时间 → 立刻重新武装接收
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    rx_any_cnt++;        /* 仪表：收到任何字节都记一笔，主循环靠它点灯 */

#if UART_SELFTEST_MODE
    if (rx_byte == 0x55) { selftest_rx_cnt++; }   /* 自检：数一数收回来的测试字节 */
#endif

#if DEBUG_UART_ECHO && !UART_SELFTEST_MODE
    /* 调试回显：只回显"合法指令"。
       手机发 F → 手机收到 F，说明链路双向通；
       线上窜进来的噪声字节一律不回显，手机屏幕保持干净 */
    if (Cmd_IsValid(rx_byte))
    {
      echo_char    = rx_byte;
      echo_pending = 1;
    }
#endif

    /* 关键：只有合法指令才更新指令和时间戳。
       否则线上的噪声会不断刷新 last_cmd_tick，把 800ms 断连保护彻底废掉 */
    if (Cmd_IsValid(rx_byte))
    {
      car_cmd = rx_byte;                     /* 存下收到的字符 */
      last_cmd_tick = HAL_GetTick();         /* 记录时间，用于断连保护 */
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* 关键：重新开一次接收 */
  }
}

/**
  * @brief  串口错误回调（溢出/帧错误/噪声都走这里）
  *         作用：线上噪声太猛时串口会溢出，HAL 默认不再自动开接收 → 从此永远收不到数据。
  *         手动清标志并重新武装，让串口能自己爬起来。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);        /* 读 SR + DR，清掉溢出标志 */
    huart->RxState = HAL_UART_STATE_READY;  /* 解开可能卡在 BUSY_RX 的状态 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

#if UART_SCAN_MODE
/**
  * @brief  波特率扫描：按编号设置 USART1 的波特率
  * @param  idx 0~5，对应 9600 / 19200 / 38400 / 57600 / 115200 / 4800
  * @note   直接写 BRR 寄存器，不做 HAL 重新初始化 —— 因为 HAL_UART_Init 在串口
  *         正在发送时会返回 BUSY，而且会重配 GPIO，这样最干净也最快。
  *         USART1 挂在 APB2（72MHz），默认 16 倍过采样 → BRR = 72MHz / 波特率
  */
static void UART_ScanApply(uint8_t idx)
{
  static const uint32_t bauds[6] = {9600, 19200, 38400, 57600, 115200, 4800};

  huart1.Init.BaudRate = bauds[idx];
  huart1.Instance->BRR = (uint16_t)((72000000UL + bauds[idx] / 2U) / bauds[idx]);
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
