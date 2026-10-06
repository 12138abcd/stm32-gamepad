/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button_front.h"
#include "joystick.h"
#include "report.h"
#include "usbd_customhid.h"   /* CUSTOM_HID_EPIN_ADDR（调试打点用） */
#include <string.h>           /* memcmp / memcpy（调试打点用）      */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===================== 延迟测量：直方图的桶设置 =====================
   延迟主要在 2~4ms 这个量级（消抖 2 拍 + 1ms 节拍对齐 + 协议栈 + USB 帧等待），
   所以桶宽取 10µs、量程 0~5000µs 刚好够：
     500 个桶 × 2 字节 = 1000 字节，F103 的 20KB SRAM 放得下。 */
#define PROBE_BUCKET_US   10u    /* 每个桶 10 微秒 */
#define PROBE_BUCKETS     500u   /* 0 ~ 5000 微秒 */

/* 一次测量完成后的"封锁期"：期间不再武装新的测量。
   机械按键按下时有 5~10ms 的抖动，会让原始电平反复跳变；不封锁的话
   一次按键会被当成好几次测量。20ms 比最长抖动还长，而人最快也就
   每秒按十几次（间隔 60ms+），所以不会漏掉真实操作。 */
#define PROBE_LOCKOUT_CYCLES  (SystemCoreClock / 50u)   /* 20ms 对应的周期数 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
uint16_t adc_buf[2];      /* [0] = PA0 = VRx, [1] = PA1 = VRy */

/* ---- 调试打点用的变量（必须加 volatile，否则会被优化掉、调试器里永远是 0）----
   这一组计数器专门用来定位"打点链路断在哪一节"，测完可以整段删掉。 */
static uint8_t    probe_prev[9];           /* 上一拍的报告内容，判断输入有没有变化 */
volatile uint32_t probe_reg_result  = 0;   /* 注册返回值：0 = HAL_OK；非 0 = 注册失败 */
volatile uint32_t probe_fall        = 0;   /* PB10 被拉低的次数（输入变化）*/
volatile uint32_t probe_rise        = 0;   /* PB10 被拉高的次数（报告已上总线）*/
volatile uint32_t probe_datain_any  = 0;   /* DataIn 回调被调用的总次数（任何端点）*/
volatile uint32_t probe_datain_ep1  = 0;   /* 其中 epnum == 1（HID 的 IN 端点）的次数 */

/* ===================== 延迟测量：直方图 + 分位数 =====================
   原理：用 Cortex-M3 的 DWT->CYCCNT（72MHz 自由运行计数器，分辨率 13.9ns）
   在固件内部测"输入变化 → 报告发上 USB 总线"的耗时，按 10µs 分桶累加。
   优点：不需要任何外部仪器，可以自动跑几千次，直接在 Watch 里读分位数。 */
volatile uint16_t probe_hist[PROBE_BUCKETS];      /* 各延迟区间的命中次数（下标 = 延迟/10µs）*/
volatile uint16_t probe_hist_over = 0;            /* 超过量程（>5000µs）的次数 */
volatile uint32_t probe_count     = 0;            /* 有效样本数 */
volatile uint32_t probe_max_us    = 0;            /* 见过的最长延迟（µs）*/
volatile uint32_t probe_min_us    = 0xFFFFFFFFu;  /* 最短延迟（还没采到样本时是 0xFFFFFFFF）*/
volatile uint32_t probe_p50       = 0;            /* 分位数（每采到新样本后重算一次）*/
volatile uint32_t probe_p95       = 0;
volatile uint32_t probe_p99       = 0;
static   uint32_t probe_send_done    = 0;         /* 已完成的 HID 报告发送次数 */
static   uint32_t probe_pending_done = 0;         /* 报告变化时"当时已完成"的发送数 */
static   uint32_t probe_t0           = 0;         /* 【起点】原始电平第一次变化的时刻 */
static   uint32_t probe_lockout      = 0;         /* 上次测量完成的时刻（封锁期用）*/
static   uint8_t  probe_armed        = 0;         /* 是否已武装（有一次测量在进行）*/
static   uint8_t  probe_changed      = 0;         /* 本次测量中报告内容是否已变化 */
static   uint8_t  probe_raw_prev     = 0;         /* 上一拍的 8 个按键原始电平 */

/* 在线清零开关：在 Keil 的 Watch 窗口里把它改成 1，下一拍就会清空所有统计。
   这样两轮参数实验不用重新烧写，测试条件完全一致（数据才可比）。 */
volatile uint8_t  probe_clear        = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* ===== 调试打点：USB 延迟测量（定义都在 USER CODE 4）=====
   打点脚 PB10 —— 选它的理由：不占 ADC 通道、不占 SPI1（PA4~PA7 留给无线）、
   不占 USART1（PA9/PA10 留给串口）、不是 JTAG 脚（PA15/PB3/PB4）、不是板载 LED。
   测法：逻辑分析仪 CH1 接按键引脚、CH2 接 PB10 ——
     CH2 下降沿 = 固件"发现"输入变化的时刻（1ms 轮询的那一拍）
     CH2 上升沿 = 报告真正发上 USB 总线的时刻（DataIn 回调）
   于是 CH1 下降沿 → CH2 上升沿 = 设备侧 input latency（含轮询量化 + 消抖 + 节拍 + 协议栈）
        CH1 下降沿 → CH2 下降沿 = 1ms 轮询带来的量化延迟（单独量出来） */
void Probe_MarkInputChange(void);   /* 输入有变化时把 PB10 拉低 */
void Probe_DataInHook(uint8_t epnum);   /* 由 usbd_conf.c 里的宏在 DataIn 时调用 */
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
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 2);

	/* ===================== 调试打点：初始化（PB10）=====================
	   全部写在 USER CODE 区、不经过 CubeMX，所以重新生成代码不会丢。
	   初始电平【高】—— 打点逻辑是"变化时拉低、发出时拉高"，
	   所以一个完整的测量是 CH2 上的一个负脉冲：下降沿=输入变化，上升沿=已发出。 */
	{
	    GPIO_InitTypeDef probe = {0};
	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    probe.Pin   = GPIO_PIN_10;
	    probe.Mode  = GPIO_MODE_OUTPUT_PP;
	    probe.Pull  = GPIO_NOPULL;
	    probe.Speed = GPIO_SPEED_FREQ_HIGH;
	    HAL_GPIO_Init(GPIOB, &probe);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   /* 空闲 = 高 */
	}

	/* 打点挂钩不在这里做 —— 改成在 usbd_conf.c 的 USER CODE 区用一个宏包装
	   USBD_LL_DataInStage() 的调用（见那个文件里的说明）。
	   好处：不依赖 HAL 的回调注册机制，也不会因为生成代码被重写而失效。 */

	/* ===================== 延迟测量：打开 DWT 周期计数器 =====================
	   Cortex-M3 的 DWT->CYCCNT 跟着内核时钟自由运行（72MHz → 13.9ns 分辨率），
	   用来在固件内部精确测量"输入变化 → 报告发上 USB 总线"的耗时。
	   三个步骤缺一不可：TRCENA 使能跟踪 → 计数器清零 → CYCCNTENA 使能计数。 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t last = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//非阻塞软定时器
    if (HAL_GetTick() - last >= 1)
    {
        last = HAL_GetTick();
				scan_buttons();   /* read GPIOs and debounce                 */
				joystick_update(); /* read ADC, fill X/Y axes                 */
				Probe_MarkInputChange(); /* 调试打点：输入有变化就拉低 PB10   */
				send_report();    /* skip this tick if the previous transfer */
									
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : up_Pin down_Pin left_Pin right_Pin
                           A_Pin B_Pin X_Pin Y_Pin
                           SW1_Pin */
  GPIO_InitStruct.Pin = up_Pin|down_Pin|left_Pin|right_Pin
                          |A_Pin|B_Pin|X_Pin|Y_Pin
                          |SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* ======================================================================
   ==========  调试打点：USB 延迟测量（临时测量装置，不是产品功能）  ==========
   ======================================================================
   目的：用逻辑分析仪测"输入电平变化 → 报告真正发上 USB 总线"的延迟。
   接法：分析仪 CH1 → 按键引脚（如 PB5 = A 键）
        分析仪 CH2 → PB10（打点脚）
        分析仪 GND → 板子 GND
   读法：CH2 是一个负脉冲 —— 下降沿 = 固件发现输入变化；上升沿 = 报告已发上总线。
        · CH1 下降沿 → CH2 上升沿 = 【设备侧 input latency】
          （含 1ms 轮询量化 + 消抖 2 拍 + 节拍对齐 + 协议栈 + USB 帧等待）
        · CH1 下降沿 → CH2 下降沿 = 轮询量化延迟（单独量出来，理论上 0~1ms）
   注意：这段代码在 USB 中断回调里执行，只做一次 GPIO 翻转（几十 ns），不影响实时性。 */

/* （probe_prev 和那几个调试计数器都定义在文件顶部的 USER CODE PV 区，
    因为 USER CODE 2 里要用到，必须先声明） */

/* 每 1ms 在主循环里调一次（scan/joystick 之后、send_report 之前）。干两件事：
   ① 报告内容变化 → 把打点脚拉低（给逻辑分析仪一个可见的边沿）
   ② 按键【原始电平】变化 → 武装一次延迟测量，记下起点时间

   ② 为什么读原始电平、而不是等报告变化：
   消抖要 2 拍（2ms），是整个延迟里最大的一块。如果从"报告变化"才开始计时，
   就整整漏掉 2ms，测出来的数没有意义。所以起点必须是"固件第一次看到电平变化"，
   终点是"这一帧报告真正发上 USB 总线"——中间那 2ms 消抖才算得进去。 */
void Probe_MarkInputChange(void)
{
    uint8_t raw = 0u;

    /* ---- 在线清零：Watch 里把 probe_clear 改成 1 即可（下一拍生效）----
       只清统计量，不动 probe_send_done / probe_pending_done —— 那两个是
       "帧序号"，清零反而会让完成判据错乱。 */
    if (probe_clear)
    {
        uint16_t k;
        probe_clear = 0u;
        for (k = 0u; k < PROBE_BUCKETS; k++) { probe_hist[k] = 0u; }
        probe_hist_over  = 0u;
        probe_count      = 0u;
        probe_max_us     = 0u;
        probe_min_us     = 0xFFFFFFFFu;
        probe_p50 = 0u; probe_p95 = 0u; probe_p99 = 0u;
        probe_fall       = 0u;
        probe_rise       = 0u;
        probe_datain_any = 0u;
        probe_datain_ep1 = 0u;
        probe_armed      = 0u;
        probe_changed    = 0u;
        probe_lockout    = DWT->CYCCNT;   /* 清完先封锁，避开清空瞬间的残留 */
    }

    /* ---- 读 8 个按键引脚的【原始电平】（未经消抖）---- */
    if (HAL_GPIO_ReadPin(A_GPIO_Port,     A_Pin)     == GPIO_PIN_RESET) { raw |= 0x01u; }
    if (HAL_GPIO_ReadPin(B_GPIO_Port,     B_Pin)     == GPIO_PIN_RESET) { raw |= 0x02u; }
    if (HAL_GPIO_ReadPin(X_GPIO_Port,     X_Pin)     == GPIO_PIN_RESET) { raw |= 0x04u; }
    if (HAL_GPIO_ReadPin(Y_GPIO_Port,     Y_Pin)     == GPIO_PIN_RESET) { raw |= 0x08u; }
    if (HAL_GPIO_ReadPin(up_GPIO_Port,    up_Pin)    == GPIO_PIN_RESET) { raw |= 0x10u; }
    if (HAL_GPIO_ReadPin(down_GPIO_Port,  down_Pin)  == GPIO_PIN_RESET) { raw |= 0x20u; }
    if (HAL_GPIO_ReadPin(left_GPIO_Port,  left_Pin)  == GPIO_PIN_RESET) { raw |= 0x40u; }
    if (HAL_GPIO_ReadPin(right_GPIO_Port, right_Pin) == GPIO_PIN_RESET) { raw |= 0x80u; }

    /* ---- ① 报告内容变了 → 打点脚拉低 ---- */
    if (memcmp(probe_prev, gamepad_report, 9) != 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);   /* 下降沿 = 变化已确认 */
        probe_fall++;                                            /* 调试计数 */
        memcpy(probe_prev, gamepad_report, 9);

        /* 正在测量的话，记下"这一刻已完成多少帧"——变化之后发出去的第一帧就是终点 */
        if (probe_armed)
        {
            probe_pending_done = probe_send_done;
            probe_changed      = 1u;
        }
    }

    /* ---- ② 原始电平变了 → 武装一次测量（已武装、或在封锁期内则忽略）---- */
    if (raw != probe_raw_prev)
    {
        probe_raw_prev = raw;

        if ((probe_armed == 0u) && ((DWT->CYCCNT - probe_lockout) > PROBE_LOCKOUT_CYCLES))
        {
            probe_armed   = 1u;
            probe_changed = 0u;
            probe_t0      = DWT->CYCCNT;   /* ★起点：固件第一次看到电平变化的那一拍 */
        }
    }
}

/* 从直方图重算 P50 / P95 / P99。
   每采到一个新样本后调一次（500 次循环，72MHz 下约 70µs），
   而且只在输入变化时才触发，所以对实时性没有影响。

   两个稳健性处理（都为了不让调试器读到假值）：
   ① 样本少于 20 个时干脆不更新 —— 没有统计意义，而且早期样本少时
      算出来的分位数很容易出现"比 min 还小"这种不可能的值；
   ② 三个值先算到局部变量，最后一次性赋值 —— 避免调试器恰好在
      "清完 0、还没写新值"的瞬间读到中间状态。 */
static void Probe_UpdateStats(void)
{
    uint32_t total = 0u, cum = 0u, i;
    uint32_t t50, t95, t99;
    uint32_t r50 = 0u, r95 = 0u, r99 = 0u;

    for (i = 0u; i < PROBE_BUCKETS; i++) { total += probe_hist[i]; }
    if (total < 20u) { return; }

    t50 = (total * 50u) / 100u;
    t95 = (total * 95u) / 100u;
    t99 = (total * 99u) / 100u;

    for (i = 0u; i < PROBE_BUCKETS; i++)
    {
        cum += probe_hist[i];
        if ((r50 == 0u) && (cum >= t50)) { r50 = (i + 1u) * PROBE_BUCKET_US; }
        if ((r95 == 0u) && (cum >= t95)) { r95 = (i + 1u) * PROBE_BUCKET_US; }
        if ((r99 == 0u) && (cum >= t99)) { r99 = (i + 1u) * PROBE_BUCKET_US; }
    }

    probe_p50 = r50;
    probe_p95 = r95;
    probe_p99 = r99;
}

/* USB DataIn 打点钩子。
   它由 usbd_conf.c 里的宏在【原回调调用 USBD_LL_DataInStage() 的那一刻】调用，
   所以这里不需要自己再调 USBD_LL_DataInStage —— 原始调用照常执行。 */
void Probe_DataInHook(uint8_t epnum)
{
    probe_datain_any++;                        /* 钩子有没有被调用 */

    /* epnum 是【不含方向位】的端点号：CUSTOM_HID_EPIN_ADDR = 0x81 → epnum = 1 */
    if (epnum == (CUSTOM_HID_EPIN_ADDR & 0x7FU))
    {
        probe_datain_ep1++;                    /* 端点号判断有没有命中 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);     /* 上升沿 = 已发上总线 */
        probe_rise++;                          /* 调试计数 */

        probe_send_done++;                     /* 又成功发出去了一帧报告 */

        /* 测量完成要同时满足三个条件：
             armed        —— 有一次测量正在进行
             changed      —— 报告内容已经反映出这次变化（也就是消抖已经完成）
             send_done >  —— 而且这一帧正是"变化之后发出去的第一帧"
           这样才能测到【原始电平变化 → 消抖 2 拍 → 上 USB 总线】的完整时间，
           既不会漏掉消抖，也不会被前一次还在途中的发送干扰。 */
        if (probe_armed && probe_changed && (probe_send_done > probe_pending_done))
        {
            uint32_t cycles = DWT->CYCCNT - probe_t0;   /* 无符号减法：计数器溢出也正确 */
            uint32_t us     = cycles / (SystemCoreClock / 1000000u);

            probe_armed   = 0u;
            probe_changed = 0u;
            probe_lockout = DWT->CYCCNT;                /* 进入封锁期，压掉机械抖动 */
            probe_count++;

            if (us < (PROBE_BUCKETS * PROBE_BUCKET_US)) { probe_hist[us / PROBE_BUCKET_US]++; }
            else                                        { probe_hist_over++; }

            if (us > probe_max_us) { probe_max_us = us; }
            if (us < probe_min_us) { probe_min_us = us; }

            Probe_UpdateStats();
        }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
