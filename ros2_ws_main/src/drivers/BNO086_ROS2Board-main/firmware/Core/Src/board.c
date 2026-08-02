#include "main.h"
#include "board.h"

SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart1;

/**
 * HSE (Y2, 8 MHz) -> PLL x6 -> 48 MHz SYSCLK.
 * 48 MHz is mandatory here: the USB peripheral has no fractional divider and
 * is clocked straight from the PLL output.
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef       osc = {0};
  RCC_ClkInitTypeDef       clk = {0};
  RCC_PeriphCLKInitTypeDef periph = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState       = RCC_HSE_ON;
  osc.PLL.PLLState   = RCC_PLL_ON;
  osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  osc.PLL.PREDIV     = RCC_PREDIV_DIV1;
  osc.PLL.PLLMUL     = RCC_PLL_MUL6;          /* 8 MHz * 6 = 48 MHz */
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    board_error();
  }

  clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
    board_error();
  }

  periph.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  periph.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
#ifdef HOST_LINK_USB
  periph.PeriphClockSelection |= RCC_PERIPHCLK_USB;
  periph.UsbClockSelection     = RCC_USBCLKSOURCE_PLL;
#endif
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
    board_error();
  }
}

static void gpio_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* Outputs driving the BNO086 straps/control lines.
   * Park them in the state required while NRST is asserted:
   *   BOOTN = 1 (run the internal firmware, not the bootloader)
   *   PS0   = 1, which together with the R7 pull-up on PS1 selects SPI.
   *           Per the BNO08X datasheet (1.16, section 1.2.4 / figure 1-20)
   *           BOTH strap pins must be high from before reset until the first
   *           H_INTN assertion. Driving PS0 low here selects UART-RVC and the
   *           part then never answers on SPI at all.
   *   CSN   = 1 (idle) */
  HAL_GPIO_WritePin(IMU_RST_PORT,  IMU_RST_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU_BOOT_PORT, IMU_BOOT_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_PORT,   IMU_CS_PIN,   GPIO_PIN_SET);

  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;

  g.Pin = IMU_RST_PIN;   HAL_GPIO_Init(IMU_RST_PORT,  &g);
  g.Pin = IMU_BOOT_PIN;  HAL_GPIO_Init(IMU_BOOT_PORT, &g);
  g.Pin = IMU_WAKE_PIN;  HAL_GPIO_Init(IMU_WAKE_PORT, &g);
  g.Pin = IMU_CS_PIN;    HAL_GPIO_Init(IMU_CS_PORT,   &g);

  /* H_INTN is push-pull from the BNO086 but floats while it is held in
   * reset, so bias it to the inactive level. */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Pin  = IMU_INT_PIN;
  HAL_GPIO_Init(IMU_INT_PORT, &g);

  /* Status LEDs, active high through a 5.1k series resistor to GND. */
  HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = LED1_PIN; HAL_GPIO_Init(LED1_PORT, &g);
  g.Pin   = LED2_PIN; HAL_GPIO_Init(LED2_PORT, &g);
}

static void spi1_init(void)
{
  hspi1.Instance               = IMU_SPI;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  /* BNO08x samples on the rising edge of a clock that idles high: mode 3. */
  hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  /* PCLK 48 MHz / 16 = 3 MHz, the BNO08x maximum. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7;
  hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    board_error();
  }
}

#ifdef HOST_LINK_UART
static void usart1_init(void)
{
  huart1.Instance            = HOST_UART;
  huart1.Init.BaudRate       = UART_BAUD;
  huart1.Init.WordLength     = UART_WORDLENGTH_8B;
  huart1.Init.StopBits       = UART_STOPBITS_1;
  huart1.Init.Parity         = UART_PARITY_NONE;
  huart1.Init.Mode           = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    board_error();
  }
}
#endif

void board_init(void)
{
  gpio_init();
  spi1_init();
#ifdef HOST_LINK_UART
  usart1_init();
#endif
}

uint32_t board_micros(void)
{
  uint32_t ms, val, load;

  /* Re-read the millisecond counter to make sure it did not tick between
   * the two reads. */
  do {
    ms  = HAL_GetTick();
    val = SysTick->VAL;
  } while (ms != HAL_GetTick());

  load = SysTick->LOAD + 1U;
  return (ms * 1000U) + (((load - val) * 1000U) / load);
}

void board_delay_us(uint32_t us)
{
  uint32_t start = board_micros();
  while ((board_micros() - start) < us) {
    /* busy wait */
  }
}

/** Fatal error: fast-blink both LEDs forever. */
void board_error(void)
{
  __disable_irq();
  for (;;) {
    LED1_TOG();
    LED2_TOG();
    for (volatile uint32_t i = 0; i < 200000U; i++) {
      /* crude delay - SysTick is not running here */
    }
  }
}
