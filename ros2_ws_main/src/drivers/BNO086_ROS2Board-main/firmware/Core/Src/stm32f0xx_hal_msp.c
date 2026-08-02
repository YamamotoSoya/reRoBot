#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef g = {0};

  if (hspi->Instance != IMU_SPI) {
    return;
  }

  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  g.Pin       = IMU_SCK_PIN | IMU_MISO_PIN | IMU_MOSI_PIN;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = IMU_SPI_AF;
  HAL_GPIO_Init(IMU_SPI_PORT, &g);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == IMU_SPI) {
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(IMU_SPI_PORT, IMU_SCK_PIN | IMU_MISO_PIN | IMU_MOSI_PIN);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef g = {0};

  if (huart->Instance != HOST_UART) {
    return;
  }

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  g.Pin       = HOST_UART_TX_PIN | HOST_UART_RX_PIN;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = HOST_UART_AF;
  HAL_GPIO_Init(HOST_UART_PORT, &g);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == HOST_UART) {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(HOST_UART_PORT, HOST_UART_TX_PIN | HOST_UART_RX_PIN);
  }
}

#ifdef HOST_LINK_USB
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Instance != USB) {
    return;
  }
  /* PA11/PA12 are dedicated USB pins on the 48-pin package - no GPIO or
   * SYSCFG remap configuration is required. The F042 also has the D+
   * pull-up built in, which is why the PCB has no external resistor. */
  __HAL_RCC_USB_CLK_ENABLE();
  HAL_NVIC_SetPriority(USB_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USB_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Instance == USB) {
    __HAL_RCC_USB_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USB_IRQn);
  }
}
#endif /* HOST_LINK_USB */
