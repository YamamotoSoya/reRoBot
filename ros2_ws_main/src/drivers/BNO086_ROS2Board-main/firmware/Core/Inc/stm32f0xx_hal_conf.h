/**
 * Trimmed HAL configuration for STM32F042C6.
 * Only the modules the application links are enabled - the part has 32 KiB
 * of flash and the full HAL does not fit alongside the USB device stack.
 */
#ifndef STM32F0xx_HAL_CONF_H
#define STM32F0xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------- Module selection ------ */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#ifdef HOST_LINK_USB
#define HAL_PCD_MODULE_ENABLED
#endif

/* ------------------------------------------------- Oscillator values ----- */
#if !defined(HSE_VALUE)
#define HSE_VALUE             8000000U   /* Y2, 8 MHz crystal */
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT   100U
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE             8000000U
#endif
#if !defined(HSI_STARTUP_TIMEOUT)
#define HSI_STARTUP_TIMEOUT   5000U
#endif
#if !defined(HSI14_VALUE)
#define HSI14_VALUE           14000000U
#endif
#if !defined(HSI48_VALUE)
#define HSI48_VALUE           48000000U
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE             40000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE             32768U
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT   5000U
#endif

/* ------------------------------------------------- System configuration -- */
#define VDD_VALUE                     3300U
#define TICK_INT_PRIORITY             0U
#define USE_RTOS                      0U
#define PREFETCH_ENABLE               1U
#define INSTRUCTION_CACHE_ENABLE      0U
#define DATA_CACHE_ENABLE             0U
#define USE_SPI_CRC                   0U

#define USE_HAL_ADC_REGISTER_CALLBACKS   0U
#define USE_HAL_CAN_REGISTER_CALLBACKS   0U
#define USE_HAL_PCD_REGISTER_CALLBACKS   0U
#define USE_HAL_SPI_REGISTER_CALLBACKS   0U
#define USE_HAL_UART_REGISTER_CALLBACKS  0U

/* ------------------------------------------------- Assert --------------- */
/* #define USE_FULL_ASSERT   1U */
#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

/* ------------------------------------------------- Includes ------------- */
#include "stm32f0xx_hal_rcc.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_cortex.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_pwr.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_uart.h"

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f0xx_hal_pcd.h"
#endif
#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f0xx_hal_can.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32F0xx_HAL_CONF_H */
