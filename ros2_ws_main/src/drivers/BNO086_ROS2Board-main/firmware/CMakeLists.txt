cmake_minimum_required(VERSION 3.22)

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake"
      CACHE FILEPATH "ARM cross toolchain")
endif()

project(bno086_ros2board C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------- options ----
# Exactly one host transport is compiled in; they are mutually exclusive so
# that a stalled USB endpoint can never starve the UART and vice versa.
option(HOST_LINK_USB  "Stream IMU data over the USB-C CDC port"      ON)
option(HOST_LINK_UART "Stream IMU data over USART1 (J4) instead"     OFF)
# The board carries a CAN transceiver on J5, but no CAN transport is
# implemented yet - see docs/HARDWARE.md before designing one.

if(HOST_LINK_USB AND HOST_LINK_UART)
  message(FATAL_ERROR "Enable only one of HOST_LINK_USB / HOST_LINK_UART")
endif()
if(NOT HOST_LINK_USB AND NOT HOST_LINK_UART)
  message(FATAL_ERROR "Enable one of HOST_LINK_USB / HOST_LINK_UART")
endif()
set(UART_BAUD 921600 CACHE STRING "USART1 baud rate for the host link")

# ------------------------------------------------------------ dependencies ---
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(cmsis_core
  GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_core.git
  GIT_TAG        v5.9.0_20250520
  GIT_SHALLOW    TRUE)
FetchContent_Declare(cmsis_device_f0
  GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_f0.git
  GIT_TAG        v2.3.7
  GIT_SHALLOW    TRUE)
FetchContent_Declare(stm32f0xx_hal_driver
  GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f0xx_hal_driver.git
  GIT_TAG        v1.7.8
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(cmsis_core cmsis_device_f0 stm32f0xx_hal_driver)

if(HOST_LINK_USB)
  FetchContent_Declare(stm32_mw_usb_device
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32_mw_usb_device.git
    GIT_TAG        v2.9.0
    GIT_SHALLOW    TRUE)
  FetchContent_MakeAvailable(stm32_mw_usb_device)
endif()

# ----------------------------------------------------------------- sources ---
# Only the HAL modules we actually need - the part has 32 KiB of flash.
set(HAL_MODULES
  stm32f0xx_hal
  stm32f0xx_hal_cortex
  stm32f0xx_hal_rcc
  stm32f0xx_hal_rcc_ex
  stm32f0xx_hal_gpio
  stm32f0xx_hal_spi
  stm32f0xx_hal_uart
  stm32f0xx_hal_uart_ex
  stm32f0xx_hal_dma
  stm32f0xx_hal_pwr
  stm32f0xx_hal_pwr_ex
  stm32f0xx_hal_flash
  stm32f0xx_hal_flash_ex)

if(HOST_LINK_USB)
  list(APPEND HAL_MODULES stm32f0xx_hal_pcd stm32f0xx_hal_pcd_ex stm32f0xx_ll_usb)
endif()

set(HAL_SOURCES "")
foreach(m ${HAL_MODULES})
  list(APPEND HAL_SOURCES "${stm32f0xx_hal_driver_SOURCE_DIR}/Src/${m}.c")
endforeach()

set(APP_SOURCES
  Core/Src/main.c
  Core/Src/board.c
  Core/Src/stm32f0xx_it.c
  Core/Src/stm32f0xx_hal_msp.c
  App/bno08x.c
  App/host_link.c
  "${cmsis_device_f0_SOURCE_DIR}/Source/Templates/system_stm32f0xx.c"
  "${cmsis_device_f0_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f042x6.s")

if(HOST_LINK_USB)
  list(APPEND APP_SOURCES
    USB/usb_device.c
    USB/usbd_conf.c
    USB/usbd_desc.c
    USB/usbd_cdc_if.c
    "${stm32_mw_usb_device_SOURCE_DIR}/Core/Src/usbd_core.c"
    "${stm32_mw_usb_device_SOURCE_DIR}/Core/Src/usbd_ctlreq.c"
    "${stm32_mw_usb_device_SOURCE_DIR}/Core/Src/usbd_ioreq.c"
    "${stm32_mw_usb_device_SOURCE_DIR}/Class/CDC/Src/usbd_cdc.c")
endif()

add_executable(${PROJECT_NAME}.elf ${APP_SOURCES} ${HAL_SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
  Core/Inc
  App
  "${cmsis_core_SOURCE_DIR}/Include"
  "${cmsis_device_f0_SOURCE_DIR}/Include"
  "${stm32f0xx_hal_driver_SOURCE_DIR}/Inc")

if(HOST_LINK_USB)
  target_include_directories(${PROJECT_NAME}.elf PRIVATE
    USB
    "${stm32_mw_usb_device_SOURCE_DIR}/Core/Inc"
    "${stm32_mw_usb_device_SOURCE_DIR}/Class/CDC/Inc")
endif()

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
  STM32F042x6
  USE_HAL_DRIVER
  HSE_VALUE=8000000U
  UART_BAUD=${UART_BAUD}
  $<$<BOOL:${HOST_LINK_USB}>:HOST_LINK_USB=1>
  $<$<BOOL:${HOST_LINK_UART}>:HOST_LINK_UART=1>)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
  -mcpu=cortex-m0 -mthumb
  -Os -ffunction-sections -fdata-sections -fno-common
  -Wall -Wextra -Wno-unused-parameter
  $<$<COMPILE_LANGUAGE:C>:-std=gnu11>)

target_link_options(${PROJECT_NAME}.elf PRIVATE
  -mcpu=cortex-m0 -mthumb
  -T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F042C6UX_FLASH.ld
  --specs=nano.specs --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--print-memory-usage)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
  COMMENT "Generating .bin/.hex and reporting size")
