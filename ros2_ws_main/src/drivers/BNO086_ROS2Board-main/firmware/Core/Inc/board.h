/**
 * board.h - pin map for BNO086_ROS2Board rev.1
 *
 * Every assignment below was extracted from the KiCad PCB netlist
 * (PCB/BNO086_ROS2Board/BNO086_ROS2Board.kicad_pcb), not from the .ioc file,
 * which is out of date - see docs/HARDWARE.md.
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f0xx_hal.h"

/* --- BNO086 (U4) ------------------------------------------------------- */
/* SPI1: PA5/PA6/PA7. The BNO08x SPI slave is mode 3, MSB first, <= 3 MHz. */
#define IMU_SPI                 SPI1
#define IMU_SCK_PIN             GPIO_PIN_5      /* PA5  */
#define IMU_MISO_PIN            GPIO_PIN_6      /* PA6  */
#define IMU_MOSI_PIN            GPIO_PIN_7      /* PA7  */
#define IMU_SPI_PORT            GPIOA
#define IMU_SPI_AF              GPIO_AF0_SPI1

#define IMU_RST_PORT            GPIOA           /* PA4  -> U4 pin 11 NRST   */
#define IMU_RST_PIN             GPIO_PIN_4      /*         R5 10k pull-up   */

#define IMU_CS_PORT             GPIOB           /* PB10 -> U4 pin 18 CSN    */
#define IMU_CS_PIN              GPIO_PIN_10

#define IMU_INT_PORT            GPIOB           /* PB2  <- U4 pin 14 H_INTN */
#define IMU_INT_PIN             GPIO_PIN_2      /*         active low       */
#define IMU_INT_IRQn            EXTI2_3_IRQn

/* PS0 doubles as H_WAKEN once the part has booted in SPI mode.
 * R6 pulls it up and JP1 can strap it to GND; JP1 must stay OPEN so that
 * firmware can drive the level during reset. */
#define IMU_WAKE_PORT           GPIOB           /* PB0  -> U4 pin 6 PS0/WAKE */
#define IMU_WAKE_PIN            GPIO_PIN_0

#define IMU_BOOT_PORT           GPIOB           /* PB1  -> U4 pin 4 BOOTN   */
#define IMU_BOOT_PIN            GPIO_PIN_1      /*         R8 10k pull-up   */

/* PS1 (U4 pin 5) is strapped high by R7; JP2 can pull it to GND.
 * PS1=1, PS0=0 at the rising edge of NRST selects SPI. Leave JP2 OPEN. */

/* --- Indicators (active high, cathode through 5.1k to GND) -------------- */
#define LED1_PORT               GPIOB           /* PB12 */
#define LED1_PIN                GPIO_PIN_12
#define LED2_PORT               GPIOB           /* PB13 */
#define LED2_PIN                GPIO_PIN_13

/* --- Host link --------------------------------------------------------- */
#define HOST_UART               USART1          /* PA9 TX / PA10 RX -> J4   */
#define HOST_UART_TX_PIN        GPIO_PIN_9
#define HOST_UART_RX_PIN        GPIO_PIN_10
#define HOST_UART_PORT          GPIOA
#define HOST_UART_AF            GPIO_AF1_USART1

/* --- CAN (U5 transceiver, J5) ------------------------------------------ */
#define CAN_RX_PIN              GPIO_PIN_8      /* PB8 */
#define CAN_TX_PIN              GPIO_PIN_9      /* PB9 */
#define CAN_PORT                GPIOB
#define CAN_AF                  GPIO_AF4_CAN

/* --- helpers ----------------------------------------------------------- */
#define LED1_ON()   HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET)
#define LED1_OFF()  HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET)
#define LED1_TOG()  HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN)
#define LED2_ON()   HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET)
#define LED2_OFF()  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET)
#define LED2_TOG()  HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN)

void board_init(void);
void board_error(void);

extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;

/** Microsecond counter derived from SysTick; wraps every ~71 minutes. */
uint32_t board_micros(void);
void     board_delay_us(uint32_t us);

#endif /* BOARD_H */
