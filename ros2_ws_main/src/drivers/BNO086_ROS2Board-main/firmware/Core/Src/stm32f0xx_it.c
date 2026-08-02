#include "main.h"

#ifdef HOST_LINK_USB
extern PCD_HandleTypeDef hpcd_USB_FS;
#endif

void NMI_Handler(void)
{
  for (;;) {
  }
}

void HardFault_Handler(void)
{
  board_error();
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

#ifdef HOST_LINK_USB
void USB_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}
#endif
