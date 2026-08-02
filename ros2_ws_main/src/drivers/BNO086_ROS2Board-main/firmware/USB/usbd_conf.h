#ifndef USBD_CONF_H
#define USBD_CONF_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f0xx_hal.h"

#define USBD_MAX_NUM_INTERFACES      2U
#define USBD_MAX_NUM_CONFIGURATION   1U
#define USBD_MAX_STR_DESC_SIZ        64U
#define USBD_SELF_POWERED            0U
#define USBD_DEBUG_LEVEL             0U

/* Index of the only USB peripheral on this part. */
#define DEVICE_FS                    0

/* CDC_DATA_FS/HS_MAX_PACKET_SIZE come from usbd_cdc.h - it defines them
 * unconditionally, so setting them here would only produce redefinition
 * warnings. */

/* No heap on this part - the class instance is served from a static pool. */
#define USBD_malloc  (void *)USBD_static_malloc
#define USBD_free    USBD_static_free
#define USBD_memset  memset
#define USBD_memcpy  memcpy
#define USBD_Delay   HAL_Delay

#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...) do { printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_UsrLog(...)
#endif
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

void *USBD_static_malloc(uint32_t size);
void  USBD_static_free(void *p);

#endif /* USBD_CONF_H */
