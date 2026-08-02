#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/**
 * Queue Len bytes on the CDC IN endpoint.
 * @return USBD_OK on success, USBD_BUSY if the previous transfer is still in
 *         flight or the device is not configured.
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#endif /* USBD_CDC_IF_H */
