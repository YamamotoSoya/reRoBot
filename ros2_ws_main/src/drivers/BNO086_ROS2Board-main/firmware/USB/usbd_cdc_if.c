/**
 * usbd_cdc_if.c - CDC interface hooks.
 *
 * Received bytes are handed straight to the host_link command decoder; the
 * line coding requests are accepted and ignored because the link runs over
 * USB where the baud rate is meaningless.
 */
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "host_link.h"

static uint8_t s_rx_buf[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t s_tx_buf[CDC_DATA_FS_MAX_PACKET_SIZE];

/* Accepted but unused; a terminal that sets a baud rate must not get a STALL. */
static uint8_t s_line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, s_tx_buf, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, s_rx_buf);
  return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
  return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd) {
    case CDC_SET_LINE_CODING:
      if (length >= sizeof(s_line_coding)) {
        memcpy(s_line_coding, pbuf, sizeof(s_line_coding));
      }
      break;
    case CDC_GET_LINE_CODING:
      if (length >= sizeof(s_line_coding)) {
        memcpy(pbuf, s_line_coding, sizeof(s_line_coding));
      }
      break;
    default:
      break;
  }
  return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len)
{
  host_link_rx_bytes(pbuf, *Len);

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &pbuf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum)
{
  (void)pbuf;
  (void)Len;
  (void)epnum;
  return USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hcdc == NULL) {
    return USBD_BUSY;
  }
  if (hcdc->TxState != 0U) {
    return USBD_BUSY;
  }
  if (Len > sizeof(s_tx_buf)) {
    Len = sizeof(s_tx_buf);
  }

  /* Copy out: the caller's buffer is a ring that keeps being written. */
  memcpy(s_tx_buf, Buf, Len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, s_tx_buf, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
