/**
 * usbd_desc.c - device and string descriptors.
 *
 * VID 0x0483 / PID 0x5740 is ST's own CDC example pair. It works with the
 * in-box CDC-ACM drivers on Linux, macOS and Windows 10+; replace it if this
 * board is ever shipped as a product.
 */
#include "usbd_core.h"
#include "usbd_desc.h"

#define USBD_VID           0x0483
#define USBD_PID           0x5740
#define USBD_LANGID_STRING 0x0409  /* en-US */
#define USBD_MANUFACTURER  "fTomo-robot"
#define USBD_PRODUCT       "BNO086 ROS2 IMU Board"
#define USBD_CONFIGURATION "CDC Config"
#define USBD_INTERFACE     "CDC Interface"

/* 96-bit unique device ID */
#define DEVICE_ID1 (0x1FFFF7ACU)
#define DEVICE_ID2 (0x1FFFF7B0U)
#define DEVICE_ID3 (0x1FFFF7B4U)

static uint8_t *Get_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *Get_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *Get_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *Get_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *Get_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *Get_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *Get_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef CDC_Desc = {
  Get_DeviceDescriptor,
  Get_LangIDStrDescriptor,
  Get_ManufacturerStrDescriptor,
  Get_ProductStrDescriptor,
  Get_SerialStrDescriptor,
  Get_ConfigStrDescriptor,
  Get_InterfaceStrDescriptor,
};

__ALIGN_BEGIN static uint8_t s_device_desc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,                       /* bLength */
  USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
  0x00, 0x02,                 /* bcdUSB 2.00 */
  0x02,                       /* bDeviceClass: CDC */
  0x02,                       /* bDeviceSubClass */
  0x00,                       /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,           /* bMaxPacketSize0 */
  LOBYTE(USBD_VID), HIBYTE(USBD_VID),
  LOBYTE(USBD_PID), HIBYTE(USBD_PID),
  0x00, 0x02,                 /* bcdDevice 2.00 */
  USBD_IDX_MFC_STR,
  USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN static uint8_t s_langid_desc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN static uint8_t s_str_desc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;
__ALIGN_BEGIN static uint8_t s_serial_str[26] __ALIGN_END = {
  26, USB_DESC_TYPE_STRING
};

static void hex_to_unicode(uint32_t value, uint8_t *pbuf, uint8_t digits)
{
  for (uint8_t i = 0; i < digits; i++) {
    uint8_t nibble = (uint8_t)((value >> (28U - (4U * i))) & 0x0FU);
    pbuf[2U * i]      = (uint8_t)((nibble < 10U) ? (nibble + 0x30U) : (nibble + 0x41U - 10U));
    pbuf[2U * i + 1U] = 0;
  }
}

/** Build a UTF-16LE string descriptor from an ASCII literal. */
static void ascii_to_desc(const char *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 2;

  for (const char *p = desc; *p != '\0'; p++) {
    if ((uint32_t)idx >= (USBD_MAX_STR_DESC_SIZ - 2U)) {
      break;
    }
    unicode[idx++] = (uint8_t)*p;
    unicode[idx++] = 0;
  }
  unicode[0] = idx;
  unicode[1] = USB_DESC_TYPE_STRING;
  *len       = idx;
}

static uint8_t *Get_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  *length = sizeof(s_device_desc);
  return s_device_desc;
}

static uint8_t *Get_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  *length = sizeof(s_langid_desc);
  return s_langid_desc;
}

static uint8_t *Get_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ascii_to_desc(USBD_MANUFACTURER, s_str_desc, length);
  return s_str_desc;
}

static uint8_t *Get_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ascii_to_desc(USBD_PRODUCT, s_str_desc, length);
  return s_str_desc;
}

static uint8_t *Get_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  uint32_t d0 = *(uint32_t *)DEVICE_ID1;
  uint32_t d1 = *(uint32_t *)DEVICE_ID2;
  uint32_t d2 = *(uint32_t *)DEVICE_ID3;

  (void)speed;
  d0 += d2;
  if (d0 != 0U) {
    hex_to_unicode(d0, &s_serial_str[2], 8U);
    hex_to_unicode(d1, &s_serial_str[18], 4U);
  }
  *length = sizeof(s_serial_str);
  return s_serial_str;
}

static uint8_t *Get_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ascii_to_desc(USBD_CONFIGURATION, s_str_desc, length);
  return s_str_desc;
}

static uint8_t *Get_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ascii_to_desc(USBD_INTERFACE, s_str_desc, length);
  return s_str_desc;
}
