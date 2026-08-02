#include "host_link.h"
#include "board.h"
#include <string.h>

#ifdef HOST_LINK_USB
#include "usbd_cdc_if.h"
#endif

#if defined(HOST_LINK_USB) && defined(HOST_LINK_UART)
#error "Pick one host transport: -DHOST_LINK_USB=ON or -DHOST_LINK_UART=ON"
#endif
#if !defined(HOST_LINK_USB) && !defined(HOST_LINK_UART)
#error "No host transport selected"
#endif

#define TX_RING_SIZE 512u          /* power of two */
#define RX_RING_SIZE 128u          /* power of two */
#define MAX_PAYLOAD  32u

static uint8_t           s_tx_ring[TX_RING_SIZE];
static volatile uint16_t s_tx_head, s_tx_tail;

static uint8_t           s_rx_ring[RX_RING_SIZE];
static volatile uint16_t s_rx_head, s_rx_tail;

static host_cmd_t        s_cmd;
static bool              s_cmd_pending;

/* ------------------------------------------------------------------ CRC -- */

static uint16_t crc16_ccitt(uint16_t crc, uint8_t byte)
{
  crc ^= (uint16_t)byte << 8;
  for (uint8_t i = 0; i < 8u; i++) {
    crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
  }
  return crc;
}

/* ---------------------------------------------------------- transmit ring - */

static inline uint16_t tx_free(void)
{
  return (uint16_t)(TX_RING_SIZE - 1u - ((s_tx_head - s_tx_tail) & (TX_RING_SIZE - 1u)));
}

static void tx_push(uint8_t b)
{
  s_tx_ring[s_tx_head] = b;
  s_tx_head = (uint16_t)((s_tx_head + 1u) & (TX_RING_SIZE - 1u));
}

/**
 * Frame and enqueue one message. If the ring cannot hold the whole frame the
 * message is dropped rather than truncated - a partial frame would desync the
 * host parser, and losing one sample of a 400 Hz stream is harmless.
 */
static void send_frame(uint8_t id, const uint8_t *payload, uint8_t len)
{
  uint16_t crc = 0xFFFFu;

  if (tx_free() < (uint16_t)(len + 6u)) {
    return;
  }

  tx_push(HL_SYNC0);
  tx_push(HL_SYNC1);
  tx_push(id);
  tx_push(len);
  crc = crc16_ccitt(crc, id);
  crc = crc16_ccitt(crc, len);
  for (uint8_t i = 0; i < len; i++) {
    tx_push(payload[i]);
    crc = crc16_ccitt(crc, payload[i]);
  }
  tx_push((uint8_t)(crc & 0xFFu));
  tx_push((uint8_t)(crc >> 8));
}

/* ------------------------------------------------------------ packing ----- */

static uint8_t *put_u16(uint8_t *p, uint16_t v)
{
  *p++ = (uint8_t)(v & 0xFFu);
  *p++ = (uint8_t)(v >> 8);
  return p;
}

static uint8_t *put_u32(uint8_t *p, uint32_t v)
{
  *p++ = (uint8_t)(v & 0xFFu);
  *p++ = (uint8_t)((v >> 8) & 0xFFu);
  *p++ = (uint8_t)((v >> 16) & 0xFFu);
  *p++ = (uint8_t)((v >> 24) & 0xFFu);
  return p;
}

static inline uint32_t get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void host_link_send_imu(const bno08x_data_t *d)
{
  uint8_t  buf[28];
  uint8_t *p = buf;

  p = put_u32(p, d->host_us);
  p = put_u16(p, (uint16_t)d->qi);
  p = put_u16(p, (uint16_t)d->qj);
  p = put_u16(p, (uint16_t)d->qk);
  p = put_u16(p, (uint16_t)d->qr);
  p = put_u16(p, (uint16_t)d->q_accuracy);
  p = put_u16(p, (uint16_t)d->gx);
  p = put_u16(p, (uint16_t)d->gy);
  p = put_u16(p, (uint16_t)d->gz);
  p = put_u16(p, (uint16_t)d->ax);
  p = put_u16(p, (uint16_t)d->ay);
  p = put_u16(p, (uint16_t)d->az);

  *p++ = (uint8_t)((d->q_status & 0x03u) |
                   ((d->g_status & 0x03u) << 2) |
                   ((d->a_status & 0x03u) << 4));
  *p++ = (uint8_t)(((d->updated & BNO08X_UPD_QUAT)  ? HL_FLAG_QUAT_VALID  : 0u) |
                   ((d->updated & BNO08X_UPD_GYRO)  ? HL_FLAG_GYRO_VALID  : 0u) |
                   ((d->updated & BNO08X_UPD_ACCEL) ? HL_FLAG_ACCEL_VALID : 0u));

  send_frame(HL_MSG_IMU, buf, (uint8_t)(p - buf));
}

void host_link_send_mag(const bno08x_data_t *d)
{
  uint8_t  buf[12];
  uint8_t *p = buf;

  p = put_u32(p, d->host_us);
  p = put_u16(p, (uint16_t)d->mx);
  p = put_u16(p, (uint16_t)d->my);
  p = put_u16(p, (uint16_t)d->mz);
  *p++ = (uint8_t)(d->m_status & 0x03u);
  *p++ = 0u;

  send_frame(HL_MSG_MAG, buf, (uint8_t)(p - buf));
}

void host_link_send_status(uint16_t reset_count, bool imu_ready, uint32_t frames,
                           const bno08x_diag_t *diag)
{
  uint8_t  buf[20];
  uint8_t *p = buf;

  p = put_u32(p, board_micros());
  p = put_u16(p, reset_count);
  *p++ = (uint8_t)((imu_ready ? HL_STATUS_IMU_READY : 0u) |
                   ((diag != NULL && diag->int_asserted) ? HL_STATUS_INT_ASSERTED : 0u));
  *p++ = HL_PROTO_VERSION;
  p = put_u32(p, frames);
  p = put_u16(p, (diag != NULL) ? diag->reset_attempts : 0u);
  p = put_u16(p, (diag != NULL) ? diag->spi_timeouts : 0u);
  p = put_u16(p, (diag != NULL) ? diag->packets_seen : 0u);
  *p++ = (diag != NULL) ? diag->last_stage : 0u;
  *p++ = 0u;                          /* reserved */

  send_frame(HL_MSG_STATUS, buf, (uint8_t)(p - buf));
}

void host_link_send_trace(void)
{
  bno08x_trace_t entries[BNO08X_TRACE_MAX];
  uint8_t        buf[1u + (BNO08X_TRACE_MAX * (3u + BNO08X_TRACE_PREVIEW))];
  uint8_t       *p = buf;
  uint8_t        n = bno08x_get_trace(entries, BNO08X_TRACE_MAX);

  *p++ = n;
  for (uint8_t i = 0; i < n; i++) {
    *p++ = entries[i].channel;
    p    = put_u16(p, entries[i].length);
    for (uint8_t j = 0; j < BNO08X_TRACE_PREVIEW; j++) {
      *p++ = entries[i].data[j];
    }
  }

  send_frame(HL_MSG_TRACE, buf, (uint8_t)(p - buf));
}

void host_link_send_scan(const bno08x_scan_t *results, uint8_t n)
{
  uint8_t  buf[1u + (BNO08X_SCAN_MAX * 8u)];
  uint8_t *p = buf;

  if (n > BNO08X_SCAN_MAX) {
    n = BNO08X_SCAN_MAX;
  }

  *p++ = n;
  for (uint8_t i = 0; i < n; i++) {
    *p++ = results[i].mode;
    p    = put_u16(p, results[i].clock_khz);
    *p++ = results[i].valid_packets;
    *p++ = results[i].got_reset_msg;
    *p++ = results[i].first_channel;
    p    = put_u16(p, results[i].first_length);
  }

  send_frame(HL_MSG_SCAN, buf, (uint8_t)(p - buf));
}

void host_link_send_pins(const bno08x_pinprobe_t *p)
{
  uint8_t buf[6];

  buf[0] = p->miso_idle_pullup;
  buf[1] = p->miso_idle_pulldown;
  buf[2] = p->miso_sel_pullup;
  buf[3] = p->miso_sel_pulldown;
  buf[4] = p->int_with_pullup;
  buf[5] = p->int_with_pulldown;

  send_frame(HL_MSG_PINS, buf, sizeof(buf));
}

void host_link_send_cmdresp(const bno08x_cmdresp_t *r)
{
  uint8_t buf[7];

  buf[0] = r->valid;
  buf[1] = r->command;
  buf[2] = r->seq;
  buf[3] = r->r[0];
  buf[4] = r->r[1];
  buf[5] = r->r[2];
  buf[6] = r->r[3];

  send_frame(HL_MSG_CMDRESP, buf, sizeof(buf));
}

/* ------------------------------------------------------------- transports - */

void host_link_init(void)
{
  s_tx_head = s_tx_tail = 0;
  s_rx_head = s_rx_tail = 0;
  memset(&s_cmd, 0, sizeof(s_cmd));
  s_cmd_pending = false;
}

void host_link_flush(void)
{
  uint16_t head = s_tx_head;
  uint16_t tail = s_tx_tail;
  uint16_t chunk;

  if (head == tail) {
    return;
  }

  /* Longest run that does not wrap the ring. */
  chunk = (head > tail) ? (uint16_t)(head - tail)
                        : (uint16_t)(TX_RING_SIZE - tail);

#ifdef HOST_LINK_USB
  /* Keep every transfer short of wMaxPacketSize so the CDC class never has
   * to append a zero length packet. */
  if (chunk > 63u) {
    chunk = 63u;
  }
  if (CDC_Transmit_FS(&s_tx_ring[tail], chunk) != USBD_OK) {
    /* Endpoint busy, or nothing enumerated on the other end. Consume
     * nothing and retry next iteration; with no host attached the ring
     * fills up and send_frame() starts dropping whole frames. */
    return;
  }
#else
  /* Blocking, but at 921600 baud a 34 byte frame is ~370 us and the BNO086
   * keeps H_INTN asserted meanwhile, so no samples are lost. */
  (void)HAL_UART_Transmit(&huart1, &s_tx_ring[tail], chunk, 10u);
#endif

  s_tx_tail = (uint16_t)((tail + chunk) & (TX_RING_SIZE - 1u));
}

void host_link_rx_bytes(const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++) {
    uint16_t next = (uint16_t)((s_rx_head + 1u) & (RX_RING_SIZE - 1u));
    if (next == s_rx_tail) {
      return;                       /* overflow: drop the rest */
    }
    s_rx_ring[s_rx_head] = data[i];
    s_rx_head = next;
  }
}

/* -------------------------------------------------------- command decode -- */

static void apply_command(uint8_t id, const uint8_t *payload, uint8_t len)
{
  switch (id) {
    case HL_MSG_SET_RATE:
      if (len >= 8u) {
        s_cmd.set_rate        = true;
        s_cmd.imu_interval_us = get_u32(&payload[0]);
        s_cmd.mag_interval_us = get_u32(&payload[4]);
        /* Byte 8 picks the rotation vector; older hosts omit it and keep the
         * magnetometer-referenced one. */
        s_cmd.rv_report_id    = (len >= 9u && payload[8] != 0u)
                                  ? SH2_GAME_ROTATION_VECTOR
                                  : SH2_ROTATION_VECTOR;
        s_cmd_pending = true;
      }
      break;
    case HL_MSG_TARE:
      s_cmd.tare         = true;
      s_cmd.tare_persist = (len >= 1u && payload[0] != 0u);
      /* Byte 1 is the axis bitmap; older hosts omit it and get all axes. */
      s_cmd.tare_axes    = (len >= 2u) ? payload[1] : 0u;
      s_cmd_pending = true;
      break;
    case HL_MSG_SAVE_DCD:
      s_cmd.save_dcd = true;
      s_cmd_pending  = true;
      break;
    case HL_MSG_RESET_IMU:
      s_cmd.reset_imu = true;
      s_cmd_pending   = true;
      break;
    case HL_MSG_PING:
      s_cmd.ping    = true;
      s_cmd_pending = true;
      break;
    default:
      break;
  }
}

bool host_link_poll(host_cmd_t *cmd)
{
  /* Parser state persists across calls because bytes trickle in. */
  static enum { W_SYNC0, W_SYNC1, W_ID, W_LEN, W_PAYLOAD, W_CRC0, W_CRC1 } st = W_SYNC0;
  static uint8_t  id, len, idx, payload[MAX_PAYLOAD];
  static uint16_t crc, rx_crc;

#ifdef HOST_LINK_UART
  /* Poll USART1 without interrupts; one byte per call is plenty for a
   * command channel. */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
    uint8_t b = (uint8_t)(huart1.Instance->RDR & 0xFFu);
    host_link_rx_bytes(&b, 1u);
  }
  /* Clear the sticky error flags that would otherwise stall the receiver. */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_NEFLAG(&huart1);
  __HAL_UART_CLEAR_FEFLAG(&huart1);
#endif

  while (s_rx_tail != s_rx_head) {
    uint8_t b = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & (RX_RING_SIZE - 1u));

    switch (st) {
      case W_SYNC0:
        if (b == HL_SYNC0) { st = W_SYNC1; }
        break;
      case W_SYNC1:
        st = (b == HL_SYNC1) ? W_ID : W_SYNC0;
        break;
      case W_ID:
        id  = b;
        crc = crc16_ccitt(0xFFFFu, b);
        st  = W_LEN;
        break;
      case W_LEN:
        len = b;
        crc = crc16_ccitt(crc, b);
        idx = 0;
        if (len > MAX_PAYLOAD) {
          st = W_SYNC0;              /* not a frame we can hold */
        } else {
          st = (len == 0u) ? W_CRC0 : W_PAYLOAD;
        }
        break;
      case W_PAYLOAD:
        payload[idx++] = b;
        crc = crc16_ccitt(crc, b);
        if (idx >= len) { st = W_CRC0; }
        break;
      case W_CRC0:
        rx_crc = b;
        st = W_CRC1;
        break;
      case W_CRC1:
        rx_crc |= (uint16_t)b << 8;
        if (rx_crc == crc) {
          apply_command(id, payload, len);
        }
        st = W_SYNC0;
        break;
    }
  }

  if (s_cmd_pending) {
    *cmd = s_cmd;
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd_pending = false;
    return true;
  }
  return false;
}
