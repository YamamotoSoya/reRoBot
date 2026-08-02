#include "bno08x.h"
#include "board.h"
#include <string.h>

/* --- SHTP channels ------------------------------------------------------ */
#define CH_COMMAND      0u   /* advertisement / SHTP housekeeping */
#define CH_EXECUTABLE   1u   /* reset / on / sleep               */
#define CH_CONTROL      2u   /* SH-2 control (set feature, ...)  */
#define CH_INPUT_NORMAL 3u   /* sensor reports                   */
#define CH_INPUT_WAKE   4u
#define CH_INPUT_GYRO   5u
#define CH_COUNT        6u

/* --- SH-2 control opcodes ---------------------------------------------- */
#define SHTP_REPORT_BASE_TIMESTAMP 0xFBu
#define SH2_SET_FEATURE_COMMAND    0xFDu
#define SH2_COMMAND_REQUEST        0xF2u
#define SH2_COMMAND_RESPONSE       0xF1u

#define SH2_CMD_TARE               0x03u
#define SH2_CMD_SAVE_DCD           0x06u

#define TARE_SUB_NOW               0x00u
#define TARE_SUB_PERSIST           0x01u
#define TARE_AXIS_ALL              0x07u
#define TARE_BASIS_ROTATION_VECTOR      0x00u
#define TARE_BASIS_GAME_ROTATION_VECTOR 0x01u

/* Executable channel payloads */
#define EXEC_RESET_COMPLETE        0x01u

/* Big enough for every report we subscribe to in one packet; longer packets
 * (the ~276 byte boot advertisement) are clocked out and discarded. */
#define RX_BUF_SIZE  128u
#define TX_BUF_SIZE  24u

#define SHTP_HDR_LEN 4u

static uint8_t  s_rx[RX_BUF_SIZE];
static uint8_t  s_tx[TX_BUF_SIZE];
static uint8_t  s_seq[CH_COUNT];
static uint16_t s_reset_count;
static bool     s_ready;

/* Diagnostics - reported in the STATUS frame so a board that will not come
 * up can be triaged without a debugger attached. */
static bool     s_spi_fault;
static uint16_t s_spi_timeouts;
static uint16_t s_reset_attempts;
static uint16_t s_packets_seen;
static uint8_t  s_last_stage;

static bno08x_trace_t s_trace[BNO08X_TRACE_MAX];
static uint8_t        s_trace_count;

static bno08x_cmdresp_t s_cmdresp;

/* Cached subscription so the reports can be restored after a device reset. */
static uint32_t s_imu_interval_us;
static uint32_t s_mag_interval_us;
static uint8_t  s_rv_report_id = SH2_ROTATION_VECTOR;

/* ------------------------------------------------------------ SPI layer -- */

static inline void cs_low(void)  { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET); }
static inline bool int_asserted(void)
{
  return HAL_GPIO_ReadPin(IMU_INT_PORT, IMU_INT_PIN) == GPIO_PIN_RESET;
}

/**
 * Full-duplex byte loop straight on the SPI registers.
 * HAL_SPI_TransmitReceive() would need matching buffers and adds ~2 us of
 * call overhead per chunk, which is significant at a 400 Hz report rate.
 * tx == NULL clocks out zeros, rx == NULL discards the input.
 */
/* One byte at 3 MHz takes ~2.7 us; this bound is ~1 ms of spinning, far
 * beyond any legitimate wait but short enough that a dead SPI slave cannot
 * wedge the main loop. */
#define SPI_SPIN_LIMIT 20000u

static inline uint8_t spi_byte(uint8_t out)
{
  SPI_TypeDef *spi = IMU_SPI;
  uint32_t     spins;

  spins = SPI_SPIN_LIMIT;
  while ((spi->SR & SPI_SR_TXE) == 0u) {
    if (--spins == 0u) {
      s_spi_fault = true;
      s_spi_timeouts++;
      return 0xFFu;
    }
  }
  *(volatile uint8_t *)&spi->DR = out;

  spins = SPI_SPIN_LIMIT;
  while ((spi->SR & SPI_SR_RXNE) == 0u) {
    if (--spins == 0u) {
      s_spi_fault = true;
      s_spi_timeouts++;
      return 0xFFu;
    }
  }
  return *(volatile uint8_t *)&spi->DR;
}

static void spi_xfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
  for (uint16_t i = 0; i < n; i++) {
    uint8_t d = spi_byte((tx != NULL) ? tx[i] : 0x00u);
    if (rx != NULL) {
      rx[i] = d;
    }
  }
}

/** Largest SHTP packet the BNO086 ever sends (the boot advertisement). */
#define SHTP_MAX_PACKET 512u

/**
 * Sanity-check an SHTP header before acting on it.
 * A packet must name a defined channel and carry a length that at least
 * covers its own header.
 */
static bool shtp_header_plausible(const uint8_t *hdr, uint16_t len)
{
  if (len < SHTP_HDR_LEN || len > SHTP_MAX_PACKET) {
    return false;
  }
  if (hdr[2] >= CH_COUNT) {
    return false;
  }
  return true;
}

/** Assert H_WAKEN until the device answers with H_INTN, so we may write. */
static bool wake_device(uint32_t timeout_ms)
{
  uint32_t start;

  if (int_asserted()) {
    return true;                     /* already has our attention */
  }

  HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN, GPIO_PIN_RESET);
  start = HAL_GetTick();
  while (!int_asserted()) {
    if ((HAL_GetTick() - start) > timeout_ms) {
      HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN, GPIO_PIN_SET);
      return false;
    }
  }
  HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN, GPIO_PIN_SET);
  return true;
}

/**
 * One SHTP bus transaction.
 *
 * SPI is symmetric here: whatever we send goes out while the device's own
 * pending packet comes back, so a write and a read always happen together.
 * The transaction length is the longer of the two packets.
 *
 * @param tx      packet to send including its 4 byte SHTP header, or NULL
 * @param tx_len  length of tx
 * @param rx_len  out: length of the received packet payload (header stripped),
 *                clamped to what fitted in s_rx
 * @param rx_ch   out: channel the received packet arrived on
 * @return true if a packet was received
 */
static bool shtp_transact(const uint8_t *tx, uint16_t tx_len,
                          uint16_t *rx_len, uint8_t *rx_ch)
{
  uint8_t  hdr[SHTP_HDR_LEN];
  uint16_t dev_len, body, stored, tx_payload;

  *rx_len = 0;
  *rx_ch  = 0xFF;

  s_spi_fault = false;
  cs_low();
  /* The BNO08x needs a short setup time after CS before the first edge. */
  board_delay_us(2);

  spi_xfer(tx, hdr, SHTP_HDR_LEN);

  dev_len = (uint16_t)hdr[0] | ((uint16_t)(hdr[1] & 0x7Fu) << 8);
  *rx_ch  = hdr[2];

  /* Reject anything that cannot be a real SHTP header. 0x0000 means "nothing
   * to send"; a floating MISO produces huge lengths and channels far outside
   * the defined range, and clocking out those phantom lengths would stall the
   * bus for thousands of bytes. */
  if (!shtp_header_plausible(hdr, dev_len)) {
    dev_len = 0u;
  }

  /* Clock out the rest of the longer of the two packets. */
  body = (tx_len > dev_len) ? tx_len : dev_len;
  body = (body > SHTP_HDR_LEN) ? (uint16_t)(body - SHTP_HDR_LEN) : 0u;

  /* Bytes of the device's packet we have room to keep. The remainder (the
   * ~276 byte boot advertisement, mostly) still has to be clocked out or the
   * byte stream would lose framing. */
  stored     = (dev_len > SHTP_HDR_LEN) ? (uint16_t)(dev_len - SHTP_HDR_LEN) : 0u;
  stored     = (stored > RX_BUF_SIZE) ? RX_BUF_SIZE : stored;
  tx_payload = (tx != NULL && tx_len > SHTP_HDR_LEN) ? (uint16_t)(tx_len - SHTP_HDR_LEN) : 0u;

  for (uint16_t i = 0; i < body; i++) {
    uint8_t in = spi_byte((i < tx_payload) ? tx[SHTP_HDR_LEN + i] : 0x00u);
    if (i < stored) {
      s_rx[i] = in;
    }
  }
  *rx_len = stored;

  cs_high();

  if (s_spi_fault) {
    return false;
  }
  if (dev_len > 0u) {
    s_packets_seen++;

    /* Log the first few packets of a bring-up attempt so the host can see
     * exactly what the device answered. Stops once streaming starts. */
    if (!s_ready && s_trace_count < BNO08X_TRACE_MAX) {
      bno08x_trace_t *t = &s_trace[s_trace_count++];
      t->channel = *rx_ch;
      t->length  = dev_len;
      for (uint8_t i = 0; i < BNO08X_TRACE_PREVIEW; i++) {
        t->data[i] = (i < stored) ? s_rx[i] : 0x00u;
      }
    }
  }
  return dev_len > 0u;
}

uint8_t bno08x_get_trace(bno08x_trace_t *out, uint8_t max)
{
  uint8_t n = (s_trace_count < max) ? s_trace_count : max;

  for (uint8_t i = 0; i < n; i++) {
    out[i] = s_trace[i];
  }
  return n;
}

/** Build an SHTP header in front of a payload already sitting in s_tx. */
static bool shtp_send(uint8_t channel, uint16_t payload_len)
{
  uint16_t total = payload_len + SHTP_HDR_LEN;
  uint16_t rx_len;
  uint8_t  rx_ch;

  if (total > TX_BUF_SIZE) {
    return false;
  }
  if (!wake_device(20u)) {
    return false;
  }

  s_tx[0] = (uint8_t)(total & 0xFFu);
  s_tx[1] = (uint8_t)((total >> 8) & 0x7Fu);
  s_tx[2] = channel;
  s_tx[3] = s_seq[channel]++;

  (void)shtp_transact(s_tx, total, &rx_len, &rx_ch);
  return true;
}

/* ------------------------------------------------------------- SH-2 API -- */

bool bno08x_set_report(uint8_t report_id, uint32_t interval_us)
{
  uint8_t *p = &s_tx[SHTP_HDR_LEN];

  memset(p, 0, 17);
  p[0]  = SH2_SET_FEATURE_COMMAND;
  p[1]  = report_id;
  /* p[2] feature flags, p[3..4] change sensitivity: leave at 0 */
  p[5]  = (uint8_t)(interval_us & 0xFFu);
  p[6]  = (uint8_t)((interval_us >> 8) & 0xFFu);
  p[7]  = (uint8_t)((interval_us >> 16) & 0xFFu);
  p[8]  = (uint8_t)((interval_us >> 24) & 0xFFu);
  /* p[9..12] batch interval, p[13..16] sensor specific config: 0 */

  return shtp_send(CH_CONTROL, 17u);
}

static bool sh2_command(uint8_t command, const uint8_t params[9])
{
  uint8_t *p = &s_tx[SHTP_HDR_LEN];
  static uint8_t cmd_seq;

  memset(p, 0, 12);
  p[0] = SH2_COMMAND_REQUEST;
  p[1] = cmd_seq++;
  p[2] = command;
  if (params != NULL) {
    memcpy(&p[3], params, 9);
  }
  return shtp_send(CH_CONTROL, 12u);
}

bool bno08x_tare(uint8_t axes)
{
  uint8_t params[9] = {0};

  params[0] = TARE_SUB_NOW;
  params[1] = (axes == 0u) ? BNO08X_TARE_AXIS_ALL : (axes & BNO08X_TARE_AXIS_ALL);
  /* The basis has to name the rotation vector actually being streamed.
   * Taring against the magnetometer-referenced vector leaves the game
   * rotation vector untouched, which looks exactly like the command being
   * ignored. */
  params[2] = (s_rv_report_id == SH2_GAME_ROTATION_VECTOR)
                ? TARE_BASIS_GAME_ROTATION_VECTOR
                : TARE_BASIS_ROTATION_VECTOR;
  return sh2_command(SH2_CMD_TARE, params);
}

bool bno08x_persist_tare(void)
{
  uint8_t params[9] = {0};
  params[0] = TARE_SUB_PERSIST;
  return sh2_command(SH2_CMD_TARE, params);
}

bool bno08x_save_dcd(void)
{
  return sh2_command(SH2_CMD_SAVE_DCD, NULL);
}

uint16_t bno08x_reset_count(void) { return s_reset_count; }
bool     bno08x_is_ready(void)    { return s_ready; }

void bno08x_get_cmdresp(bno08x_cmdresp_t *out)
{
  *out = s_cmdresp;
}

void bno08x_get_diag(bno08x_diag_t *d)
{
  d->reset_attempts = s_reset_attempts;
  d->spi_timeouts   = s_spi_timeouts;
  d->packets_seen   = s_packets_seen;
  d->last_stage     = s_last_stage;
  d->int_asserted   = int_asserted();
}

/* --------------------------------------------------------------- reset --- */

bool bno08x_reset(void)
{
  uint32_t start;
  uint16_t rx_len;
  uint8_t  rx_ch;
  bool     saw_int    = false;
  bool     saw_packet = false;

  s_ready = false;
  s_reset_attempts++;
  s_trace_count = 0;
  memset(s_seq, 0, sizeof(s_seq));

  __HAL_SPI_ENABLE(&hspi1);

  /* Protocol straps: PS1 = 1 (R7 pull-up, JP2 open) and PS0 = 1 selects SPI.
   * Both must stay high from before NRST is released until the device makes
   * its first H_INTN assertion - see the note in board.c. PS0 only becomes
   * the H_WAKEN input afterwards. */
  HAL_GPIO_WritePin(IMU_BOOT_PORT, IMU_BOOT_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN, GPIO_PIN_SET);
  cs_high();

  HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_SET);

  /* The part answers with an advertisement packet on channel 0 followed by
   * "reset complete" on the executable channel. */
  start = HAL_GetTick();
  while ((HAL_GetTick() - start) < 1000u) {
    if (!int_asserted()) {
      continue;
    }
    saw_int = true;
    if (!shtp_transact(NULL, 0, &rx_len, &rx_ch)) {
      continue;
    }
    saw_packet = true;
    if (rx_ch == CH_EXECUTABLE && rx_len >= 1u && s_rx[0] == EXEC_RESET_COMPLETE) {
      s_ready = true;
      break;
    }
  }

  /* Record how far we got so the host can tell "the IMU is not there at all"
   * apart from "it talks but never announced a reset". */
  if (s_ready) {
    s_last_stage = BNO08X_STAGE_READY;
  } else if (saw_packet) {
    s_last_stage = BNO08X_STAGE_NO_RESET_MSG;
  } else if (saw_int) {
    s_last_stage = BNO08X_STAGE_SPI_SILENT;
  } else {
    s_last_stage = BNO08X_STAGE_NO_INT;
  }

  /* PS0 is the H_WAKEN input from now on. It is already at the idle level,
   * since SPI selection required it high throughout the reset. */
  HAL_Delay(10);

  return s_ready;
}

/* --------------------------------------------------------- bring-up scan -- */

/** Reconfigure SPI1 for one of the four classic modes at a given prescaler. */
static void spi_set_mode(uint8_t mode, uint32_t prescaler)
{
  __HAL_SPI_DISABLE(&hspi1);
  hspi1.Init.CLKPolarity       = (mode & 0x02u) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = (mode & 0x01u) ? SPI_PHASE_2EDGE   : SPI_PHASE_1EDGE;
  hspi1.Init.BaudRatePrescaler = prescaler;
  (void)HAL_SPI_Init(&hspi1);
  __HAL_SPI_ENABLE(&hspi1);
}

uint8_t bno08x_scan(bno08x_scan_t *out, uint8_t max)
{
  static const uint8_t  modes[4]  = {0u, 1u, 2u, 3u};
  static const uint32_t presc[2]  = {SPI_BAUDRATEPRESCALER_16, SPI_BAUDRATEPRESCALER_64};
  static const uint16_t khz[2]    = {3000u, 750u};

  uint8_t n = 0;

  for (uint8_t pi = 0; pi < 2u && n < max; pi++) {
    for (uint8_t mi = 0; mi < 4u && n < max; mi++) {
      bno08x_scan_t *r = &out[n++];
      uint32_t       start;
      uint16_t       rx_len;
      uint8_t        rx_ch;

      r->mode          = modes[mi];
      r->clock_khz     = khz[pi];
      r->valid_packets = 0;
      r->got_reset_msg = 0;
      r->first_channel = 0xFFu;
      r->first_length  = 0;

      spi_set_mode(modes[mi], presc[pi]);

      /* Full reset so the device re-runs its boot handshake for this mode,
       * with both protocol straps high to select SPI. */
      HAL_GPIO_WritePin(IMU_BOOT_PORT, IMU_BOOT_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN, GPIO_PIN_SET);
      cs_high();
      HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_RESET);
      HAL_Delay(10);
      HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_SET);

      start = HAL_GetTick();
      while ((HAL_GetTick() - start) < 300u) {
        if (!int_asserted()) {
          continue;
        }
        if (!shtp_transact(NULL, 0, &rx_len, &rx_ch)) {
          continue;
        }
        if (r->valid_packets < 255u) {
          r->valid_packets++;
        }
        if (r->first_channel == 0xFFu) {
          r->first_channel = rx_ch;
          r->first_length  = (uint16_t)(rx_len + SHTP_HDR_LEN);
        }
        if (rx_ch == CH_EXECUTABLE && rx_len >= 1u && s_rx[0] == EXEC_RESET_COMPLETE) {
          r->got_reset_msg = 1u;
          break;
        }
      }
    }
  }

  /* Leave the bus on the configured defaults again. */
  spi_set_mode(3u, SPI_BAUDRATEPRESCALER_16);
  return n;
}

/** Read one pin as a plain input under a given internal bias. */
static uint8_t probe_pin(GPIO_TypeDef *port, uint16_t pin, uint32_t pull)
{
  GPIO_InitTypeDef g = {0};

  g.Pin   = pin;
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = pull;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &g);

  /* Let the line settle: an internal pull-up is ~40 kohm and the trace plus
   * pad capacitance needs a moment against it. */
  HAL_Delay(2);

  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
}

void bno08x_probe_pins(bno08x_pinprobe_t *p)
{
  GPIO_InitTypeDef g = {0};

  __HAL_SPI_DISABLE(&hspi1);

  cs_high();
  p->miso_idle_pullup   = probe_pin(IMU_SPI_PORT, IMU_MISO_PIN, GPIO_PULLUP);
  p->miso_idle_pulldown = probe_pin(IMU_SPI_PORT, IMU_MISO_PIN, GPIO_PULLDOWN);

  cs_low();
  p->miso_sel_pullup    = probe_pin(IMU_SPI_PORT, IMU_MISO_PIN, GPIO_PULLUP);
  p->miso_sel_pulldown  = probe_pin(IMU_SPI_PORT, IMU_MISO_PIN, GPIO_PULLDOWN);
  cs_high();

  p->int_with_pullup    = probe_pin(IMU_INT_PORT, IMU_INT_PIN, GPIO_PULLUP);
  p->int_with_pulldown  = probe_pin(IMU_INT_PORT, IMU_INT_PIN, GPIO_PULLDOWN);

  /* Hand MISO back to SPI1 and restore the interrupt input bias. */
  g.Pin       = IMU_MISO_PIN;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = IMU_SPI_AF;
  HAL_GPIO_Init(IMU_SPI_PORT, &g);

  (void)probe_pin(IMU_INT_PORT, IMU_INT_PIN, GPIO_PULLUP);

  __HAL_SPI_ENABLE(&hspi1);
}

bool bno08x_configure(uint32_t imu_interval_us, uint32_t mag_interval_us,
                      uint8_t rv_report_id)
{
  bool    ok = true;
  uint8_t other;

  if (rv_report_id != SH2_ROTATION_VECTOR &&
      rv_report_id != SH2_GAME_ROTATION_VECTOR) {
    rv_report_id = SH2_ROTATION_VECTOR;
  }

  s_imu_interval_us = imu_interval_us;
  s_mag_interval_us = mag_interval_us;
  s_rv_report_id    = rv_report_id;

  /* Switching flavours: silence the other one first, or both would stream
   * into the same quaternion fields. */
  other = (rv_report_id == SH2_ROTATION_VECTOR) ? SH2_GAME_ROTATION_VECTOR
                                                : SH2_ROTATION_VECTOR;
  (void)bno08x_set_report(other, 0u);
  HAL_Delay(2);

  ok &= bno08x_set_report(rv_report_id, imu_interval_us);
  HAL_Delay(2);
  ok &= bno08x_set_report(SH2_GYROSCOPE_CALIBRATED, imu_interval_us);
  HAL_Delay(2);
  ok &= bno08x_set_report(SH2_ACCELEROMETER, imu_interval_us);
  HAL_Delay(2);
  if (mag_interval_us > 0u) {
    ok &= bno08x_set_report(SH2_MAGNETIC_FIELD_CAL, mag_interval_us);
    HAL_Delay(2);
  }
  return ok;
}

/* -------------------------------------------------------------- parsing -- */

static inline int16_t rd16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** Total length of a sensor report, including its 4 byte prefix. */
static uint8_t report_len(uint8_t id)
{
  switch (id) {
    case SH2_ACCELEROMETER:
    case SH2_GYROSCOPE_CALIBRATED:
    case SH2_MAGNETIC_FIELD_CAL:
    case SH2_LINEAR_ACCELERATION:
    case SH2_GRAVITY:                    return 10u;
    case SH2_GAME_ROTATION_VECTOR:       return 12u;
    case SH2_ROTATION_VECTOR:
    case SH2_GEOMAGNETIC_ROTATION_VEC:
    case SH2_ARVR_STAB_ROTATION_VEC:     return 14u;
    default:                             return 0u;  /* unknown - stop */
  }
}

static bool parse_input_reports(uint16_t len, bno08x_data_t *d)
{
  uint16_t i = 0;
  bool     got = false;

  while (i < len) {
    uint8_t id = s_rx[i];

    if (id == SHTP_REPORT_BASE_TIMESTAMP) {
      i += 5u;                       /* opcode + int32 base delta */
      continue;
    }

    uint8_t rlen = report_len(id);
    /* An unrecognised ID means we no longer know where the next report
     * starts, so abandon the rest of the packet rather than emit garbage. */
    if (rlen == 0u || (uint16_t)(i + rlen) > len) {
      break;
    }

    const uint8_t *r      = &s_rx[i];
    uint8_t        status = r[2] & 0x03u;

    switch (id) {
      case SH2_ROTATION_VECTOR:
      case SH2_ARVR_STAB_ROTATION_VEC:
      case SH2_GEOMAGNETIC_ROTATION_VEC:
        d->qi         = rd16(&r[4]);
        d->qj         = rd16(&r[6]);
        d->qk         = rd16(&r[8]);
        d->qr         = rd16(&r[10]);
        d->q_accuracy = rd16(&r[12]);
        d->q_status   = status;
        d->updated   |= BNO08X_UPD_QUAT;
        got = true;
        break;

      case SH2_GAME_ROTATION_VECTOR:
        d->qi         = rd16(&r[4]);
        d->qj         = rd16(&r[6]);
        d->qk         = rd16(&r[8]);
        d->qr         = rd16(&r[10]);
        d->q_accuracy = 0;
        d->q_status   = status;
        d->updated   |= BNO08X_UPD_QUAT;
        got = true;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        d->gx = rd16(&r[4]);
        d->gy = rd16(&r[6]);
        d->gz = rd16(&r[8]);
        d->g_status = status;
        d->updated |= BNO08X_UPD_GYRO;
        got = true;
        break;

      case SH2_ACCELEROMETER:
        d->ax = rd16(&r[4]);
        d->ay = rd16(&r[6]);
        d->az = rd16(&r[8]);
        d->a_status = status;
        d->updated |= BNO08X_UPD_ACCEL;
        got = true;
        break;

      case SH2_MAGNETIC_FIELD_CAL:
        d->mx = rd16(&r[4]);
        d->my = rd16(&r[6]);
        d->mz = rd16(&r[8]);
        d->m_status = status;
        d->updated |= BNO08X_UPD_MAG;
        got = true;
        break;

      default:
        break;
    }
    i += rlen;
  }

  if (got) {
    d->host_us = board_micros();
  }
  return got;
}

bool bno08x_service(bno08x_data_t *data)
{
  uint16_t rx_len;
  uint8_t  rx_ch;

  if (!int_asserted()) {
    return false;
  }
  if (!shtp_transact(NULL, 0, &rx_len, &rx_ch)) {
    return false;
  }

  switch (rx_ch) {
    case CH_INPUT_NORMAL:
    case CH_INPUT_WAKE:
      return parse_input_reports(rx_len, data);

    case CH_EXECUTABLE:
      if (rx_len >= 1u && s_rx[0] == EXEC_RESET_COMPLETE) {
        /* The device rebooted on its own (brown-out, ESD). Re-subscribe so
         * the stream comes back without host intervention. */
        s_reset_count++;
        s_ready = true;
        memset(s_seq, 0, sizeof(s_seq));
        HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN, GPIO_PIN_SET);
        (void)bno08x_configure(s_imu_interval_us, s_mag_interval_us, s_rv_report_id);
      }
      return false;

    case CH_CONTROL:
      /* Command responses tell us whether Tare and Save DCD were accepted.
       * Layout: F1, seq, command, cmd_seq, resp_seq, R0..R10 */
      if (rx_len >= 9u && s_rx[0] == SH2_COMMAND_RESPONSE) {
        s_cmdresp.valid   = 1u;
        s_cmdresp.command = s_rx[2];
        s_cmdresp.seq     = s_rx[3];
        s_cmdresp.r[0]    = s_rx[5];
        s_cmdresp.r[1]    = s_rx[6];
        s_cmdresp.r[2]    = s_rx[7];
        s_cmdresp.r[3]    = s_rx[8];
      }
      return false;

    default:
      /* The channel 0 advertisement is not needed for streaming. */
      return false;
  }
}
