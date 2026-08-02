/**
 * bno08x.h - minimal SHTP / SH-2 client for the BNO086 over SPI.
 *
 * This is deliberately not CEVA's full sh2 library: that pulls in more RAM
 * than the STM32F042C6 has (6 KiB). Only the subset needed to stream the
 * rotation vector, accelerometer, gyroscope and magnetometer is implemented.
 *
 * All values are passed through in the sensor's native fixed-point (Q) format
 * so the MCU never has to touch floating point; the host applies the scaling.
 */
#ifndef BNO08X_H
#define BNO08X_H

#include <stdint.h>
#include <stdbool.h>

/* --- SH-2 sensor report IDs -------------------------------------------- */
#define SH2_ACCELEROMETER            0x01u  /* Q8  m/s^2  */
#define SH2_GYROSCOPE_CALIBRATED     0x02u  /* Q9  rad/s  */
#define SH2_MAGNETIC_FIELD_CAL       0x03u  /* Q4  uT     */
#define SH2_LINEAR_ACCELERATION      0x04u  /* Q8  m/s^2  */
#define SH2_ROTATION_VECTOR          0x05u  /* Q14 unit quaternion + Q12 rad */
#define SH2_GRAVITY                  0x06u  /* Q8  m/s^2  */
#define SH2_GAME_ROTATION_VECTOR     0x08u  /* Q14, no magnetometer          */
#define SH2_GEOMAGNETIC_ROTATION_VEC 0x09u  /* Q14                           */
#define SH2_ARVR_STAB_ROTATION_VEC   0x28u  /* Q14                           */

/* --- how far the last reset attempt got -------------------------------- */
#define BNO08X_STAGE_NONE         0u  /* never attempted                     */
#define BNO08X_STAGE_NO_INT       1u  /* H_INTN never asserted - IMU silent  */
#define BNO08X_STAGE_SPI_SILENT   2u  /* INT asserted but SPI returned nothing */
#define BNO08X_STAGE_NO_RESET_MSG 3u  /* packets arrived, no reset-complete  */
#define BNO08X_STAGE_READY        4u  /* reset-complete seen                 */

typedef struct {
  uint16_t reset_attempts;
  uint16_t spi_timeouts;
  uint16_t packets_seen;
  uint8_t  last_stage;      /* BNO08X_STAGE_* */
  bool     int_asserted;    /* current H_INTN level (true = low = asserted) */
} bno08x_diag_t;

void bno08x_get_diag(bno08x_diag_t *d);

/* --- bring-up trace ----------------------------------------------------- */
#define BNO08X_TRACE_MAX     8u
#define BNO08X_TRACE_PREVIEW 4u

/** Header plus the first few payload bytes of one received SHTP packet. */
typedef struct {
  uint8_t  channel;
  uint16_t length;                        /* full SHTP length, header included */
  uint8_t  data[BNO08X_TRACE_PREVIEW];
} bno08x_trace_t;

/**
 * Copy the packets recorded since the last reset attempt. Recording stops
 * once the device is ready, so this only ever costs anything during bring-up.
 * @return number of entries written.
 */
uint8_t bno08x_get_trace(bno08x_trace_t *out, uint8_t max);

/* --- SPI bring-up scan -------------------------------------------------- */
#define BNO08X_SCAN_MAX 8u

/** Outcome of one (SPI mode, clock) combination. */
typedef struct {
  uint8_t  mode;          /* 0..3, the usual CPOL/CPHA numbering */
  uint16_t clock_khz;
  uint8_t  valid_packets; /* packets with a plausible SHTP header */
  uint8_t  got_reset_msg; /* 1 if the executable reset-complete appeared */
  uint8_t  first_channel;
  uint16_t first_length;
} bno08x_scan_t;

/**
 * Reset the BNO086 once per SPI mode/clock combination and report which one
 * produces valid SHTP. Takes a couple of seconds; intended for bring-up when
 * the configured settings do not work.
 * @return number of entries written.
 */
uint8_t bno08x_scan(bno08x_scan_t *out, uint8_t max);

/* --- pin-level probe ---------------------------------------------------- */

/**
 * Levels read back from MISO and H_INTN while the MCU biases them with its
 * own internal pull-up and then pull-down.
 *
 * If a line follows our bias in both directions it is floating - nothing on
 * the other end is driving it. If it holds one level against the opposing
 * bias, the BNO086 really is driving it. This separates "the IMU is not in
 * SPI mode / not soldered" from "the IMU is dead or unpowered" without a
 * meter on the board.
 */
typedef struct {
  /* MISO with CS released: an SPI slave must leave it high-Z here. */
  uint8_t miso_idle_pullup;
  uint8_t miso_idle_pulldown;
  /* MISO with CS asserted: an SPI slave must drive it here. If it still
   * follows our bias, the part is not selected - wrong protocol strap, a
   * broken CS net, or an open MISO joint. */
  uint8_t miso_sel_pullup;
  uint8_t miso_sel_pulldown;
  uint8_t int_with_pullup;
  uint8_t int_with_pulldown;
} bno08x_pinprobe_t;

void bno08x_probe_pins(bno08x_pinprobe_t *p);

/* --- bits in bno08x_data_t.updated ------------------------------------- */
#define BNO08X_UPD_QUAT  (1u << 0)
#define BNO08X_UPD_GYRO  (1u << 1)
#define BNO08X_UPD_ACCEL (1u << 2)
#define BNO08X_UPD_MAG   (1u << 3)

typedef struct {
  uint32_t host_us;     /* board_micros() when the report was pulled in */

  int16_t  qi, qj, qk, qr;  /* Q14 */
  int16_t  q_accuracy;      /* Q12, radians */
  uint8_t  q_status;        /* 0..3 accuracy class */

  int16_t  gx, gy, gz;      /* Q9  */
  uint8_t  g_status;

  int16_t  ax, ay, az;      /* Q8  */
  uint8_t  a_status;

  int16_t  mx, my, mz;      /* Q4  */
  uint8_t  m_status;

  uint16_t updated;         /* BNO08X_UPD_* accumulated since last clear */
} bno08x_data_t;

/**
 * Hold the part in reset, drive the PS0/PS1 straps for SPI, release it and
 * wait for the executable "reset complete" message.
 * @return false on timeout.
 */
bool bno08x_reset(void);

/** Enable a sensor report. interval_us == 0 disables it. */
bool bno08x_set_report(uint8_t report_id, uint32_t interval_us);

/**
 * Apply the standard report set: a rotation vector + gyro + accel at
 * imu_interval_us, magnetometer at mag_interval_us.
 *
 * @param rv_report_id which rotation vector to stream:
 *   SH2_ROTATION_VECTOR      - fuses the magnetometer, so yaw is an absolute
 *                              compass heading that survives a reset but is
 *                              wrecked by nearby steel or motors.
 *   SH2_GAME_ROTATION_VECTOR - gyro and accelerometer only. Yaw starts at 0
 *                              on every boot and is immune to magnetic
 *                              disturbance, at the cost of slow drift.
 *   Anything else falls back to SH2_ROTATION_VECTOR.
 */
bool bno08x_configure(uint32_t imu_interval_us, uint32_t mag_interval_us,
                      uint8_t rv_report_id);

/**
 * Drain at most one pending SHTP packet and merge any sensor reports it
 * carries into *data.
 * @return true if at least one sensor report was decoded.
 */
bool bno08x_service(bno08x_data_t *data);

/* Axis bitmap for bno08x_tare(). Zeroing yaw alone is usually what a robot
 * wants: roll and pitch stay referenced to gravity. */
#define BNO08X_TARE_AXIS_X   0x01u
#define BNO08X_TARE_AXIS_Y   0x02u
#define BNO08X_TARE_AXIS_Z   0x04u
#define BNO08X_TARE_AXIS_ALL 0x07u

/** Zero the current orientation on the given axes (SH-2 Tare Now). */
bool bno08x_tare(uint8_t axes);

/* --- last SH-2 command response ---------------------------------------- */
typedef struct {
  uint8_t valid;      /* 0 until the device has answered a command */
  uint8_t command;    /* the command it is responding to */
  uint8_t seq;        /* command sequence number echoed back */
  uint8_t r[4];       /* R0..R3; for Tare, R0 == 0 means success */
} bno08x_cmdresp_t;

/** Most recent SH-2 command response seen on the control channel. */
void bno08x_get_cmdresp(bno08x_cmdresp_t *out);

/** Persist the tare offset into flash. */
bool bno08x_persist_tare(void);

/** Save the dynamic calibration data record. */
bool bno08x_save_dcd(void);

/** Number of unexpected device resets seen since power-up. */
uint16_t bno08x_reset_count(void);

/** True once a reset-complete message has been observed. */
bool bno08x_is_ready(void);

#endif /* BNO08X_H */
