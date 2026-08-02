/**
 * host_link.h - framed binary link to the PC.
 *
 * Frame layout (little endian throughout):
 *
 *   0xAA 0x55 | id:u8 | len:u8 | payload[len] | crc16:u16
 *
 * The CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) taken over
 * id, len and payload. See docs/PROTOCOL.md for the payload definitions.
 *
 * Sensor values keep the BNO086's native Q formats so the MCU never runs
 * floating point; the ROS 2 node applies the scale factors.
 */
#ifndef HOST_LINK_H
#define HOST_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "bno08x.h"

#define HL_PROTO_VERSION 2u

#define HL_SYNC0 0xAAu
#define HL_SYNC1 0x55u

/* device -> host */
#define HL_MSG_IMU     0x01u
#define HL_MSG_MAG     0x02u
#define HL_MSG_STATUS  0x03u
#define HL_MSG_TRACE   0x04u
#define HL_MSG_SCAN    0x05u
#define HL_MSG_PINS    0x06u
#define HL_MSG_CMDRESP 0x07u

/* host -> device */
#define HL_MSG_SET_RATE  0x80u
#define HL_MSG_TARE      0x81u
#define HL_MSG_SAVE_DCD  0x82u
#define HL_MSG_RESET_IMU 0x83u
#define HL_MSG_PING      0x84u

/* HL_MSG_STATUS flags byte */
#define HL_STATUS_IMU_READY    (1u << 0)
#define HL_STATUS_INT_ASSERTED (1u << 1)

/* HL_MSG_IMU flags byte */
#define HL_FLAG_QUAT_VALID  (1u << 0)
#define HL_FLAG_GYRO_VALID  (1u << 1)
#define HL_FLAG_ACCEL_VALID (1u << 2)

/** Commands decoded from the host, drained by the main loop. */
typedef struct {
  bool     set_rate;
  uint32_t imu_interval_us;
  uint32_t mag_interval_us;
  uint8_t  rv_report_id;   /* SH2_ROTATION_VECTOR or SH2_GAME_ROTATION_VECTOR */
  bool     tare;
  bool     tare_persist;
  uint8_t  tare_axes;      /* BNO08X_TARE_AXIS_* bitmap; 0 means all */
  bool     save_dcd;
  bool     reset_imu;
  bool     ping;
} host_cmd_t;

void host_link_init(void);

void host_link_send_imu(const bno08x_data_t *d);
void host_link_send_mag(const bno08x_data_t *d);
void host_link_send_status(uint16_t reset_count, bool imu_ready, uint32_t frames,
                           const bno08x_diag_t *diag);

/** Emit the SHTP packets recorded during the last bring-up attempt. */
void host_link_send_trace(void);

/** Emit the result of a SPI mode/clock scan. */
void host_link_send_scan(const bno08x_scan_t *results, uint8_t n);

/** Emit the MISO / H_INTN pin-level probe. */
void host_link_send_pins(const bno08x_pinprobe_t *p);

/** Emit the last SH-2 command response, so the host can see if it worked. */
void host_link_send_cmdresp(const bno08x_cmdresp_t *r);

/** Push queued bytes towards USB/UART. Call every main-loop iteration. */
void host_link_flush(void);

/** Decode incoming bytes; returns true if any command field was set. */
bool host_link_poll(host_cmd_t *cmd);

/** Feed bytes received from the USB CDC endpoint into the command decoder. */
void host_link_rx_bytes(const uint8_t *data, uint32_t len);

#endif /* HOST_LINK_H */
