/**
 * BNO086_ROS2Board - firmware entry point.
 *
 * Streams BNO086 sensor reports to a PC over USB CDC (or USART1) using the
 * framed binary protocol in docs/PROTOCOL.md. A ROS 2 node on the host turns
 * those frames into sensor_msgs/Imu and sensor_msgs/MagneticField.
 */
#include "main.h"
#include "bno08x.h"
#include "host_link.h"

#ifdef HOST_LINK_USB
#include "usb_device.h"
#endif

/* Defaults; the host can override them at runtime with HL_MSG_SET_RATE. */
#define DEFAULT_IMU_INTERVAL_US 10000u   /* 100 Hz */
#define DEFAULT_MAG_INTERVAL_US 40000u   /*  25 Hz */

#define STATUS_PERIOD_MS 1000u
#define RETRY_PERIOD_MS  2000u

static bno08x_data_t s_data;


static void handle_command(const host_cmd_t *cmd, uint32_t *imu_us, uint32_t *mag_us,
                           uint8_t *rv_id)
{
  if (cmd->set_rate) {
    *imu_us = cmd->imu_interval_us;
    *mag_us = cmd->mag_interval_us;
    *rv_id  = cmd->rv_report_id;
    (void)bno08x_configure(*imu_us, *mag_us, *rv_id);
  }
  if (cmd->tare) {
    (void)bno08x_tare(cmd->tare_axes);
    if (cmd->tare_persist) {
      /* The tare has to be applied before it can be persisted. 20 ms was not
       * enough for the persist to stick, so give the device room here. */
      HAL_Delay(200);
      (void)bno08x_persist_tare();
    }
  }
  if (cmd->save_dcd) {
    (void)bno08x_save_dcd();
  }
  if (cmd->reset_imu) {
    if (bno08x_reset()) {
      (void)bno08x_configure(*imu_us, *mag_us, *rv_id);
    }
  }
  if (cmd->ping) {
    bno08x_diag_t diag;
    bno08x_get_diag(&diag);
    host_link_send_status(bno08x_reset_count(), bno08x_is_ready(), 0, &diag);
  }
}

int main(void)
{
  uint32_t   imu_us = DEFAULT_IMU_INTERVAL_US;
  uint32_t   mag_us = DEFAULT_MAG_INTERVAL_US;
  uint8_t    rv_id  = SH2_ROTATION_VECTOR;
  uint32_t   frames = 0;
  uint32_t   last_status = 0;
  uint32_t   last_retry  = 0;
  uint32_t   last_imu_us = 0;
  bool       configured  = false;
  host_cmd_t cmd;

  bno08x_scan_t     scan[BNO08X_SCAN_MAX];
  uint8_t           scan_n    = 0;
  bool              scan_done = false;
  bno08x_pinprobe_t pins      = {0};

  HAL_Init();
  SystemClock_Config();
  board_init();
  host_link_init();

#ifdef HOST_LINK_USB
  MX_USB_DEVICE_Init();
#endif

  LED1_ON();                       /* powered and running */

  if (bno08x_reset()) {
    configured = bno08x_configure(imu_us, mag_us, rv_id);
    LED2_ON();                     /* IMU up */
  }

  for (;;) {
    uint32_t now = HAL_GetTick();

    host_link_flush();

    if (host_link_poll(&cmd)) {
      handle_command(&cmd, &imu_us, &mag_us, &rv_id);
    }

    /* The BNO086 may not have been ready at boot (slow supply ramp) or may
     * have browned out; keep trying rather than requiring a power cycle. */
    if (!configured) {
      /* The configured SPI settings did not bring the IMU up. Sweep the four
       * SPI modes at two clocks once, so the host can see which - if any -
       * the device actually answers on. */
      if (!scan_done) {
        scan_done = true;
        bno08x_probe_pins(&pins);
        scan_n     = bno08x_scan(scan, BNO08X_SCAN_MAX);
        last_retry = HAL_GetTick();
      }
      if ((now - last_retry) >= RETRY_PERIOD_MS) {
        last_retry = now;
        LED2_OFF();
        if (bno08x_reset()) {
          configured = bno08x_configure(imu_us, mag_us, rv_id);
          if (configured) {
            LED2_ON();
          }
        }
      }
    } else if (bno08x_service(&s_data)) {
      const uint16_t imu_bits = BNO08X_UPD_QUAT | BNO08X_UPD_GYRO | BNO08X_UPD_ACCEL;

      /* The rotation vector, gyroscope and accelerometer usually arrive in
       * three separate SHTP packets. Emitting one frame per packet would
       * triple the link rate and hand the host three messages per sample
       * period with only one field fresh in each. Instead the reports are
       * accumulated and sent once all three are fresh.
       *
       * Anchoring on the rotation vector alone is not enough: above ~200 Hz
       * it often arrives before the matching gyro and accelerometer reports,
       * which would emit a frame carrying stale values for them.
       *
       * The elapsed-time fallback covers a dropped report and the case where
       * a sensor is not subscribed at all. */
      if (s_data.updated & imu_bits) {
        uint32_t since_us = board_micros() - last_imu_us;
        bool     complete = (s_data.updated & imu_bits) == imu_bits;

        if (complete || since_us >= (imu_us * 2u)) {
          host_link_send_imu(&s_data);
          s_data.updated &= (uint16_t)~imu_bits;
          last_imu_us = board_micros();
          frames++;
        }
      }
      if (s_data.updated & BNO08X_UPD_MAG) {
        host_link_send_mag(&s_data);
        s_data.updated &= (uint16_t)~BNO08X_UPD_MAG;
      }
    }

    if ((now - last_status) >= STATUS_PERIOD_MS) {
      bno08x_diag_t diag;

      last_status = now;
      bno08x_get_diag(&diag);
      host_link_send_status(bno08x_reset_count(), bno08x_is_ready(), frames, &diag);
      {
        bno08x_cmdresp_t resp;
        bno08x_get_cmdresp(&resp);
        if (resp.valid) {
          host_link_send_cmdresp(&resp);
        }
      }
      if (!configured) {
        /* Bring-up aids: what the IMU answered, and which SPI settings (if
         * any) it answered on at all. */
        host_link_send_trace();
        if (scan_done) {
          host_link_send_pins(&pins);
        }
        if (scan_n > 0u) {
          host_link_send_scan(scan, scan_n);
        }
      }
      LED1_TOG();                  /* 1 Hz heartbeat */
    }
  }
}
