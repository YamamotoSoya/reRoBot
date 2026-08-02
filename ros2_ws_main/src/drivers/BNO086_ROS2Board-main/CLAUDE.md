# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A complete custom IMU board: KiCad 9.0 PCB design, STM32F042C6 firmware, and a ROS 2
driver package. The sensor is a **BNO086** 9-axis IMU; the MCU streams its reports to
a PC over USB CDC, and a host-side ROS 2 node publishes `sensor_msgs/Imu`.

| Path | Contents |
|---|---|
| `PCB/BNO086_ROS2Board/` | KiCad 9.0 schematic and board |
| `firmware/` | STM32F042C6 firmware (CMake + STM32 HAL, no CubeIDE) |
| `ros2_ws/src/bno086_imu_driver/` | ROS 2 Jazzy driver (ament_python) |
| `tools/` | udev rules; `serial_bridge.py` (serial↔TCP relay, macOS only) |
| `docker/` | ROS 2 container, only needed on macOS |
| `docs/` | `HARDWARE.md`, `PROTOCOL.md`, datasheets |

`README.md` is the detailed guide — read it before changing firmware or driver
behaviour, especially the "実機での検証状況" section, which records what has been
verified on hardware and what is still unresolved.

**The firmware does not come from CubeMX.** `BNO086_ROS2Board.ioc` is stale and does
not match the board (it puts the IMU interrupt on `PF11`, which does not exist on this
package). Do not regenerate from it. The authoritative pin map is
`firmware/Core/Inc/board.h`, extracted from the PCB netlist.

Constraints that shape almost every firmware decision: **32 KiB flash, 6 KiB RAM**
(currently 61% / 78% used). That rules out micro-ROS and CEVA's `sh2` library; the
firmware carries a minimal SHTP/SH-2 client (`firmware/App/bno08x.c`) and does no
floating-point work — fixed-point sensor values are forwarded as-is and scaled on the
host.

## Building and testing

**The deployment target is Ubuntu 24.04 with ROS 2 Jazzy installed natively.** The
macOS container setup under `docker/` exists because the board was brought up on a Mac;
treat it as a secondary path (README 付録) and keep the native instructions primary.

```sh
# Firmware
cmake -S firmware -B build/fw -G Ninja && cmake --build build/fw

# ROS 2 driver tests - 52 of them, none need the board
source /opt/ros/jazzy/setup.bash
cd ros2_ws/src/bno086_imu_driver && python3 -m pytest test -q
```

`.github/workflows/ci.yml` runs both on every push and pull request.

Everything recorded under 実機での検証状況 was measured on macOS + Docker + the TCP
relay. Nothing has been confirmed on native Ubuntu yet — do not describe that path as
verified.

Flashing needs **S2 (BOOT) held while S1 (RESET) is pressed** to enter the ROM DFU
bootloader. A firmware-side `ENTER_DFU` command was tried and removed — with BOOT0 low
the bootloader jumps straight back to user code.

## Hardware facts that are easy to get wrong

- **Protocol straps: SPI needs PS1=1 AND PS0=1**, both high from before NRST is
  released until the first `H_INTN`. PS1 comes from the R7 pull-up, PS0 from firmware
  holding PB0 high. Getting PS0 wrong selects UART-RVC and the IMU never answers SPI.
  JP1/JP2 must stay open.
- **U4 (the BNO086) is placed at 180° on the PCB**, so its +X/+Y point along the
  board's −X/−Y. The driver compensates with `mount_yaw_deg` (default `180.0`) and
  publishes in the board frame.
- CAN has circuitry but no firmware, and U5's `Vio` wiring to +5 V is unverified.

## KiCad tooling

The board is normally edited in the **KiCad 9.0 GUI**. For headless / scripted tasks
use `kicad-cli` (run from `PCB/BNO086_ROS2Board/`):

```sh
# Electrical Rules Check (schematic)
kicad-cli sch erc BNO086_ROS2Board.kicad_sch -o erc.rpt

# Design Rules Check (board)
kicad-cli pcb drc BNO086_ROS2Board.kicad_pcb -o drc.rpt

# Export Gerbers / drill / PDF / 3D step
kicad-cli pcb export gerbers BNO086_ROS2Board.kicad_pcb -o gerbers/
kicad-cli pcb export drill   BNO086_ROS2Board.kicad_pcb -o gerbers/
kicad-cli sch export pdf      BNO086_ROS2Board.kicad_sch -o schematic.pdf
```

**Manufacturing outputs are produced by the "Fabrication Toolkit" KiCad plugin** (a
GUI plugin, not CLI), configured by `fabrication-toolkit-options.json`. It writes
JLCPCB-ready files into `production/` (`bom.csv`, `designators.csv`, `positions.csv`,
`netlist.ipc`). Treat `production/` as generated output — regenerate it via the plugin
rather than hand-editing. `bom/ibom.html` is a generated interactive BOM.

## Schematic architecture

The schematic is **hierarchical**, rooted at `BNO086_ROS2Board.kicad_sch` with two
child sheets:

- **Root sheet** — STM32F042C6 MCU (U3), USB-C (J1), UART/CAN/VIN/SWD connectors
  (J2–J5), CAN transceiver (U5), 3.3 V LDO (U2, TLV74033), BOOT/RESET, the BNO086
  `PS0`/`PS1` protocol-select solder jumpers (JP1/JP2).
- **`IMU.kicad_sch`** — BNO086 sensor (U4) and its 32.768 kHz crystal (Y1).
- **`DC-DC.kicad_sch`** — buck converter front-end: NJW1933F1 controller (IC1),
  22 µH inductor (L1), Schottky diodes (D1/D2/D3/D7). Provides the upstream rail that
  the LDO regulates to 3.3 V.

### Key MCU interface mapping

`firmware/Core/Inc/board.h` is the source of truth for pin usage — **not** the `.ioc`.
The board is a **2-layer** design (F.Cu / B.Cu). Notable connections:

- **SPI1** (PA5 SCK / PA6 MISO / PA7 MOSI, PA4 CS) → BNO086 (mode 3, 3 MHz)
- **CAN** (PB8 RX / PB9 TX) → transceiver → CAN connector (J5)
- **USART2** (PA2/PA3) → UART connector (J4); **USART1** (PA9/PA10)
- **USB** (PA11/PA12) → USB-C (J1)
- **SWD** (PA13/PA14) → SWD header (J3)
- Clocks: HSE 8 MHz crystal (Y2) × PLL6 = 48 MHz, mandatory for USB; 32.768 kHz (Y1)
  for the BNO086

## Custom libraries

Third-party parts not in the KiCad standard libraries live under `libs/` and are wired
in via `sym-lib-table` / `fp-lib-table`: `BNO086/` (symbol + footprint),
`NJW1933F1-AT-TE2/`, `TCAN844DDFRQ1/`. When adding a part from a vendor, follow this
same pattern (drop it in `libs/<part>/` and register it in the lib tables) rather than
editing global KiCad libraries.

## Conventions

- KiCad writes timestamped auto-backup zips into `BNO086_ROS2Board-backups/` and
  `production/backups/`. These are machine-generated and churn constantly in
  `git status` — do not hand-edit them, and generally don't commit noise from them.
- Commit messages in this repo are written in **Japanese** — match that style.
- Firmware and driver comments are written in **English**; the README and `docs/` are
  in Japanese. Match whichever file you are editing.
- Anything not yet confirmed on real hardware belongs in the README's 未検証 list
  rather than being described as working.
