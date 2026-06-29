# NerveBlock IPG Firmware

Firmware for an implantable pulse generator (IPG) targeting the **STM32WB55** microcontroller. The device drives two proprietary stimulation ASICs ("Saturn") to deliver biphasic electrical stimulation for peripheral nerve block applications.

## Hardware

| Component | Details |
|---|---|
| MCU | STM32WB55RGVx (Cortex-M4 @ 32 MHz + Cortex-M0+ BLE coprocessor) |
| HF Stimulator | Saturn1 ASIC via SPI1 (GPIOA) |
| LF Stimulator | Saturn2 ASIC via SPI2 (GPIOD) |
| Calibration storage | Ferroelectric RAM (FRAM) via SPI2 (GPIOB) |
| Communication | USB CDC and/or BLE GATT notifications |
| RTOS | Azure RTOS ThreadX |

> **Note:** The STM32WB55 wireless coprocessor (CPU2) must have `stm32wb5x_BLE_Stack_fw.bin` pre-flashed via STM32CubeProgrammer before running this firmware. Binaries are located under `Projects/STM32_Copro_Wireless_Binaries` in the ST firmware package.

## Prerequisites

- `arm-none-eabi-gcc` toolchain on `PATH` (or pass `GCC_PATH=...` to make)
- STM32CubeProgrammer (for flashing)
- GNU Make

## Build

```bash
# Debug build (default)
make

# Release build — edit Makefile: OPT = -O2, DEBUG = 0
make

# Custom GCC toolchain path
make GCC_PATH=/opt/arm-none-eabi/bin

# Custom baud rate
make BAUD_RATE=115200

# Clean
make clean
```

Output files are placed in `bld_saturn_wave_fwr_output/`:

| File | Use |
|---|---|
| `saturn_wave_fwr.elf` | Debug symbol file for GDB / IDE |
| `saturn_wave_fwr.hex` | Flash with STM32CubeProgrammer |
| `saturn_wave_fwr.bin` | Raw binary |

## Flashing

```bash
STM32_Programmer_CLI -c port=SWD -w bld_saturn_wave_fwr_output/saturn_wave_fwr.hex -v -rst
```

## Project Structure

```
App/        Command protocol parser and stimulation command handlers
Core/       STM32 HAL peripheral init, ThreadX task creation (app_entry.c)
Hal/        Hardware drivers: Saturn ASICs, FRAM, Impedance, USB, GPIO
STM32_WPAN/ BLE peer-to-peer GATT server application
Drivers/    STM32 HAL/CMSIS (vendor, do not edit)
Middlewares/ ThreadX RTOS + STM32 WPAN stack (vendor, do not edit)
Utilities/  ST low-power manager and sequencer utilities
```

## Communication Protocol

Commands are sent as binary-framed packets over USB CDC or BLE GATT:

```
[0xFC][0x1A][token][resp_byte][data_len][data...][crc_lo][crc_hi]
```

- Sync bytes: `0xFC 0x1A`
- `token`: command identifier (see `App/Inc/CmdProtocolDefinitions.h`)
- `resp_byte`: upper nibble encodes bits 8–9 of `data_len`; lower nibble is the response code
- CRC-16 is present in the frame but currently bypassed (all packets accepted)

The device automatically selects the active transport: BLE notifications when a client is connected, USB otherwise.

## Key Subsystems

### Saturn ASIC Drivers (`Hal/Src/Saturn_spi.cpp`, `Hal/Src/Saturn2_spi.cpp`)

Each Saturn ASIC is controlled via SPI and exposes registers for pulse width, amplitude, frequency, interphase delay, electrode mapping, and waveform shape. Stimulation parameters are encoded as clock-divisor values (`register = 2²⁴ / period_µs`).

**Harmonics mode** (`bool harmonics`) allows Saturn1 to drive two sequencer banks simultaneously at different frequencies, enabling fundamental + 2nd-harmonic waveforms.

**Charge safety:** After every amplitude or pulse-width change, `chargecheck()` verifies that charge-per-phase (`amplitude_µA × PW_µs / 10000`) does not exceed the per-electrode limit. A `CT_ChargeLimit` unsolicited response is sent to the host if any channel violates the limit.

### FRAM (`Hal/Src/FRAM.cpp`)

Stores factory calibration values for both Saturn ASICs. Read at boot into `calvals[]` and applied to the ASICs via `Calibration()`. The SPI2 bus is shared between FRAM and Saturn2 — `gpio_init()`/`gpio_deinit()` must be called to switch ownership.

### Impedance Measurement (`Hal/Src/Impedance.cpp`)

ADC-based differential impedance measurement. `StartMeasurement()` starts a DMA-driven ADC capture while Saturn2 delivers a stimulation waveform, then streams the results back as a series of 130-byte `CT_ImpedanceData` packets (64 signed int16 mV samples per packet, last packet flagged with MSB set).

### Ramp Stimulation

Amplitude ramps from a minimum to maximum over a configurable duty cycle, synchronized to the `HF_STIM_SYNC` GPIO edge. The ramp runs in a dedicated ThreadX thread (`RampStimulation`) that is dynamically created and destroyed per session by `start_stim_control()` / `stop_stim_control()`.

## SWO Trace

ITM trace is enabled at startup on port 0. Connect a SWO-capable debugger (e.g. ST-LINK) and use STM32CubeIDE or OpenOCD to capture `APP_DBG_MSG()` output.
