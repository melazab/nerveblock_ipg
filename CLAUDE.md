# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

Requires the `arm-none-eabi-gcc` cross-compiler toolchain on `PATH`.

```bash
# Full build — produces .elf, .hex, and .bin in bld_saturn_wave_fwr_output/
make

# Specify a custom GCC path
make GCC_PATH=/path/to/arm-none-eabi/bin

# Build with a custom baud rate define
make BAUD_RATE=115200

# Clean all build artifacts
make clean

# Build with release optimization (edit Makefile: OPT = -O2, DEBUG = 0)
make
```

Build outputs land in `bld_saturn_wave_fwr_output/` (`.elf`, `.hex`, `.bin`). Object files go in `bld_saturn_wave_fwr_obj/` (overridable with `OBJ_BLD_ROOT`). There are no automated tests or linter targets — this is embedded firmware.

The wireless coprocessor (CPU2) requires `stm32wb5x_BLE_Stack_fw.bin` pre-flashed via STM32CubeProgrammer before the application will run.

## Architecture

### Hardware Target

**STM32WB55** dual-core SoC:
- **CPU1 (Cortex-M4 @ 32 MHz)** — runs all application code in this repo
- **CPU2 (Cortex-M0+)** — runs the closed-source BLE wireless firmware; communicated with via IPCC/SHCI

The board is a nerve-block implantable pulse generator (IPG) driving two proprietary stimulation ASICs ("Saturn").

### Layer Map

```
App/         — Command protocol parsing and dispatch
Hal/         — Hardware drivers: Saturn ASICs, FRAM, Impedance, USB, BLE, GPIO
Core/        — STM32 HAL init (CubeMX-generated), ThreadX task creation
STM32_WPAN/  — BLE peer-to-peer server application (GATT notifications)
Middlewares/ — ThreadX RTOS, STM32 WPAN stack (vendor-provided, do not edit)
Drivers/     — STM32 HAL/CMSIS drivers (vendor-provided, do not edit)
Utilities/   — ST low-power manager and sequencer utilities
```

### Boot & Initialization Sequence (`Core/Src/main.cpp`)

1. `HAL_Init()` → `MX_APPE_Config()` (HSE tuning, device reset)
2. `SystemClock_Config()` — HSE as SYSCLK, LSE for RTC/RF wake-up
3. ITM/DWT trace enabled for SWO debugging
4. Peripherals: DMA → ADC1 → RF → RTC → RNG → TIM2
5. `Impedance::start_up_seq()`, `FRAM::enable()` / `FRAM::ReadCalValues()` / `FRAM::gpio_deinit()`
6. `Saturn1_spi` and `Saturn2_spi`: `SATURN_DIG_STARTUP` → `SATURN_BBC_STARTUP_R1` → `DEFAULT_STIM` → `Calibration`
7. `MX_APPE_Init()` — system power config, timer server, UART RX
8. `MX_ThreadX_Init()` → `tx_kernel_enter()` — never returns; all runtime work is in ThreadX threads

### ThreadX Tasks (created in `tx_application_define`, `Core/Src/app_entry.c`)

| Thread | Priority | Purpose |
|---|---|---|
| `thread_ShciUserEvtProcess` | 16 | Handles async events from CPU2 wireless firmware via SHCI |
| `CmdParserRecvByte` | 15 | Dequeues bytes from `queueCmdParser` and feeds them to the packet parser |
| `DebugSerialByteIn` | 25 | Reads incoming USB bytes and enqueues them for parsing |
| `Thread Manager` | 15 | Lifecycle manager — terminates/deletes `RampStimulation` thread when signalled |
| `Battery Charging` | 20 | ADC-based battery/rectifier voltage monitoring (`Hal/Src/battery_charging.c`) |
| `RampStimulation` | 4 | Dynamically created/deleted per ramp session; drives amplitude ramping loop |

### Command / Response Protocol (`App/`)

Binary framing over USB CDC or BLE GATT notifications:

```
[0xFC][0x1A][token][resp_byte][data_len][data_0..n][crc_lo][crc_hi]
```

- Sync bytes: `0xFC 0x1A`
- `resp_byte` upper nibble encodes bits 8–9 of `data_len`; lower nibble is response code
- CRC-16 is **currently disabled** — the parser always accepts any CRC (`CmdRespProtocol.c:165`)

Flow: USB/BLE ISR → `crProtQueueByte()` → `queueCmdParser` → `CmdParserRecvByte` thread → `crProtNewInCmdChar()` (state machine) → `cmdHndlCommandRecv()` → `cmdHndlProcessCmd()` → `cmdHandlerTable[token].handler(cmd)` → response sent back via `Trigger_Send_Notification()` (BLE) or `usbSendSerialDataBuf()` (USB).

Command tokens and all error codes are defined in `App/Inc/CmdProtocolDefinitions.h`. The dispatch table in `CommandHandler.cpp` maps token byte values 0x00–0x28 to handler functions.

### Saturn ASIC Drivers (`Hal/Src/Saturn_spi.cpp`, `Hal/Src/Saturn2_spi.cpp`)

- **Saturn1** ("HF STIM") — High-frequency stimulator, SPI1 on GPIOA pins
- **Saturn2** ("LF STIM") — Low-frequency stimulator, SPI2 on GPIOD pins
- Both share the same logical API (startup, calibration, amplitude, pulse width, frequency, electrode enable/disable, sequencer RAM, charge limit)
- **Harmonics mode** (`bool harmonics` in `CommandHandler.cpp`): routes both sequencer banks through Saturn1 to produce a fundamental + 2nd-harmonic waveform. When harmonics is active, many command handlers call `Saturn1_spi::UpdateSequencerAddresses(2)` then repeat the operation, then `UpdateSequencerAddresses(1)` to restore
- GPIO pins for each SPI are manually toggled (bit-banged init) then handed to the SPI peripheral; `gpio_init()`/`gpio_deinit()` switch ownership between FRAM and Saturn2 (they share bus pins)

### FRAM (`Hal/Src/FRAM.cpp`)

Ferroelectric RAM for calibration value persistence. Shares the SPI2 bus with Saturn2 — callers must call `Saturn2_spi::gpio_deinit()` before `FRAM::gpio_init()` and reverse after. Calibration values are loaded into `calvals[1024]` at boot and applied to both Saturn ASICs.

### Impedance Measurement (`Hal/Src/Impedance.cpp`)

Controlled by `IMP_ENn_Pin` and a MUX select pin. `CloseSwitches()` + `Saturn2_spi::ElectrodeEnable()` sets up the measurement path; `StartMeasurement()` triggers ADC-based streaming. The `measuringimp` flag gates stimulation during measurement.

### BLE (`STM32_WPAN/App/`, `Middlewares/ST/STM32_WPAN/`)

Peer-to-peer GATT server. `APP_BLE_Get_Server_Connection_Status()` determines whether responses go over BLE notifications or USB. BLE stack runs on CPU2; CPU1 communicates via the SHCI/TL transport layer.

### Key Cross-Cutting Concerns

- **SPI bus sharing**: FRAM and Saturn2 share physical SPI2 lines. Always deinit one before initing the other.
- **`harmonics` global**: declared in `CommandHandler.cpp`, `extern`'d in `CommandHandler.hpp`. Several command handlers change behavior based on this flag — always check it when modifying stimulation parameter handlers.
- **Ramp stimulation thread lifecycle**: `start_stim_control()` creates the thread; `stop_stim_control()` sets a flag that `Thread Manager` polls to terminate and delete it. Direct `tx_thread_terminate` is used intentionally because the ramp loop cannot self-exit cleanly mid-sleep.
- **`TIM17` ISR** (`HAL_TIM_PeriodElapsedCallback`): drives `HAL_IncTick()` and forwards the tick to both Saturn drivers for timing their stimulation windows.
