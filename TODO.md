# TODO

## Critical Bugs

### 1. CRC validation is permanently disabled on incoming packets

**File:** `App/Src/CmdRespProtocol.c:165`

The parser always accepts packets regardless of CRC:

```c
if ( true )    // TODO CRC disabled for now - was: (crcCalc == crcWord)
```

Any corrupted or malformed packet will be executed. Re-enable the check once the CRC polynomial and byte order used by the host is confirmed:

```c
if ( crcCalc == crcWord )
```

---

### 2. Outgoing packets always transmit CRC `0x0000`

**File:** `App/Src/CmdRespProtocol.c:215` and `CmdRespProtocol.c:230`

`crProtBuildSerialOutBuffer` and `crProtBuildSerialOutBufferSIGNED` both hardcode the outgoing CRC to zero instead of computing it. The host cannot detect corrupted responses.

---

### 3. Bitwise AND bug in `SATURN_BBC_STARTUP_R1` — startup may never complete

**File:** `Hal/Src/Saturn_spi.cpp:425` and `:432`

```c
while ( ( START_COUNT < 100 ) && ( !( ARG7 && 0x80 ) ) )  // BUG
if ( ( ARG7 && 0x80 ) )                                    // BUG
```

`&&` is logical AND, not bitwise AND. `ARG7 && 0x80` is `true` for any non-zero `ARG7`, not just when bit 7 is set. The intent is to poll the BCG (bandgap calibration done) bit. Should be:

```c
while ( ( START_COUNT < 100 ) && ( !( ARG7 & 0x80 ) ) )
if ( ( ARG7 & 0x80 ) )
```

The same bug exists in the Saturn2 equivalent.

---

### 4. `Saturn1_spi::ReadReturn` has no implementation — undefined behaviour

**File:** `Hal/Src/Saturn_spi.cpp:304`

The function body is entirely commented out. It returns a `CmdHandlerReturnCode_T` but falls off the end without a return statement — undefined behaviour in C++. It is reachable via `cmd_hndl_spi_transfer` when `cmdDataBuf[1] != 1`.

---

### 5. Ramp amplitude does not reset between sessions

**File:** `Hal/Src/Saturn_spi.cpp:1709`

```c
static uint32_t amp = ramping::min_amplitude;
```

The `static` variable is initialized once at program start. If a ramp is stopped mid-cycle and restarted, `amp` retains its last value rather than restarting from `min_amplitude`. Add an explicit reset:

```c
amp = ramping::min_amplitude;
```

at the top of `start_ramping()`, or when the ramp thread is started.

---

### 6. `threadManagerEntry` polls `threadRampStim` before it is ever created

**File:** `Core/Src/app_entry.c:366`

`stop_stim_thread` is initialized to `TX_TRUE`, so `threadManagerEntry` immediately calls `tx_thread_info_get(&threadRampStim, ...)` on the very first tick before any ramp has been started. `threadRampStim` is an uninitialised `TX_THREAD` struct at this point. The `TX_SUCCESS` guard prevents a crash, but the behaviour is implementation-defined. Initialise `stop_stim_thread` to `TX_FALSE` and only set it `TX_TRUE` after a ramp has been started at least once.

---

### 7. SPI2 bus shared between FRAM and Saturn2 without a mutex

**File:** `Hal/Src/FRAM.cpp`, `Hal/Src/Saturn2_spi.cpp`

`gpio_init()` / `gpio_deinit()` is used to hand off the SPI2 bus between FRAM and Saturn2. If `cmd_hndl_WriteCalValues` is called while Saturn2 is in the middle of a transaction (e.g. from `CurrentTick` in the TIM17 ISR), the bus ownership can corrupt both transfers. Protect all SPI2 bus handoffs with a mutex or disable the TIM17 ISR around the FRAM transaction.

---

### 8. HAL SPI return values silently ignored in FRAM

**File:** `Hal/Src/FRAM.cpp:113`, `:129`

`HAL_SPI_Transmit` return values in `write_enable()` and `write_single()` are stored in `hal_status` but never checked. A failed write to FRAM during calibration save will go undetected. Add `Error_Handler()` or return an error code on failure.

---

### 9. `printf` called inside `crc16CreateLookupTable` (CRCVER 1 branch)

**File:** `App/Src/Crc16.c:113–115`

The CRCVER==1 implementation calls `printf` to dump the first 16 table entries during initialization — this runs at boot and blocks on a UART that may not be configured. Remove or replace with `APP_DBG_MSG`.

---

## Improvements & Features

### 10. Consolidate stimulation parameter validation

Bounds checks are duplicated across `CommandHandler.cpp` (e.g. amplitude max `0x3a98`, pulse width max `0x989680`). Move them into a single `validate_stim_params()` helper or add `static_assert` / `constexpr` constants so limits only need updating in one place.

---

### 11. Persist stimulation parameters across power cycles

Calibration values are saved in FRAM but stimulation parameters (amplitude, pulse width, frequency, electrode mapping) are not. Storing the last-used configuration would allow the device to resume therapy after a reset without waiting for the host to re-send all parameters.

---

### 12. Watchdog timer

There is no hardware watchdog configured. A lockup in the ThreadX scheduler or an infinite loop in a driver (e.g. `SATURN_BBC_STARTUP_R1`) would leave the device unresponsive indefinitely. Enable the IWDG and kick it from `Thread Manager` or a dedicated low-priority thread.

---

### 13. Battery and rectifier voltage reporting to host

`Hal/Src/battery_charging.c` measures battery and rectifier voltages via ADC but the measured values are not proactively reported to the host. Add an unsolicited status packet (or a polled `CT_DeviceStatus` command) so the host can display charge level and alert on low battery.

---

### 14. USB CDC reconnection handling

If the USB host disconnects and reconnects while the device is running, there is no re-initialisation of the CDC interface or flush of the pending response queue. Stale bytes from before disconnection can be delivered to the new host session as if they were valid data.

---

### 15. BLE byte-input thread is commented out

**File:** `Core/Src/app_entry.c:348–358`

`threadBLEByteIn` is fully commented out. BLE-received bytes currently bypass the queued parser path. Wire `BLEByteInThreadEntry` through `crProtQueueByte` the same way USB does, so BLE packets go through the same CRC and framing validation.

---

### 16. `crProtBuildSerialOutBufferSIGNED` is a duplicate of `crProtBuildSerialOutBuffer`

**File:** `App/Src/CmdRespProtocol.c:220`

The two functions are identical except for the `rspBuf` pointer type (`int8_t*` vs `uint8_t*`). The cast difference can be handled at the call site with a single unsigned-buffer version, eliminating ~25 lines of duplicated code.

---

### 17. Replace magic register addresses in `DEFAULT_STIM` with named constants

**File:** `Hal/Src/Saturn_spi.cpp:465–888`

`DEFAULT_STIM` writes ~200 register addresses as bare hex literals with no indication of which Saturn register they correspond to. Defining named constants (e.g. `SAT_REG_ENABLE`, `SAT_REG_BC_CTRL0`) would make the startup configuration readable and make it obvious when a register is written in both `DEFAULT_STIM` and later command handlers.

---

### 18. OTA firmware update

The STM32WB55 BLE stack supports OTA DFU. Adding firmware update capability over BLE would allow field updates without physical access to the SWD port.
