# Roomba 900 BMS unlock

ESP-IDF firmware for an ESP32-WROOM-32. It reaches a TI fuel gauge through a TCA9548A and, when that gauge is in permanent-fail, clears the latch so the pack FETs can close.

The gauge is expected at SMBus address `0x0B`. A Roomba 900 pack that answers with ManufacturerData `00010302` is a bq20z70/bq20z75; the device type is the word read back from `ManufacturerAccess` after writing `0x0001` (default `0x0700`), not that ASCII string. bq30z55 / bq30z554 and bq40z50 are still accepted. A 4S pack with cells near 4.1 V then has about 16.4 V on the battery output tabs.

## Wiring

ESP32-WROOM-32 to the TCA9548A upstream bus:

| ESP32 | TCA9548A (TSSOP-24) |
| --- | --- |
| GPIO21 SDA | SDA, pin 23 |
| GPIO22 SCL | SCL, pin 22 |
| 3.3 V | VCC pin 24, and RESET pin 3 |
| GND | GND pin 12, and the BMS ground |

Tie A2, A1, and A0 (pins 21, 2, and 1) to GND so the mux answers at `0x70`.

Channel 3 is the pair the firmware selects:

| TCA9548A | BMS |
| --- | --- |
| SD3, pin 10 | pack SDA |
| SC3, pin 11 | pack SCL |

Put a 3k resistor from the ESP SDA to 3.3 V and another from SCL to 3.3 V. The mux does not pull up its downstream pins, so SD3 and SC3 need the same 3k to 3.3 V unless the pack board already has pull-ups.

The gauge is powered by the cells. Connect SDA, SCL, and ground only. Do not tie pack positive to the ESP32 3.3 V pin.

UART adapter, 115200 8N1:

| Adapter | ESP32 |
| --- | --- |
| TX | GPIO3 (U0RXD) |
| RX | GPIO1 (U0TXD) |
| GND | GND |

`scripts/flash.sh` asks esptool to pulse RTS at the end of the write. The Prolific PL2303 on `/dev/ttyUSB0` only has TX, RX, and ground connected to the module, so that pulse never reaches EN. After the write, press the EN button. GPIO0 stays high for a normal boot. Hold GPIO0 low and pulse EN only to enter the download ROM. A first line `flash read err, 1000` followed by `rst:0x10 (RTCWDT_RTC_RESET)` and the `roomba:` banner is the button bounce; the watchdog boots the image.

## What it does

1. Probes the TCA9548A at `0x70`–`0x77` and selects channel 3.
2. Scans that channel. If `0x0B` does not ACK, it prints a serial error and sends nothing to a gauge.
3. Reads SBS `Voltage()`, name, and the device-type word from `ManufacturerAccess` `0x0001`. `ManufacturerData` `0x23` is logged as text and is not used as the device type.
4. Stops with no security write when the device type is not one of the known bq20z70/75, bq30z55, or bq40z50 ids, when `ManufacturerAccess` repeats the same word for the device-type and firmware commands, when a bq30/bq40 fuse bit is set, when permanent fail is already clear, or when any cell is outside 3000–4250 mV or the four cells differ by more than 250 mV. A failed identification prints a read-only command map (`0x3c`–`0x3f`, `0x51`, `0x53`, `0x54`, `0x5a`, and the `0x44` block result). It does not try the next family's keys.
5. On a bq30 or bq40: unseals, requests full access, sends Permanent Fail Data Reset `0x0029`, resets the gauge if the discharge FET is still off, then sends Seal Device `0x0030`. On a bq20z70/bq20z75: reads `OperationStatus` `0x54`, `SafetyStatus` `0x51`, and `PFStatus` `0x53` directly, unseals with the two-word key, clears permanent fail with the PF key, then sends Seal `0x0020`.
6. Prints a snapshot every 5 s afterwards: stack voltage, current, status, and the pack-pin voltage (`0x5a` on a bq20, `DAStatus1` on a bq40). `Voltage()` is the cell-stack sum, not the output tabs. The pack pin should rise to about 16.4 V.

Keys, from the TI technical references, not from an iRobot dump:

- bq20z70/bq20z75: unseal `0x0414` then `0x3672`. Permanent-fail clear is `0x2673` then `0x1712` (TRM SLUU310). If that leaves PF set, the byte-swapped pair `0x7326` then `0x1217` is sent once. Seal is `0x0020`. No `0x44` block and no `0x0029` are sent to this family.
- bq40z50: unseal `0x0414` then `0x3672`, full access `0xFFFF` then `0xFFFF`, written to `ManufacturerAccess` `0x00`. There is a 4.1 s quiet gap before each pair. If the word write is ignored, the same pair is sent once as a block write to `0x44`.
- bq30z55: SHA-1 unseal `0x0031` and full access `0x0032`. The challenge is read from `0x2F` and the response is written back there. The default key is `0123456789ABCDEFFEDCBA9876543210`.

If iRobot replaced those keys, the log says the key was rejected and the latch is left set. Change the constants in `main/config.h` and rebuild. A key word whose high byte is `0x00` is refused.

The fuse pin is never driven. `0x001D`, `0x0026`, and the bq40 FET test toggles `0x001F` / `0x0020` are not sent. On a bq20z70/bq20z75, `0x0020` is Seal Device and is sent after a PF clear. FET Control `0x0022` is sent only on a bq40, and only when `ManufacturingStatus` `FET_EN` reads as 0.

## Build and flash

On this Gentoo machine the framework is `dev-embedded/esp-idf`, installed at `/usr/share/esp-idf`, with the command `/usr/bin/idf`. `idf.py` is not on `PATH`. The scripts use that command and ignore an `IDF_PATH` that does not contain `tools/idf.py`.

```sh
./scripts/build.sh
./scripts/flash.sh /dev/ttyUSB0
```

`flash.sh` builds if needed, writes the image, and opens the monitor. Leave the monitor with Ctrl-].

Pins, the mux channel, the cell window, and the keys are in `main/config.h`.
