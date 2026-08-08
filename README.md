# ESP32 HackRF Upgrade

Connect an **ESP32-S3** to a **HackRF One** over USB OTG to unlock a
Flipper Zero-inspired feature set powered by a real software-defined radio.

## Features

| Feature | Description |
|---------|-------------|
| **Sub-GHz scanner** | Scan 300 / 315 / 433 / 868 / 915 MHz ISM bands for active signals |
| **Signal capture & replay** | Record raw I/Q and retransmit at the push of a button |
| **OOK/ASK transmitter** | Emit custom pulse sequences for garage doors, remote controls, etc. |
| **Spectrum analyser** | ASCII waterfall display of any frequency range |
| **IR transceiver** | Send NEC / Sony SIRC frames; receive arbitrary IR bursts via RMT |
| **BLE scanner** | Passive advertisement scan with RSSI and name decoding |
| **Wi-Fi scanner** | Active 2.4 GHz AP scan with SSID / BSSID / channel / RSSI |
| **Interactive CLI** | Full UART shell with tab-complete and command history |

## Hardware

| Part | Notes |
|------|-------|
| ESP32-S3 dev board | Must have USB OTG port (e.g. ESP32-S3-DevKitC-1) |
| HackRF One | Connected to ESP32-S3 USB OTG port via USB-A to micro-B cable |
| IR LED + receiver | 940 nm IR LED on GPIO 18; TSOP38238 (or similar) on GPIO 19 |
| 5 V supply | HackRF draws up to 500 mA; use a powered hub or external 5 V rail |

## Repository Layout

```
firmware/
├── CMakeLists.txt              Top-level ESP-IDF project file
├── sdkconfig.defaults          Recommended build configuration
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                  Application entry point
│   ├── spectrum_analyzer.[ch]  FFT-free spectrum display
│   └── signal_replayer.[ch]    Multi-slot signal library
└── components/
    ├── hackrf_usb/             USB host driver for HackRF One
    ├── subghz/                 Sub-GHz RX/TX + OOK encoder
    ├── ir_transceiver/         RMT-based IR TX/RX + NEC/SIRC builders
    ├── ble_wifi_scanner/       BLE GAP + Wi-Fi scan
    └── cli/                    ESP-IDF console REPL with all commands
```

## Building

Prerequisites: [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)

```bash
cd firmware
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## CLI Commands

Connect at **115200 baud** and type `help` to list all commands.

```
hackrf> freq 433920000          # tune to 433.92 MHz
hackrf> samplerate 2000000      # 2 Msps
hackrf> lna 24                  # LNA gain 24 dB
hackrf> vga 20                  # VGA gain 20 dB
hackrf> amp 1                   # enable RF amplifier

hackrf> subghz_scan             # scan all ISM bands (500 ms dwell)
hackrf> subghz_rx 433920000     # capture signal at 433.92 MHz (3 s)
hackrf> subghz_replay           # retransmit last captured signal

hackrf> ir_send nec 0x01 0x10   # NEC IR: address=1, command=16
hackrf> ir_send sony 0x48 0x01  # Sony SIRC: command=0x48, address=1

hackrf> ble_scan 5000           # BLE scan for 5 seconds
hackrf> wifi_scan               # Wi-Fi AP scan

hackrf> spectrum 433920000 2000000   # spectrum at 433.92 MHz ±1 MHz
```

## Architecture

```
app_main()
  ├── hackrf_usb_init()      ← enumerate HackRF via USB Host Library
  ├── subghz_init()          ← prepare sub-GHz state machine
  ├── ir_transceiver_init()  ← configure RMT TX/RX channels
  ├── ble_wifi_scanner_init() ← start BLE controller + Wi-Fi station
  ├── spectrum_analyzer_init()
  ├── signal_replayer_init()
  └── cli_start()            ← launch REPL (blocks in its own task)
```

## Legal

Radio transmission is regulated in every jurisdiction.
**Always verify you are legally permitted to transmit on any frequency
before using the Sub-GHz or signal-replay features.**
The HackRF One is a wideband SDR; use it responsibly.

## License

MIT – see [LICENSE](LICENSE).
