<p align="center">
  <img src="img/ServosAreGoLogo.png" alt="ServosAreGo" width="600">
</p>

# ServosAreGo

[日本語](README.md) | English

A tool for ESP32 to **change the ID and baud rate of, and test**, serial servos from many makers and PWM servos.
Everything from flashing the firmware to operating the servos is done in the browser.

Supported: PWM / Feetech STS・SCS / Dynamixel / Futaba TTL / JR XBUS / Hitec HSR / Kondo KRS

<p align="center">
  <img src="img/sag_1.png" alt="ServosAreGo browser version" width="900">
</p>

## Using the browser version

### 1. Flash the firmware

1. Download [`web/ServosAreGo.html`](web/ServosAreGo.html) from this branch and open it in **Chrome** on a PC (nothing to install; the firmware is included in the HTML).
2. Connect the ESP32 (Meridian Board) to the PC by USB, press "Connect" in **(1) Connect**, and select the port.
3. Press "Flash" in **(2) Firmware** (about 30 s).
   If it does not start, hold the BOOT button on the board, press "Flash", and release it when writing starts.

It is done when "Board" shows the same version as "This page". Next time, just connect.
If the board firmware is old, the page shows "Please update the firmware." Press "Flash" to update it.

> [!TIP]
> If no port appears in the list, the page shows how to install the driver. If you cannot connect, make sure the **USB cable supports data** (charge-only cables do not work).

### 2. Connect the servos (with Meridian Board)

Connect the servos to the **L line** of the Meridian Board and turn on a power supply that matches the servos' rated voltage.

> [!WARNING]
> **Mind the signal voltage.** The Meridian Board signal line is **5 V**.
> - Connect **servos that accept a 5 V signal**, such as Kondo, Futaba and PWM servos.
> - **Do not connect 3.3 V signal servos directly to the Meridian Board.** A level shifter to 3.3 V is required.

### 3. Operate

| | Panel | What to do |
|:-:|---|---|
| 3 | **Servo type** | Set Wiring to "(1) Half-duplex circuit (Meridian Board etc.)". Choose the type and press "Select", or press "Auto detect" if you are not sure (read-only, servos do not move) |
| 4 | **Find servos** | Press "Scan" to find IDs. Servos found are set as the Target ID and added to the Sync list |
| 5 | **Move** | "TX test" sweeps all IDs ±30° (works even if replies cannot be read). Use the slider to send an angle |
| 6 | **Settings** | Change the ID and baud rate. **Connect only one servo when changing the ID** |

If no servo is found, choose the maker in the drop-down list, then press "Auto detect".

The **Terminal** at the bottom of the page also accepts the text menu below.

---

## Supported servos

| No. | Type | Main models | Factory baud rate | ID |
|:-:|---|---|---|---|
| 1 | PWM servo | Common PWM servos (50 Hz, no reply) | — | none |
| 2 | Feetech STS | STS3215 etc. (STS / SMS series) | 1 Mbps | 0–253 |
| 3 | Feetech SCS | SCS0009 etc. | 1 Mbps | 0–253 |
| 4 | Dynamixel | X series etc., Protocol 2.0 | 57600 bps | 0–252 |
| 5 | Futaba TTL | RS30x such as RS303MR, RS304MD | 115200 bps | 1–127 |
| 6 | JR XBUS | NX / NXB series etc. (3.3 V signal, level shifter recommended) | 250000 bps fixed | 1–50 |
| 7 | Hitec HSR | HSR-8498HB (HMI) | 19200 bps fixed | 0–127 |
| 8 | Kondo KRS | KRS series (ICS3.5 / 3.6) | 115200 bps | 0–31 |

TTL-level one-wire servos are supported (RS-485 versions are not).

## Text menu

Use it in the browser Terminal, or in the Arduino IDE / PlatformIO serial monitor (**115200 bps, with line ending**).

On startup the type menu appears. Enter a number and press Return (`0` for auto detect, `w` to switch wiring).
The chosen type is saved; next time it starts with the previous type after 5 seconds.

After choosing a type, enter a one-letter command and press Return (Return alone shows the menu).
The keys are the same for all types.

| Key | Function | Key | Function |
|---|---|---|---|
| `f` | Find servos | `i` | Change ID |
| `z` | Set target ID | `b` | Change baud rate |
| `n` | Set sync ID list | `m` | Read version / model |
| `r` / `a` | Read position (one / all) | `o` | Set rotation direction |
| `v` | Read voltage etc. | `q` | Factory reset |
| `x` | Send angle | `p` / `s` | Read / write registers etc. |
| `c` | Center | `t` | TX test (sweeps all IDs, no reply needed) |
| `e` / `d` | Torque ON / release | `h` | Choose servo type again (restart) |

Some keys are not available, or only available, for certain types. See the menu.

## Notes

- **Match the signal voltage.** Meridian Board is 5 V, a bare ESP32 is 3.3 V. A mismatched servo may break or not be detected.
- **When Feetech is selected, IDs 1–6 move to center at startup.** Be careful: the servos move suddenly.
- ID, baud rate and EEPROM changes are saved in the servo. Connect **only one** servo when changing the ID.
- **Hitec** reads and writes without an ID, so always connect **only one** servo.
- Auto detect checks IDs 0–31 (Kondo), 0–20 (Feetech / Dynamixel) and 1–20 (Futaba / XBUS). For other IDs, choose the type and increase the Max ID for scanning.
- PWM auto detect runs only **after asking**, when no serial servo is found (XBUS servos and some others stay in PWM mode until power-cycled once they receive PWM).

## Troubleshooting

| Problem | What to do |
|---|---|
| No port appears on "Connect" | Make sure the USB cable supports data (charge-only cables do not work). On Windows, install the driver: [Silicon Labs](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers) for CP210x, [WCH CH341SER](https://www.wch-ic.com/downloads/CH341SER_EXE.html) for CH340, [WCH CH343SER](https://www.wch-ic.com/downloads/CH343SER_EXE.html) for CH9102. On Mac, usually not needed ([WCH](https://www.wch-ic.com/downloads/CH34XSER_MAC_ZIP.html) if required). On Ubuntu, run `sudo usermod -a -G dialout $USER` and log in again. Close Arduino IDE etc. if it uses the port |
| Auto detect finds nothing | Check wiring and power. Choose the type and check whether "TX test" moves the servo |
| TX test moves the servo but Scan finds nothing | A receive or baud rate problem. Press "Auto detect" with the type selected to check that type at all baud rates |
| TX test does not move the servo | Check power, the signal line, the signal voltage (5 V / 3.3 V) and the wiring setting ((1)/(2)). Power-cycle the servo |

---

## Extra: using a bare ESP32

You can connect with only an ESP32, without a half-duplex circuit.

| ESP32 | Servo |
|---|---|
| **Both** GPIO 32 (RX) and GPIO 27 (TX) | Signal |
| GND | GND (shared with servo power) |

In the browser version, set Wiring to **"(2) No circuit (RX + TX direct)"** (`w` in the text menu).
The rest is the same as with the Meridian Board.

> [!WARNING]
> The ESP32 signal is **3.3 V**. **Do not connect 5 V signal servos directly** (it exceeds the ESP32 input limit).
> For 5 V servos, add a level shifter or use the Meridian Board.

While receiving, TX output is released and the RX internal pull-up holds the signal line HIGH.

---

## For developers

The firmware is a PlatformIO project (`ServosAreGo/`).

```bash
cd ServosAreGo
pio run -t upload    # build and flash
pio device monitor   # serial monitor (115200 bps)
```

The browser HTML is built by embedding the built firmware.

```bash
cd web
python3 build_html.py            # run pio run, then build web/ServosAreGo.html
python3 build_html.py --no-build # build with the existing build output
```

```
ServosAreGo/
├── ServosAreGo/          Firmware (PlatformIO)
│   ├── platformio.ini    Pin settings (TX1=27, RX1=32)
│   └── src/
│       ├── main.cpp      Type selection, auto detect, browser commands (@...)
│       ├── servo_bus.h   Half-duplex servo bus (shared by wiring (1)/(2))
│       └── setter_*.cpp  Per-type handling
├── web/
│   ├── ServosAreGo.html  Browser version (open this)
│   ├── src/index.html    Browser version source
│   └── build_html.py     Script that builds the browser version
└── img/
```

## Disclaimer

This code is provided for testing and learning. Test it thoroughly before real use.
Changing the ID, baud rate or EEPROM directly affects servo behavior. The author is not responsible for servo damage or unexpected behavior.

## License

MIT License ([LICENSE](LICENSE)). The browser version includes [esptool-js](web/vendor/esptool-js.LICENSE).
