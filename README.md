# Cubiko Pendant

A handheld pendant controller for the [Genmitsu Cubiko](https://genmitsu.com/pages/cubiko) CNC, built on the [M5Stack Dial](https://docs.m5stack.com/en/core/M5Dial). Rotate the wheel to jog, drag-and-drop G-code files over WiFi, and press play — the pendant streams the job straight to grbl over USB.

![The M5Dial pendant plugged into a Cubiko CNC](M5DialCubiko.png)

## What it does

- **Jog** — rotate the encoder to move the currently-selected axis; tap X/Y/Z to switch axis, SLOW/MED/FAST to switch step size. Long-press an axis chip to zero its work coordinate at the current position.
- **File transfer over WiFi** — an FTP server on the pendant accepts G-code uploads. Files auto-arm as the current job; press play to run.
- **Live work coordinates** — the readout shows the actual X/Y/Z work position reported by grbl, not a virtual counter.
- **Calibrate** — one-button homing + tool-length-sensor probe using the Cubiko's built-in gold sensor at machine (149, 110).
- **Unlock** — sends `$X` to clear grbl alarm state after a failed probe or limit hit.
- **Play / pause / stop** — grbl feed hold (`!`), resume (`~`), soft reset (`\x18`) with the appropriate icon state on the button.

## Physical setup

You need exactly one thing: a **USB-C to USB-B cable** (the pendant is USB-C, the Cubiko is USB-B — the same cable that came with your Cubiko).

1. (Optional) 3D-print `hardware/PendantHolder.stl` and clip it near your Cubiko so the pendant has somewhere to live.
2. Plug the USB-B end into the Cubiko and the USB-C end into the M5Dial.
3. Turn on the Cubiko.

The Cubiko provides both **5 V power** and **USB data** through that single cable — no external power supply, no OTG cable, no splitter. The M5Dial's screen turns on immediately.

## Firmware installation

Two paths. The web installer is what to send someone new; the manual path is for people who already have `esptool` and want to script it.

### Option A: Web installer (recommended)

Open **[install.cubiko-pendant](https://jonphoton.github.io/cubiko-pendant/)** in **Chrome or Edge** on a laptop. Firefox and Safari don't have WebSerial yet.

1. Unplug the M5Dial from the Cubiko.
2. Plug it into your laptop's USB-C port **while holding the G0 side button** (this puts the ESP32-S3 into ROM bootloader mode). Release G0.
3. Click **Install** on the page and pick the M5Dial's serial port when prompted.
4. Wait ~30 seconds. When it says "Installation complete", unplug and plug it back into the Cubiko.

### Option B: Manual esptool

If you already have Python:

```
pip install esptool
```

Put the Dial into bootloader mode (unplug, hold G0, plug in, release), then from the `firmware/` folder:

```
esptool.py --chip esp32s3 --port /dev/tty.usbmodem101 write_flash \
    0x0     bootloader.bin \
    0x8000  partitions.bin \
    0x10000 firmware.bin
```

(Port name differs on Linux/Windows — check with `ls /dev/tty.*` on macOS or Device Manager on Windows.)

## First-boot WiFi setup

The pendant needs to know your home WiFi to expose its FTP server. The first time it boots:

1. Screen shows **"Hold G0 to reset WiFi (2s)"**. Ignore this on first boot.
2. Screen shows **"SETUP / Join WiFi: M5DialCubiko-Setup / open any URL"**.
3. On your phone, join the WiFi network named **`M5DialCubiko-Setup`**.
4. A captive portal pops up (or open `http://192.168.4.1/` in a browser). Pick your home network from the list, enter your WiFi password, save.
5. The pendant reboots, joins your network, and the jog screen appears with `FTP <ip>` at the bottom.

**To change networks later** (or if you gave it the wrong password): power-cycle the Dial. When "Hold G0 to reset WiFi" is showing, hold G0 for about a second. It'll wipe the saved credentials and go back to setup mode.

## Uploading G-code

The pendant's IP is shown at the bottom of the jog screen. It's also advertised on your LAN as **`cubiko.local`** via mDNS — so on macOS / recent Windows / Linux you can use the hostname directly.

### From any browser (easiest)

Open **[http://cubiko.local/](http://cubiko.local/)** on any computer or phone on the same WiFi. You get a drag-and-drop page served by the pendant itself. Drop files, done.

No install, no CLI, works from a phone. This is the recommended path for most users.

### From the command line

**With `curl`** (built into macOS and Linux):

```
curl -T job.gcode ftp://cubiko:cubiko@cubiko.local/
```

### Native desktop app

`tools/uploader/cubiko_uploader.py` is a small PyQt6 app if you prefer a native window instead of the browser:

```
uv run tools/uploader/cubiko_uploader.py
```

or with plain pip:

```
pip install -r tools/uploader/requirements.txt
python3 tools/uploader/cubiko_uploader.py
```

## On-device controls

| Control | Action |
|---|---|
| Rotate encoder | Jog the selected axis by one step per detent |
| Tap X / Y / Z | Switch which axis the encoder moves |
| **Long-press X / Y / Z** (1 s) | Zero the work coordinate of that axis at the current position (beep on press, higher beep on confirm) |
| Tap SLOW / MED / FAST | Change jog step size (0.01 / 0.10 / 1.00 mm) |
| Tap CAL | Home the machine + probe the tool-length sensor + zero work-Z at contact |
| Tap UNL | Send `$X` to clear grbl alarm state |
| Tap ■ (stop) | Abort current job (grbl soft reset); file stays loaded to re-run |
| Tap ▶ (play) | Start / pause / resume the current job |
| Green dot next to FTP IP | CNC USB link is up |

## Job flow

Upload → `RDY <file>` → press play → `RUN <line#>` → job completes → `DONE`. Press play again to re-run the same file, stop to clear, or just upload a new file (auto-replaces).

## Hardware you need

| Item | Notes |
|---|---|
| Genmitsu Cubiko CNC | runs grbl 1.1h, CH341 USB-serial |
| [M5Stack Dial](https://shop.m5stack.com/products/m5stack-dial-esp32-s3-smart-rotary-knob-w-1-28-round-touch-screen) | ESP32-S3, 240×240 round touch screen, rotary encoder |
| USB-C to USB-C cable | Any decent one. Power + data through this alone. |
| 3D-printed pendant holder | `hardware/PendantHolder.stl`, optional |

## Building from source

```
pip install platformio
pio run -e m5dial-cnc -t upload
```

`platformio.ini` has two environments:
- `m5dial` — stub CNC transport (logs to USB-CDC serial). Use this for UI iteration without a CNC.
- `m5dial-cnc` — real Ch341 USB-host transport. Use this on the actual pendant.

## License

MIT — see `LICENSE`.

## Credits

Built by [Jon Simon](https://github.com/jonphoton) with a lot of help from Claude Code. Cubiko is a product of [Genmitsu / SainSmart](https://www.sainsmart.com/products/cubiko).
