# Raspberry Pi 4 & Pi 5 — Second UART deployment (GPIO 4 / 5)

This document summarizes how a **second hardware UART** is enabled on the **40-pin J8 header** using the same **BCM GPIO numbers** on both **Raspberry Pi 4** (and BCM2711 family boards that match the `[pi4]` filter) and **Raspberry Pi 5** (boards that match `[pi5]`). It reflects the active split in `/boot/firmware/config.txt`.

On **Pi 5**, the same `config.txt` may also enable **`uart0-pi5`** (GPIO 14/15), **`uart3-pi5`** (GPIO 8/9), and **`uart4-pi5`** (GPIO 12/13) alongside **`uart2-pi5`** (GPIO 4/5). Sections **1–2**, **4**, and **6–8** still center on the **GPIO 4 / 5** link; **§3.1** shows both the **minimal** fragment (second UART only) and the **current full `[pi5]`** UART block (including **UART4**) used on this fleet.

---

## 1. Goal

- Keep the **primary** serial UART on the usual **GPIO 14 (TX) / GPIO 15 (RX)** (header pins 8 and 10) as provided by the stock image and `enable_uart=1`.
- Add a **second** full **PL011-class** UART on **GPIO 4 (TX)** and **GPIO 5 (RX)** so firmware and wiring can stay **identical** across Pi 4 and Pi 5 for that link.

---

## 2. Why Pi 4 and Pi 5 use different overlays

| Platform | Device-tree filter | Overlay | SoC UART block on GPIO 4 / 5 |
|----------|-------------------|---------|------------------------------|
| Pi 4B, Pi 400, CM4, CM4S | `[pi4]` | `dtoverlay=uart3` | On BCM2711, **UART3** is muxed to GPIOs 4–7 (TX/RX on 4 and 5 by default). |
| Pi 5, Pi 500, CM5-class (`[pi5]`) | `[pi5]` | `dtoverlay=uart2-pi5` | On BCM2712, the block exposed on **GPIO 4–5** is named **UART2** in the `-pi5` overlay set. |
| Pi 5 (fourth header UART, optional) | `[pi5]` | `dtoverlay=uart4-pi5` | **UART4** on **GPIO 12 (TX) / GPIO 13 (RX)** (J8 pins 32 / 33); typical device **`/dev/ttyAMA4`**. |

**Important:** On Pi 5, `uart3-pi5` is **not** the same pins — it maps to **GPIO 8 / 9**. For the **same wiring as Pi 4’s `uart3`**, Pi 5 must use **`uart2-pi5`**, not `uart3-pi5`.

**UART4 naming:** On Pi 5, **`uart4-pi5`** is the overlay for GPIO 12/13. Do **not** assume Pi 4’s `uart4` overlay uses the same pins — on BCM2711, `dtoverlay=uart4` muxes **GPIO 8–11**, not 12/13.

The `[pi4]` section is **not** evaluated on a Pi 5, and `[pi5]` is **not** evaluated on a Pi 4, so each board loads only its model’s `[pi*]` block (Pi 5 may list several `dtoverlay=` lines in `[pi5]`).

---

## 3. Configuration file

**Path (Raspberry Pi OS Bookworm and later):** `/boot/firmware/config.txt`  

**Note:** `/boot/config.txt` may only redirect you to `/boot/firmware/config.txt` — always edit the firmware path on current images.

### 3.1 Required fragments

**Minimum (second UART on GPIO 4 / 5 only):** Pi 4 uses `uart3`; Pi 5 uses `uart2-pi5` on the same pins.

```ini
# Second UART on same header pins (GPIO 4 = TX, GPIO 5 = RX; J8 pins 7 & 29):
#   Pi 4 / Pi 400 / CM4: overlay uart3 (BCM2711)
#   Pi 5 / Pi 500 / etc.: overlay uart2-pi5 — same GPIOs, different SoC block name
[pi4]
dtoverlay=uart3

[pi5]
dtoverlay=uart2-pi5

[all]
enable_uart=1
```

**Current full Pi 5 UART block** (UART0, UART2, UART3, UART4 on the 40-pin header — as deployed in `/boot/firmware/config.txt` for Pi 5):

```ini
# --- UARTs on the 40-pin GPIO (Pi 5): UART0, UART2, UART3, UART4 ---
# Overlay          /dev (typical)   TX / RX (BCM)   Physical header pins
# uart0-pi5        ttyAMA0          GPIO 14 / 15    pin 8 (TX), pin 10 (RX)
# uart2-pi5        ttyAMA2          GPIO 4  / 5     pin 7 (TX), pin 29 (RX)
# uart3-pi5        ttyAMA3          GPIO 8  / 9     pin 24 (TX), pin 21 (RX)
# uart4-pi5        ttyAMA4          GPIO 12 / 13    pin 32 (TX), pin 33 (RX)
#
# Pi 4: uart3 uses GPIO 4–5; on Pi 5 that mapping is uart2-pi5 (same pins as uart2 above).
[pi4]
dtoverlay=uart3

[pi5]
dtoverlay=uart0-pi5
dtoverlay=uart2-pi5
dtoverlay=uart3-pi5
dtoverlay=uart4-pi5

[all]
enable_uart=1
```

- **`enable_uart=1`** under `[all]` remains required for serial/UART behaviour as documented by Raspberry Pi.
- Do **not** duplicate these overlays outside the correct filter blocks, or a wrong board might try to load an incompatible overlay.
- On Pi 5, omit `uart0-pi5` / `uart3-pi5` / `uart4-pi5` if you only need the GPIO 4/5 port; keep **`uart2-pi5`** for that link.
- **UART4** (`uart4-pi5`) is used in this fleet for NMEA out to the flight controller (typical app device **`/dev/ttyAMA4`**); see companion deploy scripts under `deployscripts/`.

### 3.2 Applying on a new SD card or image

1. Mount the boot partition (or edit the golden image before flash).
2. Merge the `[pi4]` / `[pi5]` / `[all]` lines above with your existing `config.txt` (preserve your other `dtoverlay=` and `dtparam=` entries).
3. Ensure a **`[all]`** section still exists after any model-specific blocks so shared settings apply to every model.

---

## 4. Pinout and wiring (3.3 V TTL)

Use **BCM GPIO** names when reading schematics and software; **J8 pin** is the physical 40-pin header.

| Signal (from the Pi) | BCM GPIO | J8 physical pin | Notes |
|----------------------|----------|-----------------|--------|
| **Primary UART TX** | 14 | 8 | Connect to peripheral **RX**. |
| **Primary UART RX** | 15 | 10 | Connect to peripheral **TX**. |
| **Second UART TX** | 4 | 7 | Connect to second device **RX** (Pi 5: `uart2-pi5` / `ttyAMA2` typical). |
| **Second UART RX** | 5 | 29 | Connect to second device **TX**. |
| **Third UART TX** (Pi 5 only, optional) | 8 | 24 | Only when `dtoverlay=uart3-pi5` is loaded; connect to third device **RX** (`ttyAMA3` typical). |
| **Third UART RX** (Pi 5 only, optional) | 9 | 21 | Connect to third device **TX**. |
| **UART4 TX** (Pi 5 only, optional) | 12 | 32 | Only when `dtoverlay=uart4-pi5` is loaded; connect to fourth device **RX** (`ttyAMA4` typical). |
| **UART4 RX** (Pi 5 only, optional) | 13 | 33 | Connect to fourth device **TX**. |
| **GND** | — | e.g. 6, 9, 14, 20, 25, 30, 34, 39 | Common ground between Pi and each peripheral. |

- Logic level is **3.3 V** only; do not drive 5 V TTL directly into these pins without a level shifter.
- Optional **CTS/RTS** for UART2 and UART4 are available via overlay parameters (`ctsrts`); default is TX/RX only. See `dtoverlay -h uart3` on Pi 4; on Pi 5: `dtoverlay -h uart2-pi5`, `dtoverlay -h uart4-pi5`, etc.

---

## 5. Primary UART on Pi 5 (operational note)

On **Pi 5**, the **primary** Linux console UART is often documented on the **3-pin UART debug** connector rather than on GPIO 14/15. That does **not** change this deployment: the **second** link is still intentionally on **GPIO 4 / 5** (pins 7 and 29) for a consistent accessory connector across Pi 4 and Pi 5.

When **`uart0-pi5`** is enabled, GPIO 14/15 is an explicit PL011 UART on the header (typically **`/dev/ttyAMA0`**); use that node for the “primary” header UART instead of assuming the mini-UART naming from older images.

Always discover the actual **`/dev/*`** node after boot (next section).

---

## 6. Software: finding the serial devices

Overlay names do not map 1:1 to `/dev/ttyAMA*` indices across kernels and Bluetooth configurations.

### 6.1 After every install or major image upgrade

1. **Reboot** after editing `config.txt`.
2. List devices:

   ```bash
   ls -l /dev/serial/by-path 2>/dev/null
   ls /dev/ttyAMA*
   ```

3. Prefer **`/dev/serial/by-path/...`** (stable path by USB/UART topology) in production, or add a **udev rule** if you need a fixed symlink name (e.g. `/dev/serial/by-id/...` when the hardware exposes it).

### 6.2 Quick loopback test (optional)

With **GPIO 4 shorted to GPIO 5** (TX → RX) and a terminal on the **second** port:

```bash
# Replace DEVICE with the discovered ttyAMA* or symlink
stty -F /dev/DEVICE 115200
echo 'test' > /dev/DEVICE && timeout 1 cat /dev/DEVICE
```

(Use a real serial tool such as `minicom`, `picocom`, or Python `pyserial` for interactive tests.)

### 6.3 Per-port loopback test (Pi 5, all header UARTs)

With **TX shorted to RX on each port** (one loopback at a time), run the repo script (defaults include UART4 when listed):

```bash
python3 ~/RL/deployscripts/testuart/testuart.py \
  --ports /dev/ttyAMA0 /dev/ttyAMA2 /dev/ttyAMA3 /dev/ttyAMA4
```

Each line should report **`OK`** with an unchanged echo. Omit ports you have not wired or enabled in `config.txt`.

---

## 7. Deployment checklist

| Step | Action |
|------|--------|
| 1 | Confirm `config.txt` contains the `[pi4]` / `[pi5]` / `[all]` fragment in §3.1 (full Pi 5 four-UART block or minimal GPIO 4/5-only lines, as intended). |
| 2 | Reboot. |
| 3 | Verify overlays applied: `sudo vcdbg log msg 2>&1 | grep -i uart` or inspect kernel log `dmesg \| grep -i tty`. |
| 4 | Identify ports: `ls -l /dev/serial/by-path` and `ls /dev/ttyAMA*` (expect **`ttyAMA4`** when `uart4-pi5` is loaded). |
| 5 | Wire GPIO 4/5 second UART: peripheral RX → Pi **GPIO 4**, peripheral TX → Pi **GPIO 5**, GND common. |
| 6 | If using UART4: peripheral RX → Pi **GPIO 12**, peripheral TX → Pi **GPIO 13**, GND common (pins 32 / 33). |
| 7 | Configure application baud rate, 8N1 unless the device requires otherwise (e.g. 115200 for `testuart.py`). |

---

## 8. Troubleshooting

| Symptom | Things to check |
|---------|------------------|
| No new `ttyAMA*` after reboot | Typo in overlay name; wrong filter (`[pi4]` on a Pi 5 image section missing `[pi5]`); firmware partition not the one edited. |
| Permission denied opening port | User in `dialout` group: `sudo usermod -aG dialout $USER` then re-login. |
| Garbled data on Pi 3–class mini UART | **Not applicable** to this second port on Pi 4/Pi 5 (second link is PL011). If debugging **primary** port on older Pis, see official UART clock notes. |
| `uart3` fails on non–BCM2711 | Expected: `uart3` is BCM2711-only. Pi 5 must use `uart2-pi5` only under `[pi5]`. |
| Pi 5 `uart3-pi5` wired like Pi 4 `uart3` | Wrong: **`uart3-pi5`** is on **GPIO 8 / 9** (pins 24 / 21). GPIO 4 / 5 on Pi 5 is **`uart2-pi5`**. |
| No `ttyAMA4` on Pi 5 | **`uart4-pi5`** missing from `[pi5]` or overlay failed; confirm `dtoverlay -h uart4-pi5` and `dmesg`. |
| UART4 wired to GPIO 8–11 | On Pi 5 use **`uart4-pi5`** (GPIO **12 / 13**). Pi 4 **`uart4`** is a different pinmux (GPIO 8–11). |
| GPIO 4/5 or 12/13 already used | Another overlay or application may claim those pins; resolve pinmux conflict in `config.txt`. |

---

## 9. Official references

- [Raspberry Pi configuration — UARTs and device tree](https://www.raspberrypi.com/documentation/computers/configuration.html#uarts-and-device-trees-overlays-and-parameters)
- [config.txt — model filters `[pi4]`, `[pi5]`](https://www.raspberrypi.com/documentation/computers/config_txt.html#model-filters)
- On a running Pi: `dtoverlay -h uart3` (Pi 4); on Pi 5: `dtoverlay -h uart0-pi5`, `dtoverlay -h uart2-pi5`, `dtoverlay -h uart3-pi5`, and `dtoverlay -h uart4-pi5` as needed. See `/boot/firmware/overlays/README` for full overlay text.

---

## 10. Document control

| Item | Value |
|------|--------|
| Purpose | Deploy / audit second UART on GPIO 4 & 5 for Pi 4 and Pi 5; document optional Pi 5 UART0, UART3, and **UART4** (`uart4-pi5`, GPIO 12/13, `ttyAMA4`) when used |
| Repo / path | `~/RL/deployscripts/RPi_second_UART_GPIO4_GPIO5_deployment.md` |
| Typical boot config | `/boot/firmware/config.txt` |

If this file and the live `config.txt` diverge, treat **`config.txt` on the boot partition** as the source of truth for a given SD card.
