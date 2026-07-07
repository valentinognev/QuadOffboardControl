# Startup scripts — Raspberry Pi companion

Scripts in this folder start the **CatSwarm companion stack** on a **Raspberry Pi 5** (hardware adapter + system manager + RF RTK) and one-time **LC29H** configuration.

## Layout

| Path | Role |
|------|------|
| **`util/`** | Shared libraries (sourced by launchers; not run directly) |
| **`start_companion_drone_tmux.sh`** | Main companion stack launcher |
| **`startInitRoverPI.sh`** | One-time LC29H rover init (DA UART or EA USB via flags) |
| **`switch_EAUSB_DAUART.sh`** | Restart GPS window; switch EA ↔ DA module |
| **`switch_rtk_WIFI_RF.sh`** | Restart RTK bridge + GPS; switch WiFi ↔ serial RF |
| **`restart_gs_wifi_rtk.sh`** | Ground-station PC: restart WiFi RTK publisher |

### `util/` shared libraries

| File | Purpose |
|------|---------|
| **`gnss_serial_args.sh`** | Unified `BASE_*` / `ROVER_*` env + CLI parsing |
| **`companion_gps_module.sh`** | EA USB vs DA UART rover selection + tmux GPS launcher |
| **`companion_rtk_connection.sh`** | WiFi/LAN vs serial RF RTK path selection |

## Unified serial parameters (`util/gnss_serial_args.sh`)

All launchers in **`startup_scripts/`** and **`GPS_RTK/`** share one naming scheme (sourced from **`util/gnss_serial_args.sh`**):

| Role | Environment variable | CLI flag |
|------|---------------------|----------|
| Base serial | `BASE_PORT` | `--base-port` |
| Base baud | `BASE_BAUD` | `--base-baud` |
| Rover serial | `ROVER_PORT` | `--rover-port` |
| Rover baud | `ROVER_BAUD` | `--rover-baud` |

Legacy names (`BS_PORT`, `DA_PORT`, `EA_PORT`, `COMPANION_EA_PORT`, …) are accepted with a deprecation warning.

Companion GPS state (`~/.config/companion-gps`) stores **`ROVER_PORT`** / **`ROVER_BAUD`** (EA USB) and **`ROVER_PORT_UART`** / **`ROVER_BAUD_UART`** (DA UART).

## `startInitRoverPI.sh`

One-time LC29H setup: RTK rover mode, GGA + 5 Hz NMEA, optional RTK verify from the ground station.

**DA on UART1** (defaults **`/dev/ttyAMA0`**, **115200**):

```bash
~/RL/startup_scripts/startInitRoverPI.sh --phase1
# power-cycle DA
~/RL/startup_scripts/startInitRoverPI.sh --phase2
# power-cycle DA
~/RL/startup_scripts/startInitRoverPI.sh --verify-rtk --rtk-zmq-url=tcp://127.0.0.1:5562
```

**EA on USB** (pass rover port/baud explicitly):

```bash
~/RL/startup_scripts/startInitRoverPI.sh --rover-port=/dev/ttyUSB0 --rover-baud=460800 --phase1
# power-cycle EA
~/RL/startup_scripts/startInitRoverPI.sh --rover-port=/dev/ttyUSB0 --rover-baud=460800 --phase2
```

| Option | Meaning |
|--------|---------|
| **`--phase1`** | Set RTK rover mode + save to flash |
| **`--phase2`** | Enable GGA, 5 Hz, disable noisy NMEA |
| **`--verify-rtk`** | Inject GS RTCM (needs **`--rtk-zmq-url`** or **`--comm-serial=/dev/ttyAMA2`**) |
| **`--rover-port`** / **`--rover-baud`** | Rover serial (DA defaults **`/dev/ttyAMA0`**, **115200**) |

Implementation calls **`~/RL/GPS_RTK/combination/rover_zmq.py`**.

## Operator scripts

Switch saved preferences without tearing down the full tmux session:

```bash
# GPS module (EA USB vs DA UART)
~/RL/startup_scripts/switch_EAUSB_DAUART.sh --ea
~/RL/startup_scripts/switch_EAUSB_DAUART.sh --da

# RTK path (WiFi vs serial RF)
~/RL/startup_scripts/switch_rtk_WIFI_RF.sh --wifi --base-host=192.168.0.43
~/RL/startup_scripts/switch_rtk_WIFI_RF.sh --serial
```

## `start_companion_drone_tmux.sh`

Starts tmux session **`catswarm_sim`** (default) with:

| Window | Contents |
|--------|----------|
| **`hardware_adapter_<id>`** | mavlink_to_ZMQ, zmq_commands_mavlink, comm_to_ZMQ, **ZMQ_to_comm (PY)**, system manager |
| **`gps_rtk`** | **`startRtkCommPI.sh`** — `rover_zmq` + `emulate_gps_to_px4` |

### Pi 5 UART layout (fleet)

| UART | Device | Role |
|------|--------|------|
| UART1 | `/dev/ttyAMA0` | LC29H DA (RTK rover in) |
| UART2 | `/dev/ttyAMA2` | Ground-station comm radio (GS 107-byte frames) |
| UART3 | `/dev/ttyAMA3` | PX4 MAVLink telemetry (`mavlink-server`) |
| UART4 | `/dev/ttyAMA4` | NMEA out to PX4 |

**RTK path (RF, no Wi‑Fi to GS for corrections):**

```
GS PC:  BS → base_zmq → gs_rtk_serial → comm /dev/ttyUSB0
              (RF)
Pi:     /dev/ttyAMA2 → ZMQ_to_comm.py (reassemble) → ZMQ tcp://127.0.0.1:5562
              → rover_zmq.py → /dev/ttyAMA0 (DA)
              → emulate_gps_to_px4.py → /dev/ttyAMA4 (PX4)
```

Ground station must run **`~/RL/GPS_RTK/startRtkCommGS.sh`** (or equivalent). See [GPS_RTK/docs/guide-ground-station-network.md](../GPS_RTK/docs/guide-ground-station-network.md).

### Usage

```bash
~/RL/startup_scripts/start_companion_drone_tmux.sh 1 --serial=/dev/ttyAMA2
```

On Pi 5, **`--serial`** defaults to **`/dev/ttyAMA2`** when omitted.

| Option | Meaning |
|--------|---------|
| **`1`** / **`--drone-id=N`** | Drone index (must match `system_manager/MultiInput/multiSetup.list`) |
| **`--version=PY`** | Python hardware adapter (default; **required** for RF RTK reassembly) |
| **`--serial=DEVICE`** | GS comm radio device (Pi: **`/dev/ttyAMA2`**) |
| **`--session=NAME`** | tmux session (default **`catswarm_sim`**) |

Useful environment:

| Variable | Default | Meaning |
|----------|---------|---------|
| **`COMPANION_RTK_ZMQ_BIND`** | `tcp://127.0.0.1:5562` | Local ZMQ bridge (ZMQ_to_comm PUSH → rover_zmq PULL) |
| **`COMPANION_PYTHON`** | miniconda **`RL`** env if present | Python for ZMQ_to_comm and GPS stack |
| **`CATSWARM_TMUX_SESSION`** | `catswarm_sim` | tmux session name |

Attach: **`tmux attach -t catswarm_sim`**

Stop: **`./start_companion_drone_tmux.sh --kill`**

### Python on the Pi

RF RTK reassembly lives in **`hardware_adapter/python/ZMQ_to_comm.py`** (Python only; the C binary has no GS frame reassembly).

Recommended interpreter: **`/home/pi/miniconda/envs/RL/bin/python`** with **`pyzmq`** and **`pyserial`** installed:

```bash
/home/pi/miniconda/envs/RL/bin/pip install pyzmq pyserial
```

The launcher sets **`PYTHON`** automatically when the miniconda env exists.

## Files to keep in sync (PC ↔ Pi)

When updating RF RTK or companion behaviour, deploy these paths under **`~/RL/`** on the Pi:

| Path | Purpose |
|------|---------|
| `GPS_RTK/combination/{rover_zmq.py, rtk_comm_reassembly.py, gs_command_frame.py, verify_rtk_comm_frame.py}` | Pi-side RTK + frame helpers |
| `GPS_RTK/startRtkCommPI.sh` | GPS tmux launcher (ZMQ bridge mode) |
| `startup_scripts/util/` | Shared GNSS + companion config libraries |
| `startup_scripts/startInitRoverPI.sh` | One-time rover init on Pi |
| `hardware_adapter/python/{ZMQ_to_comm.py, gs_command_frame.py}` | Serial RX + RTK reassembly |
| `hardware_adapter/hardware_adapter_multi.sh` | PY mode + miniconda **`PYBIN`** |
| `startup_scripts/start_companion_drone_tmux.sh` | Companion launcher |
| `startup_scripts/switch_{EAUSB_DAUART,rtk_WIFI_RF}.sh` | Operator restart/switch scripts |
| `GPS_RTK/docs/*.md`, `GPS_RTK/combination/README.md` | Operator docs |

Example from the dev PC (adjust auth):

```bash
export RSYNC_RSH='sshpass -p … ssh'
rsync -avz GPS_RTK/combination/{rover_zmq.py,rtk_comm_reassembly.py,gs_command_frame.py,verify_rtk_comm_frame.py} \
  pi@192.168.0.141:~/RL/GPS_RTK/combination/
rsync -avz GPS_RTK/startRtkCommPI.sh pi@192.168.0.141:~/RL/GPS_RTK/
rsync -avz startup_scripts/util/ pi@192.168.0.141:~/RL/startup_scripts/util/
rsync -avz startup_scripts/{startInitRoverPI.sh,start_companion_drone_tmux.sh,switch_EAUSB_DAUART.sh,switch_rtk_WIFI_RF.sh,README.md} \
  pi@192.168.0.141:~/RL/startup_scripts/
rsync -avz CatSwarm/hardware_adapter/python/{ZMQ_to_comm.py,gs_command_frame.py} \
  pi@192.168.0.141:~/RL/hardware_adapter/python/
rsync -avz CatSwarm/hardware_adapter/hardware_adapter_multi.sh pi@192.168.0.141:~/RL/hardware_adapter/
rsync -avz GPS_RTK/docs/ pi@192.168.0.141:~/RL/GPS_RTK/docs/
rsync -avz GPS_RTK/combination/README.md pi@192.168.0.141:~/RL/GPS_RTK/combination/
```

Verify on the Pi:

```bash
python3 ~/RL/GPS_RTK/combination/verify_rtk_comm_frame.py   # expect PASS
```

## Related docs

- [GPS_RTK/docs/guide-raspberry-pi-rover-px4.md](../GPS_RTK/docs/guide-raspberry-pi-rover-px4.md) — DA + PX4 + RF / Wi‑Fi paths
- [GPS_RTK/docs/guide-ground-station-network.md](../GPS_RTK/docs/guide-ground-station-network.md) — GS PC (`startRtkCommGS.sh`)
- [GPS_RTK/combination/README.md](../GPS_RTK/combination/README.md) — ZMQ scripts and frame format
