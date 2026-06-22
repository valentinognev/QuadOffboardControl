# Deploy scripts — Raspberry Pi companion

Scripts in this folder start the **CatSwarm companion stack** on a **Raspberry Pi 5** (hardware adapter + system manager + RF RTK) and one-time **LC29H DA** configuration.

## `startInitRoverPI.sh`

One-time LC29H DA setup on **`/dev/ttyAMA0`** (UART1): RTK rover mode, GGA + 5 Hz NMEA, optional RTK verify from the ground station.

```bash
~/RL/deployscripts/startInitRoverPI.sh --phase1
# power-cycle DA
~/RL/deployscripts/startInitRoverPI.sh --phase2
# power-cycle DA
~/RL/deployscripts/startInitRoverPI.sh --verify-rtk --rtk-zmq-url=tcp://127.0.0.1:5562
```

| Option | Meaning |
|--------|---------|
| **`--phase1`** | Set RTK rover mode + save to flash |
| **`--phase2`** | Enable GGA, 5 Hz, disable noisy NMEA |
| **`--verify-rtk`** | Inject GS RTCM (needs **`--rtk-zmq-url`** or **`--comm-serial=/dev/ttyAMA2`**) |
| **`--da-port`** / **`--da-baud`** | DA UART (defaults **`/dev/ttyAMA0`**, **115200**) |

Implementation calls **`~/RL/GPS_RTK/combination/rover_zmq.py`**. A wrapper at **`GPS_RTK/startInitRoverPI.sh`** forwards here for backward compatibility.

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
~/RL/deployscripts/start_companion_drone_tmux.sh 1 --serial=/dev/ttyAMA2
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
| `deployscripts/startInitRoverPI.sh` | One-time DA init on Pi (`/dev/ttyAMA0`, RF verify) |
| `hardware_adapter/python/{ZMQ_to_comm.py, gs_command_frame.py}` | Serial RX + RTK reassembly |
| `hardware_adapter/hardware_adapter_multi.sh` | PY mode + miniconda **`PYBIN`** |
| `deployscripts/start_companion_drone_tmux.sh` | This companion launcher |
| `GPS_RTK/docs/*.md`, `GPS_RTK/combination/README.md` | Operator docs |

Example from the dev PC (adjust auth):

```bash
export RSYNC_RSH='sshpass -p … ssh'
rsync -avz GPS_RTK/combination/{rover_zmq.py,rtk_comm_reassembly.py,gs_command_frame.py,verify_rtk_comm_frame.py} \
  pi@192.168.0.141:~/RL/GPS_RTK/combination/
rsync -avz GPS_RTK/startRtkCommPI.sh pi@192.168.0.141:~/RL/GPS_RTK/
rsync -avz deployscripts/startInitRoverPI.sh deployscripts/start_companion_drone_tmux.sh deployscripts/README.md \
  pi@192.168.0.141:~/RL/deployscripts/
rsync -avz CatSwarm/hardware_adapter/python/{ZMQ_to_comm.py,gs_command_frame.py} \
  pi@192.168.0.141:~/RL/hardware_adapter/python/
rsync -avz CatSwarm/hardware_adapter/hardware_adapter_multi.sh pi@192.168.0.141:~/RL/hardware_adapter/
rsync -avz GPS_RTK/startInitRoverPI.sh pi@192.168.0.141:~/RL/GPS_RTK/
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
