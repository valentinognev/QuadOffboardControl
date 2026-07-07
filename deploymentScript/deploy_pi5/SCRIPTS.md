# deploy_pi5 — script reference

CatSwarm Raspberry Pi 5 companion deployment tooling. Scripts live under `~/deploy_pi5/` on the Pi (or `general_infrastructure/deploymentScript/deploy_pi5/` on the dev PC after sync).

All shell scripts support `-h`, `--help`, or `help`.

---

## Quick start

| Goal | Command |
|------|---------|
| Full Pi deploy (online) | `sudo ~/deploy_pi5/deploy_pi5_companion.sh` |
| Full Pi deploy (offline, pull from dev PC) | `sudo ~/deploy_pi5/deploy_pi5_companion.sh --from-dev=valentin@192.168.0.39` |
| Install companion at boot | `sudo ~/deploy_pi5/install-companion-boot.sh 3` |
| Start companion manually | `~/RL/startup_scripts/start_companion_drone_tmux.sh 3` |
| Install missing Python pkg (Pi offline) | `~/deploy_pi5/pull-offline-packages.sh --from=valentin@192.168.0.39 matplotlib --install` |
| Push scripts Pi → dev PC | `~/deploy_pi5/push-deploy-scripts-to-dev.sh valentin@192.168.0.39` |

---

## Root scripts

### `deploy_pi5_companion.sh`

**Main deployment script.** Run on the Pi as root (`sudo`). Installs OS packages, Pi 5 UART boot overlays, Miniconda env `RL`, mavlink-server (systemd), clones or syncs Git repos under `~/RL`, builds `hardware_adapter`, verifies `~/RL/startup_scripts`, and runs basic verification.

**Phases** (`--phase=PHASE`):

| Phase | What it does |
|-------|----------------|
| `all` | system → repos → mavlink → python → build → verify |
| `system` | apt packages, `dialout` group, `/boot/firmware/config.txt` UART overlays |
| `repos` | git clone/pull, `--from-dev` / `--push-to` rsync (incl. `startup_scripts`), or verify existing `~/RL/startup_scripts` |
| `mavlink` | mavlink-server binary, config template, systemd unit |
| `python` | Miniconda + `RL` env (pyzmq, pyserial, pymavlink, matplotlib, torch, …) |
| `build` | `make` in `hardware_adapter` (+ optional C++ system manager) |
| `verify` | import checks, optional frame verify, service status |

**Notable options:**

- `--with-ml` — also install opencv and plotly (torch is always installed)
- `--with-sim` — clone `QuadOffboardControl` → `~/RL/general_infrastructure`
- `--from-dev[=USER@IP]` — on Pi: rsync project trees from dev PC over SSH
- `--pip-via-dev[=USER@IP]` — fetch pip wheels via dev PC (auto-enabled with `--from-dev`)
- `--push-to[=USER@IP]` — on dev PC: rsync local `~/RL` trees to the Pi
- `--run-remote` — with `--push-to`: SSH to Pi and run this script (`--skip-clone`)
- `--skip-clone` — skip git and startup_scripts sync
- `--git-https` / `--git-ssh` — force clone transport

**Environment:** `RL_ROOT`, `CONDA_ENV`, `TORCH_VERSION`, `MAVLINK_SERVER_VERSION`, `DEV_PC_PASSWORD`

---

### `install-companion-boot.sh`

**Install companion stack at boot** via systemd `companion-drone.service`. Requires `sudo` and a **drone ID** argument (positive integer matching `system_manager/MultiInput/multiSetup.list`).

```bash
sudo ~/deploy_pi5/install-companion-boot.sh 3
```

**Installs:**

- `/etc/default/companion-drone` — `COMPANION_DRONE_ID`, conda paths, UART wait time
- `/usr/local/bin/run-companion-drone.sh`
- `/usr/local/lib/companion-conda-env.sh`
- `/usr/local/bin/companion-run-in-rl.sh`
- `/etc/systemd/system/companion-drone.service`

**Logs:** `journalctl -u companion-drone -f`

---

### `uninstall-companion-boot.sh`

**Remove companion from boot.** Without sudo: removes `@reboot` crontab lines for the current user. With sudo: also disables/removes `companion-drone.service` and installed companion binaries. Does not stop a running tmux session or delete `/etc/default/companion-drone`.

```bash
~/deploy_pi5/uninstall-companion-boot.sh
sudo ~/deploy_pi5/uninstall-companion-boot.sh
```

---

### `mavlink-server-configuration.sh`

**Interactive mavlink-server setup** for Pi 5 companions. Writes `/etc/mavlink-server/mavlink-server.conf` from prompts or fleet preset, optionally restarts `mavlink-server.service`.

```bash
sudo ~/deploy_pi5/mavlink-server-configuration.sh --fleet-preset
```

**Fleet defaults:** serial `/dev/ttyAMA3` @ 921600, UDP client `127.0.0.1:14540`, TCP server `0.0.0.0:5760`, web UI `0.0.0.0:8080`.

---

### `pull-offline-packages.sh`

**Run on the Pi (no internet).** SSH to a dev PC that has internet, download aarch64 Python wheels there, copy them to `~/deploy_pi5/offline-wheels/`, optionally install into conda env `RL`.

```bash
~/deploy_pi5/pull-offline-packages.sh --from=valentin@192.168.0.39 matplotlib --install
~/deploy_pi5/pull-offline-packages.sh --from=valentin@192.168.0.39 torch plotly --install
```

Uses SSH ControlMaster so the password is entered once. Set `DEV_PC_PASSWORD` for non-interactive use with `sshpass`.

---

### `push-offline-packages.sh`

**Run on the dev PC (with internet).** Downloads wheels for Pi (`manylinux2014_aarch64`, Python 3.11), rsyncs to the Pi, optionally runs remote install.

```bash
~/deploy_pi5/push-offline-packages.sh pi@rlcat3 matplotlib --install
```

---

### `offline-install-packages.sh`

**Run on the Pi.** Install packages from local wheels in `~/deploy_pi5/offline-wheels/` (no network). Skips packages already importable. Default package: `matplotlib`.

```bash
~/deploy_pi5/offline-install-packages.sh matplotlib
~/deploy_pi5/offline-install-packages.sh torch
```

---

### `offline-install-matplotlib.sh`

**Backward-compatible wrapper** around `offline-install-packages.sh matplotlib`. Prefer `offline-install-packages.sh` for new use.

---

### `offline-wheels-common.sh`

**Shared library** for offline pip workflows. Sourced by `pull-offline-packages.sh` and `push-offline-packages.sh`. Can also be run directly:

```bash
OFFLINE_WHEELS_DIR=~/deploy_pi5/offline-wheels ./offline-wheels-common.sh download matplotlib
```

**Environment:** `PI_WHEEL_PLATFORM` (default `manylinux2014_aarch64`), `PI_WHEEL_PYTHON` (default `3.11`), `OFFLINE_WHEELS_DIR`

---

### `push-deploy-scripts-to-dev.sh`

**Run on the Pi.** Push `deploy_pi5/` and `~/RL/startup_scripts/` to the dev PC (opposite of `--from-dev`). Excludes `offline-wheels/` by default.

```bash
~/deploy_pi5/push-deploy-scripts-to-dev.sh valentin@192.168.0.39
```

**Default remote paths (dev PC):**
- `deploy_pi5` → `/home/valentin/RL/CatSwarm/general_infrastructure/deploymentScript/deploy_pi5/`
- `startup_scripts` → `.../deploymentScript/startup_scripts/`

**Environment:** `DEV_PC_PASSWORD`, `DEPLOY_REMOTE_PATH`, `DEPLOY_REMOTE_STARTUP_SCRIPTS`, `DEPLOY_SOURCE_DIR`, `RL_ROOT`

---

## `files/` — installed helpers

These are copied to system paths by `install-companion-boot.sh` and used by `deploy_pi5_companion.sh` for mavlink-server.

### `files/run-companion-drone.sh`

**Boot wrapper** invoked by `companion-drone.service`. Reads `/etc/default/companion-drone` (or `~/.config/companion-drone`), activates conda, waits for UART devices, then runs `~/RL/startup_scripts/start_companion_drone_tmux.sh <drone_id>`. Skips if tmux session `catswarm_sim` is already running.

---

### `files/companion-conda-env.sh`

**Conda activation helper** — must be **sourced**, not executed. Activates Miniconda env `RL`, sets `PYTHON`, `COMPANION_PYTHON`, `COMPANION_RUN_IN_RL`, and defines `companion_wrap_rl_cmd()` for tmux pane commands.

---

### `files/companion-run-in-rl.sh`

**Run any command inside conda env `RL`.** Used by tmux panes because tmux defaults to `/bin/sh`, which cannot `source` conda. Wraps commands in `bash -lc 'source conda.sh && conda activate RL && …'`.

```bash
companion-run-in-rl.sh 'python -c "import zmq"'
```

---

### `files/run-mavlink-server.sh`

**mavlink-server launcher.** Parses TOML config (default `/etc/mavlink-server/mavlink-server.conf`) into CLI arguments and execs `/usr/bin/mavlink-server`. Used by the `mavlink-server` systemd unit.

---

### `files/mavlink-server.conf`

**TOML config template** for Pi 5 fleet: PX4 on `/dev/ttyAMA3`, UDP bridge `127.0.0.1:14540`, QGC TCP `0.0.0.0:5760`, web UI `0.0.0.0:8080`. Copied/customized by `mavlink-server-configuration.sh` and `deploy_pi5_companion.sh`.

---

## `~/RL/startup_scripts/` — companion launchers (RL tree)

Canonical on the Pi: `~/RL/startup_scripts/`. On the dev PC: `~/RL/CatSwarm/general_infrastructure/deploymentScript/startup_scripts/`. Synced by `deploy_pi5_companion.sh --from-dev` / `--push-to`, and by `push-deploy-scripts-to-dev.sh` (Pi → dev). See `~/RL/startup_scripts/README.md`.

### `start_companion_drone_tmux.sh`

**Main companion launcher.** Starts tmux session `catswarm_sim` with:

| Window | Contents |
|--------|----------|
| `hardware_adapter_<id>` | mavlink_to_ZMQ, zmq_commands_mavlink, comm_to_ZMQ, ZMQ_to_comm (PY), system manager |
| `gps_rtk` | `startRtkCommPI.sh` — rover_zmq + emulate_gps_to_px4 |

```bash
~/RL/startup_scripts/start_companion_drone_tmux.sh 3
~/RL/startup_scripts/start_companion_drone_tmux.sh --kill
```

**Pi 5 UART layout:** UART1 `/dev/ttyAMA0` (LC29H DA), UART2 `/dev/ttyAMA2` (GS radio), UART3 `/dev/ttyAMA3` (PX4 MAVLink), UART4 `/dev/ttyAMA4` (NMEA to PX4).

---

### `startInitRoverPI.sh`

**One-time LC29H DA configuration** on UART1 (`/dev/ttyAMA0`): RTK rover mode, GGA + 5 Hz NMEA, optional RTK verify from ground station.

```bash
~/RL/startup_scripts/startInitRoverPI.sh --phase1
# power-cycle DA
~/RL/startup_scripts/startInitRoverPI.sh --phase2
~/RL/startup_scripts/startInitRoverPI.sh --verify-rtk --rtk-zmq-url=tcp://127.0.0.1:5562
```

### Operator scripts

| Script | Purpose |
|--------|---------|
| `switch_EAUSB_DAUART.sh` | Switch EA USB ↔ DA UART rover module |
| `switch_rtk_WIFI_RF.sh` | Switch WiFi ↔ serial RF RTK path |
| `restart_gs_wifi_rtk.sh` | Ground-station PC: restart WiFi RTK publisher |

---

## Other directories

### `offline-wheels/`

Local cache of `.whl` files for offline pip install. Populated by `pull-offline-packages.sh`, `push-offline-packages.sh`, or `offline-wheels-common.sh download`. Not synced by `push-deploy-scripts-to-dev.sh` (too large; regenerate on dev PC).

---

## Typical workflows

### Fresh Pi (offline network)

```bash
sudo ~/deploy_pi5/deploy_pi5_companion.sh --from-dev=valentin@192.168.0.39
sudo ~/deploy_pi5/install-companion-boot.sh 3
```

### Add Python package on offline Pi

```bash
~/deploy_pi5/pull-offline-packages.sh --from=valentin@192.168.0.39 torch --install
```

### Sync scripts back to dev PC repo

```bash
~/deploy_pi5/push-deploy-scripts-to-dev.sh valentin@192.168.0.39
```

### Remove boot autostart

```bash
sudo ~/deploy_pi5/uninstall-companion-boot.sh
```
