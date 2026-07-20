# Project Updates

This file documents the development progress and changes made to the `CatSwarm/general_infrastructure` project by the AI agent.

## [2026-07-20] Deploy: skip install when SystemManagerMain already in place
- CMake writes `SystemManagerMain` to `~/RL/system_manager/`; `install` of same path failed. Now chown/chmod in place when built==dest.

## [2026-07-20] Deploy: companion apt package set + offline-debs
- Canonical lists: `COMPANION_APT_SYSTEM` / `COMPANION_APT_BUILD` (`libeigen3-dev`, `libzmq3-dev`, cmake, …) / optional `libcppzmq-dev`.
- `phase_build` always `ensure_companion_build_packages` (Update skips `phase_system`).
- Offline fallback: `offline-debs/` + `fetch-offline-debs.sh`; OB rsyncs debs with deploy scripts.

## [2026-07-20] Deploy build: install Eigen3 for SystemManagerCPP
- Soft Update `phase_build` failed on Pis without Eigen (`find_package(Eigen3)`).
- `phase_system` installs `libeigen3-dev`; `phase_build` installs it only if missing (offline-safe). `libcppzmq-dev` optional (vendored header).

## [2026-07-20] Deploy/boot: ensure PX4 NMEA UART0 before reboot
- `ensure_companion_uart_ports.sh` + `companion_gps_ensure_ports`: migrate saved `COMPANION_PX4_GPS_PORT` **ttyAMA4→ttyAMA0**, keep DA off UART0, validate tty nodes.
- Runs from companion boot wrapper, deploy `phase_verify`, and ObservationBoard `deploy_pi5` after update (before reboot).

## [2026-07-20] Pi NMEA→PX4 on UART0 + EA 10 Hz
- `startup_scripts`: NMEA→PX4 default **`/dev/ttyAMA0`** (GPIO14 TX); optional DA UART default **`/dev/ttyAMA4`**; rover init phase-2 docs **10 Hz**.
- UART tables in `startup_scripts/README.md` and `deploy_pi5/SCRIPTS.md` updated (EA USB fleet path).

## [2026-01-24] MAVLink Routing & Compatibility Fixes
- **Dynamic Port Configuration**:
    - Updated all flight command and mode transition logic to support shared (14550) and individual (14540+i / 14030+i) MAVLink ports.
- **`multidrone/mavlinkTakeoffLandAlt.py` & `multidrone/mavlinkSwitchToMode.py`**:
    - **SysID Targeting Fix**: Scripts now correctly initialize `master.target_system` and `master.target_component` by waiting for a heartbeat from the specific target SysID. 
    - **Rationale**: Essential for shared MAVLink ports (14550) where multiple drones transmit on the same link with different IDs.

## [2026-01-23] MAVLink Script Optimizations
- **`multidrone/mavlinkTakeoffLandAlt.py`**:
    - **Optimized Feedback Loop**: Replaced fixed 15s `time.sleep()` calls with a high-frequency **telemetry polling loop**.
    - **Early Exit**: Script now exits as soon as the drone reaches target altitude or state, reducing UI waiting time by 60-80%.
- **`multidrone/mavlinkSwitchToMode.py`**:
    - **Authoritative Status Reporting**: Prioritizes `master.flightmode` strings from telemetry over manual mapping for 100% accuracy.
    - **Mode 0 Mapping**: Added explicit mapping for raw mode `0` (MANUAL_0) to prevent "UNKNOWN" status reports for uninitialized drones.

## [2026-01-23] MAVLink Mode Switcher (Initial)
- **`mavlinkSwitchToMode.py`**:
    - Created a standalone Python script to switch drone flight modes via MAVLink UDP.
    - Updated to support numerical mode mapping (reliable verification) and `--status` flag.
    - Added `--sysid` for filtering drones on shared ports.
- **`drone_control.sh`**:
    - Created a convenience wrapper script.
    - Automatically maps `--index=N` to System ID `N+1` and targets port 14550.
    - Usage: `./drone_control.sh --index=1 --mode=OFFBOARD`

## [2026-01-23] Multi-Drone Hardware Adapter Fix
- **`hardware_adapter/hardware_adapter_multi.sh`**:
    - **Fixed Command Uplink**: Configured `zmq_commands_mavlink` to use Client Mode, sending directly to the drone's listening port (`127.0.0.1:14580+i`).
    - **Added Target System ID**: Now identifying the target drone by passing `--target-system $((i+1))` to the command bridge.
    - **Rationale**: This resolves the issue where the command bridge would not connect or would send commands to the wrong System ID in multi-drone simulations (where drones are assigned IDs starting from 2) because it wasn't receiving heartbeats to "learn" the configuration.

## [2026-01-22] MAVLink Altitude Control Fix
- **`multidrone/mavlinkTakeoffLandAlt.py`**:
    - Fixed altitude command failure where drone would not reach target.
    - Implemented closed-loop control: monitors altitude in real-time.
    - Added robust message reception (blocking wait) to prevent silent loops.
    - Implemented smart early exit: returns immediately if already at target.
    - Increased pre-OFFBOARD setpoints (10 -> 20) to ensure mode switch.
    - Extended control timeout (15s -> 30s) with early exit on success.

## [2026-01-21] ZMQ Takeoff/Land Debugging
- **`multidrone/zmqTakeoffLandAlt.py`**:
    - Updated default ZMQ ports: `7700` (commands out) and `9900` (state in).
    - Renamed arguments for clarity: `--zmqcmd` and `--zmqstate` (with legacy support).
    - Improved command reliability with ZMQ `LINGER` and post-send `time.sleep(1.0)`.
- **`multidrone/run_multidrone_bridges.sh`**:
    - Swapped ZMQ ports to match Python script defaults (`MAVLINK_TO_ZMQ`=9900, `ZMQ_COMMANDS_MAVLINK`=7700).

## [2026-01-21] OFFBOARD Altitude Control
- **`multidrone/zmqTakeoffLandAlt.py`**:
    - Implemented 10Hz feedback control loop for altitude changes using OFFBOARD mode.
    - Sends `quadPosNedCmd` (position setpoints) preserving X/Y while changing Z.
    - Sends `quadModeCmd` to switch to OFFBOARD before control, HOLD after reaching target.
    - Renamed `--zmq=` to `--zmqout=`, added `--zmqin=` for drone state subscription.
    - Added `--threshold=` and `--time=` for control tuning.
    - Usage: `--altitude=X --zmqin=PORT --zmqout=PORT`
- **`hardware_adapter/include/command_queue.h`**:
    - Added `CMD_TYPE_POS_NED` and `CMD_TYPE_MODE` command types.
    - Added mode ID constants: `MODE_ID_OFFBOARD`, `MODE_ID_HOLD`, `MODE_ID_LOITER`.
- **`hardware_adapter/include/zmq_topics.h`**:
    - Added `quadPosNedCmd` (position setpoint) and `quadModeCmd` (mode switch) topics.
- **`hardware_adapter/src/zmq_commands_mavlink.c`**:
    - Added handlers for `quadPosNedCmd` → `SET_POSITION_TARGET_LOCAL_NED`.
    - Added handlers for `quadModeCmd` → `MAV_CMD_DO_SET_MODE`.

## [2026-01-20] ZMQ Command Tool
- **`multidrone/simpleZMQtakeoffland.py`**:
    - Refactored to use `argparse` for robust argument parsing.
    - Added `--takeoff` and `--land` mutually exclusive flags.
    - Added `--altitude` argument (default 10.0m).
    - Implemented `quadLandCmd` topic for landing commands.

## [2026-01-20] Command Tool Unification
- **`multidrone/takeoffland.py` & `multidrone/simpleZMQtakeoffland.py`**:
    - Unified the command-line interface for both scripts.
    - Both now support `--takeoff`, `--land`, and `--altitude=...` arguments.
    - `simpleZMQtakeoffland.py` uses `--zmq=...` for port configuration.
    - `takeoffland.py` uses `--udp=...` for port configuration.
    - Implemented `argparse` in `takeoffland.py` for standard argument handling and help display.

## [2026-01-20] Documentation
- **Added `README.md`**: Created project documentation listing files and architecture.
- **Added `UPDATES.md`**: Started tracking project history.

## [2026-01-19] Multi-Drone & Hardware Adapter
- **`multidrone/run_multidrone_bridges.sh`**: Created a new script to handle multiple drones.
    - Automatically parses `positions.txt` to count drones.
    - Orchestrates a tmux session with dedicated windows for each drone.
    - Launches paired `mavlink_to_ZMQ` and `zmq_commands_mavlink` binaries with calculated ports (UDP/ZMQ).
- **`hardware_adapter.sh`**: Refined the single-drone adapter script.
- **`runSimNoeticMulti.sh`**: Updated to support mounting custom `positions.txt` files into the Docker container.

## [2026-01-XX] UI & Refinement (Related Context)
- **GUI Refinement**: Improved `CatSwarm` GUI (in parallel workspace) to handle `.csm` project files and updated control labels.
- **Drone Configuration**: Updated UI to support table-based configuration of initial positions.

## [2025-12-22] Hardware Adapter Async Support
- **`hardware_adapter.py`**: Refactored to use asynchronous threads for sending and receiving MAVLink messages. This separated the listener and commander loops to prevent blocking.

## [2025-11-19] System Identification & Timing
- **`sysid.py`**: Converted the system identification logic from a Jupyter Notebook (`sysid.ipynb`) to a standalone Python script for better version control and execution.
- **`system_manager.py`**: Implemented precise loop timing. Replaced simple `time.sleep()` with a drift-correction mechanism to ensure exactly 100Hz (0.01s) loop execution, accounting for processing time.
