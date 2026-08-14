# Project Updates

This file documents the development progress and changes made to the `CatSwarm/general_infrastructure` project by the AI agent.

## [2026-08-14] SITL: iris colors match Observation Board
- `inject_iris_colors.py` rewrites Gazebo script materials to OB palette at spawn (`iris_N` → drone `N`).
- Wired in `sitl_multiple_run.sh`; bind-mounted by `runSimNoeticMulti.sh`; baked in Noetic Dockerfile.
- Spec: ObservationBoard `docs/superpowers/specs/2026-08-12-gazebo-drone-colors-design.md`. Restart sim to see colors (host mount; rebuild image only for bake path).

## [2026-08-11] mavlink-server :14580 required for RTCM inject
- Cause: fleet `mavlink-server.conf` wrote `[[udp_server]]` **without** `address`; `run-mavlink-server.sh` skipped it → no `udpserver://0.0.0.0:14580`; Apply RTK bridge sent RTCM into the void (D2 worked only because conf was hand-fixed).
- Fix: template + `mavlink-server-configuration.sh` include `address = "0.0.0.0"`; `ensure_mavlink_rtcm_udp.sh` runs on Apply RTK (`--sink=mavlink_rtcm`).

## [2026-08-11] RTCM mavlink bridge: RF vs WiFi exclusive
- Companion switch + `startRtcmToMavlinkPI`: serial/RF Apply → rf-only bridge; WiFi Apply → wifi-only. No forced dual path for mavlink on wifi mode.
- OB Apply RTK always passes `--sink=mavlink_rtcm`.

## [2026-08-08] QGC default HIGHRES_IMU + OPTICAL_FLOW_RAD streams
- Cause: QGC on mavlink-server TCP `:5760` showed `DISTANCE_SENSOR` but not `HIGHRES_IMU` / `OPTICAL_FLOW_RAD`; HA `SET_MESSAGE_INTERVAL` on UDP `:14540` does not raise those rates on the shared QGC path.
- Fix: `startup_scripts/util/qgc_mavlink_streams.py` requests msgid 105/106 @ 50 Hz on `tcp:127.0.0.1:5760`, refreshed every 10 s; hooked from `start_companion_drone_tmux.sh` (+ kill on `--kill`). Override: `COMPANION_QGC_MAVLINK` / `COMPANION_QGC_STREAM_HZ`.
- Tests: `startup_scripts/util/tests/test_qgc_mavlink_streams.py`. Sync `startup_scripts` to Pi and restart companion (or run the script once).

## [2026-08-05] Companion sniff/profile: USB F9P @ 230400 + ROVER_WIRE
- `sniff_companion_gps_profile.py`: after ACM@115200, probe `ttyUSB*` @ 230400 then 115200 (MON-VER) → `usb_f9p|/dev/ttyUSB*`.
- `companion_gps_module.sh`: f9p fleet sets `ROVER_BAUD=230400` for `ttyUSB*`, `ROVER_WIRE=ubx`; persist/load/export `ROVER_WIRE` (LC29H → nmea).
- `flush_companion_gps_from_hw.sh` / `switch_EAUSB_DAUART.sh`: comments + `--f9p` prefers present `ttyUSB0` when no ACM.

## [2026-08-04] S5 XY wander: mockup OF+GPS fight; S5 RECAPTURE
- Cause: `EKF2_OF_CTRL=1` with Gazebo mockup OF + GPS fused → EKF XY drifts metres in S5; Python lacked CPP 8 m hold RECAPTURE so VelocityPID chased jumps.
- Fix: `.post` boots `EKF2_OF_CTRL=0` (OF still streamed); HA GPS-quality NO FIX sets OF on / GPS off; PY S5 RECAPTURE err>8 m. Restart sim + HA `zmq_commands_mavlink` + Python SMs; Land/Restart/Takeoff.

## [2026-08-04] Sim OF/ranger from 2 cm (match real air)
- Cause: `.post` set `SENS_FLOW_MINHGT=0.7` and stock `model://lidar` clamps at 0.2 m — blocked low-AGL OF/ranger while real air works from ~2 cm; S5 XY wandered (Gazebo drift >1 m/5 s on D2).
- Fix: `SENS_FLOW_MINHGT=0.02`; `inject_iris_sensors` inlines lidar with `min_distance`/`ray min` 0.02. Restart `./runSimNoeticMulti.sh --num=4` (lidar needs respawn; param also in `.post`).

## [2026-08-04] Ranger/OF: offboard streams + Gazebo loopback
- Cause: `.post` streamed `DISTANCE_SENSOR`/`OPTICAL_FLOW_RAD` only on GCS (`udp_gcs_port_local`); HA uses offboard `14580+i→14540+i`. Also gzserver multicast `Network is unreachable` → later `simulator_mavlink poll timeout` froze HIL (`bottom_clearance=-1`).
- Fix: `.post` also streams both on `$udp_offboard_port_local`; `runSimNoeticMulti.sh` sets `GAZEBO_IP=127.0.0.1` + `GAZEBO_MASTER_URI=http://127.0.0.1:11345`. Restart sim via `./runSimNoeticMulti.sh --num=4`.

## [2026-08-02] Final-review fixes: OF mockup default, GPS boot params, C bridges flag
- **I1:** `inject_iris_sensors.py` defaults to `libgazebo_opticalflow_mockup_plugin.so` (velocity+lidar; works headless). `CATSWARM_OF_MODE=camera|both` for the OpenCV `model://px4flow` plugin (needs GPU/X11). Until `OPTICAL_FLOW_RAD` is confirmed on the operator display, treat OB **NO FIX** as estimator-loss risk (GPS off + no flow), not safe GPS denial.
- **I2:** `run_multidrone_bridges.sh` no longer passes `--zmqFlightData` to the C `zmq_commands_mavlink` (unknown option → rc=1). Keep `--zmqFlightData` on Python `hardware_adapter_multi.sh` only.
- **I3:** airframe `.post` now `param set EKF2_GPS_CTRL 7` + `SYS_HAS_GPS 1` at boot (matches HA `_GPS_ON`; undoes persisted NO FIX). **`SYS_HAS_GPS` is reboot-required** — boot-time set is the recovery path; mid-sim NO FIX still relies on live `EKF2_GPS_CTRL=0`.
- Tests: `multidrone/tests/test_inject_iris_sensors.py` (mockup default + camera/both).

## [2026-08-02] `sitl_multiple_run.sh`: spawn model before starting px4 (OF boot-order fix)
- Cause (Task 2 finding, fixed in Task 7): `spawn_model` started `bin/px4` in the background **before** running jinja-gen/inject/`gz model --spawn-file`. PX4's `Sensors::init()` calls `InitializeVehicleOpticalFlow()` exactly once at boot and only wires up `OPTICAL_FLOW_RAD` if `sensor_optical_flow` is already `.advertised()` (i.e. already published at least once) at that instant — starting px4 first guaranteed the model/px4flow didn't exist yet, so the check always failed for the whole session.
- Fix: reordered so jinja-gen → inject → `gz model --spawn-file` all run **before** `bin/px4` is started, matching the reference single-instance `sitl_run.sh`.
- Camera OF under headless/Xvfb still may not publish; prefer mockup inject (entry above). `DISTANCE_SENSOR` streams correctly. Details: `ObservationBoard/.superpowers/sdd/task-7-report.md`.

## [2026-08-01] Airframe `.post` + bake multi-SITL sensors into Noetic image
- `multidrone/airframes/10015_gazebo-classic_iris.post`: OF stream/params + GPS-on boot params (see 2026-08-02 entry) + takeoff params; kept `DISTANCE_SENSOR` stream.
- `Dockerfiles/PX4NoeticSimNvidia.dockerfile` bakes inject / `sitl_multiple_run.sh` / `.post`. Build context = `general_infrastructure/` (see `Dockerfiles/PX4_noetic_sim_build.sh`). Details: `Dockerfiles/UPDATES.md` 1.4.0.

## [2026-08-01] Multi-SITL iris: sensors inject
- `inject_iris_sensors.py`: lidar always; OF via mockup (default) or `model://px4flow` (`CATSWARM_OF_MODE`). Compat wrapper: `inject_iris_lidar.py`. Tests: `multidrone/tests/test_inject_iris_sensors.py`.

## [2026-07-31] Noetic Dockerfile: PX4 v1.17.0
- `Dockerfiles/PX4NoeticSimNvidia.dockerfile` pins `v1.17.0` (`ARG PX4_TAG`); explicit `ninja … px4` + `sitl_gazebo-classic`. Rebuild: `Dockerfiles/./PX4_noetic_sim_build.sh`.

## [2026-07-31] Multi-SITL iris: downward Gazebo lidar (DISTANCE_SENSOR)
- `multidrone/inject_iris_lidar.py` injects nested `model://lidar` (not an inline link — topic must be `lidar`).
- `sitl_multiple_run.sh` runs inject after jinja; `runSimNoeticMulti.sh` mounts inject + `airframes/10015_gazebo-classic_iris.post` (streams DISTANCE_SENSOR @ 20 Hz).
- Verified: mavlink msgid 132 @ ~0.19 m AGL on 14541–14544; HA FLIG `bottom_clearance_m=0.19`. Restart sim (`./runSimNoeticMulti.sh --num=4`); SM `takeoff_height_source=rangefinder`.

## [2026-07-27] F9P→PX4: 1 Hz meas rate for flight
- Fleet profile `f9p` uses `DA_RATE_MS=1000` (was 100 / 10 Hz) to avoid high-rate noise in flight.
- PX4 stay-alive still via emulate GGA heartbeat (~450 ms).

## [2026-07-27] Boot: recognize F9P — stop 30s USB0 wait before tmux
- `run-companion-drone.sh` mapped f9p→ea and waited on `/dev/ttyUSB0` (up to 30s) before
  creating `catswarm_sim` — looked like “tmux never started” with only F9P plugged.
- Wait ACM for f9p (brief 10s); required UARTs only (skip blocking on optional AMA4).
- `companion_gps_ensure_ports` migrate no longer defaults module to ea over a saved F9P.

## [2026-07-27] A/B: FC reboot fixed GPS; F9P+LC29H both OK on AMA0
- GPS_1_CONFIG debug sweeps wedged FC GPS until soft reboot (wire was fine).
- Both modules plugged: EA then F9P each produce GPS_RAW fix_type=3 → FLIG gps≠0.

## [2026-07-27] F9P→PX4: synth RMC + 10 Hz; `--f9p` port flush
- F9P often lacks steady RMC (PX4 needs GGA+RMC); `emulate_gps_to_px4` synthesizes RMC from GGA.
- F9P CFG: GSV off + RMC all interfaces; fleet F9P rate 10 Hz; `switch --f9p` flushes ACM0@115200.
- If FC still has no GPS_RAW while GPIO14 TX passes: check physical Pi header pin 8 → FC GPS1 RX.

## [2026-07-27] F9P→PX4: 10 Hz + GGA/RMC-only NMEA bridge
- Root cause: F9P at 1 Hz with GSA/GSV flooded AMA0; PX4 never set GPS-present / GPS_RAW.
- `rover_zmq` forwards only GGA/RMC on `--nmea-zmq-bind`; F9P fleet default `DA_RATE_MS=100`.
- `switch_EAUSB_DAUART.sh --f9p` flushes ACM0@115200 (was saving EA USB0 defaults).

## [2026-07-27] Companion GPS boot: prefer saved, else sniff+persist
- `companion_gps_boot_resolve_available`: if saved rover tty missing, sniff F9P→EA→DA→UART,
  flush `~/.config/companion-gps`, then start that profile (no more silent GPS skip when USB0
  LC29H is present but saved F9P ACM is not).
- `start_companion_drone_tmux.sh` `start_gps_combo` uses the resolver; adds `--f9p`; topology
  labels F9P vs EA correctly.

## [2026-07-25] Fix Noble image FG prebuild (no sitl launch)
- Root cause: `make … flightgear_rascal` always runs `sitl_run.sh`/`fgfs`; `DONT_RUN=1` does not skip FG path.
- Dockerfile now builds `flightgear_bridge` via ninja only; Rascal launch remains runtime (`fixedwing/runSimFlightGearRascal.sh`).

## [2026-07-25] Fixed-wing FlightGear Rascal host helpers
- Noble image: pre-build nolockstep + `flightgear_bridge` (not the launch target).
- `fixedwing/fg_spawn.env` + `runSimFlightGearRascal.sh`: injectable in-air spawn (500 m, ~30 m/s).
- `fixedwing/run_straight_flight.py`: start sim + OFFBOARD body-forward velocity hold.

## [2026-07-25] Drop --disable-rembrandt for FG 2024
- FG 2024 rejects `disable-rembrandt` (GUI dialog + usage dump); removed via `Dockerfiles/patch_px4_flightgear_sitl.sh`.
- `Dockerfiles/` now has its own `UPDATES.md`; README documents Rascal SITL + FG 2024 pitfalls (MAVLink race, rembrandt, fgfs/dbus libs).

## [2026-07-25] PX4 FG SITL patches (Rascal + FG 2024)
- `patch_px4_flightgear_sitl.sh`: add omitted `1039_flightgear_rascal` to airframes CMakeLists; patch `FG_run.py` (disable TerraSync, drop duplicate `model-hz`, dedupe CLI for FG 2024, keep both `--generic`).
- Wired into `PX4NobleSimNvidia.dockerfile` after PX4 v1.17.0 checkout.

## [2026-07-25] Fix fgfs wrapper for SITL + FlightGear
- `fgfs` no longer exports AppImage `LD_LIBRARY_PATH` into `dbus-run-session` (broke `fgfs --version`).
- Documented `make px4_sitl_nolockstep flightgear_rascal` build/run flow (build px4 before bridge if MAVLink NOTFOUND).

## [2026-07-25] Dockerfiles README: Noble / FlightGear usage
- Replaced stale VS Code-only `Dockerfiles/README.md` with Noble/Noetic image table, build/run scripts, FlightGear start notes.

## [2026-07-25] PX4 sim image: Ubuntu 24.04 (Noble) + Gazebo Jetty + FlightGear 2024
- Renamed Jammy sim Docker assets to Noble (`PX4NobleSimNvidia.dockerfile`, `runSimNoble.sh`, image tag `px4-noble-sim-ros`).
- Base `nvidia/cuda:13.1.2-base-ubuntu24.04`; Gazebo `gz-jetty`; PX4 `v1.17.0`; FlightGear 2024.1.6 AppImage + data pack.
- Build+runtime smoke verified: UFO@KSFO under `dbus-run-session`+`xvfb`.

## [2026-07-21] Deploy: UART layout v3 + GPIO14 TX self-test
- Strip conflicting `dtparam=uart0=on`; probe point is header pin 8 (GPIO14), not legacy UART4 pin 32.
- `verify_px4_nmea_uart_tx.sh` samples GPIO14 during live NMEA/burst; hooked from `phase_peripherals`.

## [2026-07-21] Deploy: idempotent peripherals phase (UART layout v2)
- Soft Update previously skipped UART rewrite once an old marker existed, leaving stale boot config.
- `ensure_fleet_uart_boot_config` rewrites to `fleet-uart-layout: 2` (uart0=NMEA→PX4); new `--phase=peripherals`.
- ObservationBoard Update always runs the peripherals phase plus companion-gps ensure.

## [2026-07-20] Deploy: skip install when SystemManagerMain already in place
- CMake install of `SystemManagerMain` failed when built path equaled dest; now chown/chmod in place instead.

## [2026-07-20] Deploy: companion apt package set + offline-debs
- Canonical `COMPANION_APT_SYSTEM`/`COMPANION_APT_BUILD` lists; `phase_build` always ensures build packages.
- Offline fallback via `offline-debs/` + `fetch-offline-debs.sh`; OB rsyncs debs with deploy scripts.

## [2026-07-20] Deploy build: install Eigen3 for SystemManagerCPP
- Soft Update `phase_build` failed on Pis without Eigen; `phase_system` now installs `libeigen3-dev`.
- `phase_build` installs it only if missing (offline-safe); `libcppzmq-dev` optional (vendored header).

## [2026-07-20] Deploy/boot: ensure PX4 NMEA UART0 before reboot
- `ensure_companion_uart_ports.sh` migrates saved `COMPANION_PX4_GPS_PORT` ttyAMA4→ttyAMA0, keeps DA off UART0.
- Runs from companion boot wrapper, deploy `phase_verify`, and ObservationBoard `deploy_pi5` before reboot.

## [2026-07-20] Pi NMEA→PX4 on UART0 + EA 10 Hz
- `startup_scripts`: NMEA→PX4 default `/dev/ttyAMA0` (GPIO14 TX); optional DA UART default `/dev/ttyAMA4`.
- Rover init phase-2 docs updated to 10 Hz; UART tables in README/SCRIPTS.md updated.

## Historical summary
- **MAVLink multi-drone control** (`mavlinkTakeoffLandAlt.py`, `mavlinkSwitchToMode.py`, `drone_control.sh`): added dynamic port config for shared (14550) vs per-drone (14540+i/14030+i) links, SysID-aware targeting via heartbeat wait, telemetry-polling feedback loops replacing fixed sleeps, closed-loop altitude control with early exit, and authoritative flightmode string reporting.
- **ZMQ-based command tooling** (`simpleZMQtakeoffland.py`, `zmqTakeoffLandAlt.py`, `takeoffland.py`): unified `--takeoff`/`--land`/`--altitude` CLI across scripts, added 10 Hz OFFBOARD position-setpoint control loop (`quadPosNedCmd`/`quadModeCmd`), renamed/added ZMQ port args, improved command reliability with LINGER + post-send delay.
- **Hardware adapter** (`hardware_adapter.py`/`.sh`, `hardware_adapter_multi.sh`, `command_queue.h`, `zmq_topics.h`, `zmq_commands_mavlink.c`): switched to client-mode command uplink targeting each drone's port/SysID, added position/mode command types and handlers, refactored to async send/receive threads.
- **Multi-drone orchestration**: `run_multidrone_bridges.sh` parses `positions.txt` to spawn a tmux session with paired MAVLink↔ZMQ bridges per drone; `runSimNoeticMulti.sh` supports mounting custom `positions.txt`; parallel CatSwarm GUI work added `.csm` project support and table-based initial-position config.
- **Project bootstrap**: initial `README.md`/`UPDATES.md`; `sysid.py` converted from notebook to standalone script; `system_manager.py` implemented drift-corrected 100 Hz loop timing.
