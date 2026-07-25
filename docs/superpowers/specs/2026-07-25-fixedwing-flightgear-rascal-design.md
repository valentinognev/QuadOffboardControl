# Fixed-wing FlightGear Rascal SITL + straight flight

## Goal
Pre-build nolockstep + `flightgear_bridge` in the Noble image (do not launch `flightgear_rascal` during build — `DONT_RUN` does not skip FG `sitl_run.sh`), add `fixedwing/` host scripts that spawn in-air via mounted `FG_ARGS_EX` config, and stream OFFBOARD velocity so the plane continues straight.

## Decisions
- Image: Noble only (`PX4NobleSimNvidia.dockerfile` / `px4-noble-sim-ros`).
- Spawn config: host `fixedwing/fg_spawn.env` mounted read-only; runner sources it so edits need no image rebuild.
- Defaults: `--in-air --units-meters --altitude=500`, `--vc≈58.3` kn (30 m/s), plus `--disable-terrasync`.
- Control: OFFBOARD + `SET_POSITION_TARGET_LOCAL_NED` velocity stream (~10 Hz). Plane assumed already in air at spawn speed; script arms if PX4 is not armed.
- Runner: dedicated `fixedwing/runSimFlightGearRascal.sh` (does not change `run_px4_sitl_docker.sh`).

## Components
| Path | Role |
|------|------|
| `Dockerfiles/PX4NobleSimNvidia.dockerfile` | `ninja … flightgear_bridge` after nolockstep `px4` + mavlink symlink (no sitl launch) |
| `fixedwing/fg_spawn.env` | Editable `FG_ARGS_EX=...` |
| `fixedwing/runSimFlightGearRascal.sh` | Docker run + volume mount + make |
| `fixedwing/run_straight_flight.py` | Start sim (optional), arm, OFFBOARD, velocity loop |

## Out of scope
- Noetic / Gazebo Classic FlightGear
- Multi-vehicle FG
- ALTITUDE-mode velocity (OFFBOARD chosen instead)
