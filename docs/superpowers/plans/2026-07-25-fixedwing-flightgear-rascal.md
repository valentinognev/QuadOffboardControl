# Fixed-wing FlightGear Rascal Implementation Plan

> **For agentic workers:** Implement task-by-task. Do **not** git commit unless the user explicitly asks.

**Goal:** Pre-build Rascal FG SITL in Noble image; host `fixedwing/` runner with injectable spawn env; OFFBOARD velocity straight-flight script.

**Architecture:** Dockerfile builds `px4_sitl_nolockstep` + `flightgear_bridge` via ninja (not the `flightgear_rascal` launch target). Host mounts `fg_spawn.env` into the container and sources it before `make`. Python script starts that runner and streams body-NED velocity setpoints.

**Tech Stack:** Docker, PX4 v1.17.0, FlightGear 2024, bash, pymavlink

## Global Constraints
- Noble image only (`px4-noble-sim-ros`)
- No commits without user permission
- Update root `README.md` / `UPDATES.md` and `Dockerfiles/README.md` / `Dockerfiles/UPDATES.md`

---

### Task 1: Dockerfile build step

**Files:**
- Modify: `Dockerfiles/PX4NobleSimNvidia.dockerfile` (after `make px4_sitl_default`)
- Modify: `Dockerfiles/README.md`, `Dockerfiles/UPDATES.md`

- [ ] Add mavlink prebuild + `ninja … flightgear_bridge` (not `make … flightgear_rascal`)
- [ ] Document in Dockerfiles README/UPDATES

### Task 2: Spawn env + runner

**Files:**
- Create: `fixedwing/fg_spawn.env`
- Create: `fixedwing/runSimFlightGearRascal.sh`

- [ ] Env with in-air 500 m / 30 m/s (vc knots)
- [ ] Runner mounts env, sources it, runs make

### Task 3: Straight-flight MAVLink script + docs

**Files:**
- Create: `fixedwing/run_straight_flight.py`
- Modify: `README.md`, `UPDATES.md`

- [ ] Start sim unless `--no-sim`; arm; OFFBOARD; velocity loop
- [ ] Project README/UPDATES
