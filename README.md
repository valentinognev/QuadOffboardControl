# CatSwarm General Infrastructure

This project provides the general infrastructure for multi-drone Reinforcement Learning (RL) simulations using PX4 Autopilot and ROS Noetic. It acts as the bridge between high-level RL policies and the low-level flight control stack.

## Architecture

The system follows a modular architecture:

1.  **Simulation Environment**: Docker images for PX4 SITL — Noetic/Focal (`px4-noetic-sim-ros`, Gazebo Classic) and Noble/24.04 (`px4-noble-sim-ros`, Gazebo Jetty + FlightGear 2024). Prefer `./runSimNoble.sh` for Gazebo Jetty; FlightGear Rascal via `./fixedwing/runSimFlightGearRascal.sh` (injectable `fixedwing/fg_spawn.env`) or `make px4_sitl_nolockstep flightgear_rascal` inside the Noble image (see `Dockerfiles/README.md`). Nested sim images live in `Dockerfiles/` (`PX4dockerfiles`).
2.  **Communication Bridge**:
    - **MAVLink (UDP)**: Connects the PX4 SITL instances to the bridge.
    - **Bridges (C++)**: `mavlink_to_ZMQ` and `zmq_commands_mavlink` convert MAVLink messages to/from ZeroMQ.
    - **ZeroMQ (ZMQ)**: Provides a high-performance IPC layer for the Python control stack.
3.  **Control Stack (Python)**:
    - `system_manager.py` (assumed): Manages the main control loop.
    - `rl_policy.py`: Implements the RL inference using PyTorch.

## Key Files

### Simulation & Setup
- **`fixedwing/runSimFlightGearRascal.sh`**: Noble + FlightGear Rascal plane. Mounts `fixedwing/fg_spawn.env` (`FG_ARGS_EX`) for in-air spawn (default 500 m / ~30 m/s) without rebuilding the image.
- **`fixedwing/run_straight_flight.py`**: Starts the Rascal sim (unless `--no-sim`), arms if needed, switches OFFBOARD, streams body-forward velocity setpoints for straight flight.
- **`runSimNoeticMulti.sh`**: The main entry point. Starts the Dockerized PX4/ROS simulation environment for multiple drones. It handles container management, X11 forwarding, and cleaning up processes. Mounts iris sensors inject (px4flow + lidar), iris OB-color inject, + `10015_gazebo-classic_iris.post` so SITL publishes `OPTICAL_FLOW` / `DISTANCE_SENSOR` (matches cage rangefinder).
- **`multidrone/inject_iris_sensors.py`**: Injects nested `model://px4flow` + `model://lidar` into generated multi-SITL iris SDF before spawn. (`inject_iris_lidar.py` is a thin compat wrapper.)
- **`multidrone/inject_iris_colors.py`**: Recolors iris mesh visuals at spawn to Observation Board `droneColors.ts` palette (`iris_N` → OB id `N`).
- **`multidrone/run_multidrone_bridges.sh`**: Sets up the communication layer. It creates a tmux session with pairs of bridge processes (`mavlink_to_ZMQ` and `zmq_commands_mavlink`) for each drone defined in the positions file.
- **`multidrone/positions.txt`**: Configuration file defining the initial spawn positions (x, y, z, yaw) for the drones.
- **`hardware_adapter.sh`**: Legacy/Single-drone script to start the hardware adapter bridges.

### Core Logic
- **`rl_policy.py`**: Contains the `RLPolicy` class, a PyTorch module implementing the actor network (Encoder -> GRU -> Gaussian Head).
- **`simpleMavlinkTest.py` / `simpleMavlinkSwitchTo.py`**: Utility scripts for testing MAVLink connections and switching flight modes.
- **`multidrone/simpleZMQtakeoffland.py`**: Sends takeoff and land commands via ZMQ.
    - Usage: `python3 multidrone/simpleZMQtakeoffland.py --takeoff --altitude=15.0`
    - Usage: `python3 multidrone/simpleZMQtakeoffland.py --land --zmq=7793`
- **`multidrone/takeoffland.py`**: Sends takeoff and land commands via MAVLink UDP.
    - Usage: `python3 multidrone/takeoffland.py --takeoff --altitude=12.0 --udp=14541`
    - Usage: `python3 multidrone/takeoffland.py --land --udp=14542`

## Usage

1.  **Start the Simulation**:
    ```bash
    ./runSimNoeticMulti.sh --num 3
    ```
    (Adjust `--num` for the number of drones).

2.  **Start Communication Bridges**:
    ```bash
    ./multidrone/run_multidrone_bridges.sh
    ```
    This will start a tmux session named `multidrone_bridges`.

3.  **Run Control Logic**:
    (Depending on your specific workflow, e.g., running the system manager or RL training script).

## Project Updates

For a detailed history of changes and fixes, please refer to **[UPDATES.md](UPDATES.md)**.

**Updating Policy**:
- Every agent working on this project **MUST** update `UPDATES.md` with a summary of their changes, fixes, and new features.
- Entries should be chronological (newest first).
- Keep descriptions laconic but self-contained so that future agents can quickly understand the project state.
