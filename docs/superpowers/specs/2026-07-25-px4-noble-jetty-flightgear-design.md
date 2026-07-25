# PX4 Noble (Ubuntu 24.04) + Gazebo Jetty + FlightGear 2024 Design

Date: 2026-07-25  
Status: approved for planning (pending user review of this file)

## Goal

Upgrade the Jammy (Ubuntu 22.04) PX4 NVIDIA sim Docker image to Ubuntu 24.04 (Noble), Gazebo Jetty, PX4 v1.17.0, and FlightGear 2024.1.6 (AppImage), with build-time and runtime smoke verification that FlightGear starts.

## Decisions (locked)

| Topic | Choice |
|-------|--------|
| Naming | Ubuntu codename **noble** everywhere (files, scripts, image tag) |
| Base image | Evolve existing Dockerfile on **`nvidia/cuda:13.1.2-base-ubuntu24.04`** (Approach A) |
| Method | Incremental evolve of jammy Dockerfile, then rename (Approach 1) |
| PX4 | **`v1.17.0`** |
| Gazebo | **`gz-jetty`** via OSRF ubuntu-stable (replace `gz-harmonic`) |
| FlightGear | AppImage **2024.1.6** from ibiblio (not distro package) |
| FG verify | **Both** build-time (`xvfb-run`) and runtime (noble bash / one-shot) |
| Out of scope | Noetic image; CUDA→plain Ubuntu base; multidrone RL rework beyond `runSimNoble.sh` |

## File renames

| Current | New |
|---------|-----|
| `Dockerfiles/PX4JammySimNvidia.dockerfile` | `Dockerfiles/PX4NobleSimNvidia.dockerfile` |
| `Dockerfiles/PX4_jammy_sim_build.sh` | `Dockerfiles/PX4_noble_sim_build.sh` → tag `px4-noble-sim-ros` |
| `Dockerfiles/PX4_jammy_sim_bash.sh` | `Dockerfiles/PX4_noble_sim_bash.sh` |
| `runSimJammy.sh` | `runSimNoble.sh` → `PX4_SITL_DOCKER_NAME=px4-noble-sim-ros` |

Leave `PX4NoeticSimNvidia.dockerfile` and noetic scripts unchanged. Remove jammy-named files after noble equivalents exist (git mv preferred).

## Dockerfile changes

### Base & toolchain

- `FROM nvidia/cuda:13.1.2-base-ubuntu24.04`
- Keep existing build tools, entrypoint, ccache, astyle, user `1001`, X11/`DISPLAY` setup
- Ubuntu 24.04 fixes as needed:
  - OpenJDK: use a version available on noble (prefer 17 if 11 unavailable)
  - Python pip: satisfy PEP 668 (`--break-system-packages` and/or venv) so installs match current workflow
- Keep JSBSim; if focal `.deb` fails on noble, switch to a compatible package or skip with a documented note only if install is impossible

### Sim stack

- OSRF keyring + `gazebo/ubuntu-stable` for `$(lsb_release -cs)` (= `noble`)
- Install **`gz-jetty`** instead of `gz-harmonic`
- Keep gstreamer plugins, OpenCV, mesa-utils, `QT_X11_NO_MITSHM=1`
- Clone PX4 Autopilot, checkout **`v1.17.0`**, `git submodule update --init --recursive`
- Build `make px4_sitl_default` (same role as current image)

### FlightGear 2024.1.6 AppImage

- URL: `https://mirrors.ibiblio.org/flightgear/ftp/release-2024.1/flightgear-2024.1.6-linux-amd64.AppImage`
- Install deps needed for AppImage + OpenGL/X11 (at least): `libfuse2` (or noble equivalent), `xvfb`, `mesa-utils`, and common FG runtime libs as discovered during install
- Install path: `/opt/flightgear/flightgear-2024.1.6-linux-amd64.AppImage` (`chmod +x`)
- Provide `/usr/local/bin/fgfs` wrapper that sets `APPIMAGE_EXTRACT_AND_RUN=1` (avoids FUSE requirement inside Docker)
- Fallback if extract-and-run is unreliable: extract once to `/opt/flightgear/squashfs-root` and wrap the extracted binary

### Verification

**Build-time (fail the image build on error):**

```text
xvfb-run -a env APPIMAGE_EXTRACT_AND_RUN=1 fgfs --version
```

(or FG’s equivalent version flag). Must exit 0 and indicate 2024.x.

**Runtime:**

- After build, run container via noble bash helper / one-shot and execute the same `fgfs --version` smoke (optionally short GUI start when host `DISPLAY` is available)
- Non-zero exit if verification fails

## Docs

- Update `UPDATES.md` (new top entry) when implementation lands
- Update `README.md` only if architecture text must mention noble / Jetty / FlightGear

## Success criteria

1. Noble-named Dockerfile and scripts replace jammy names; image builds as `px4-noble-sim-ros`
2. Container OS is Ubuntu 24.04; `gz` reports Jetty / gz-sim 10 family
3. PX4 tree is at `v1.17.0` and SITL default build completes in-image
4. FlightGear AppImage is installed; build-time and runtime `fgfs` smoke checks pass
5. `runSimNoble.sh` points at the new image name

## Non-goals

- Changing Noetic/ROS Focal image
- Replacing NVIDIA base with plain `ubuntu:24.04`
- Full FG scenery/aircraft download beyond what the AppImage needs to start and report version
- Git commit/push unless explicitly requested
