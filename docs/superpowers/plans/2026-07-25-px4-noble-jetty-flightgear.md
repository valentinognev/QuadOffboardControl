# PX4 Noble Jetty FlightGear Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade Jammy PX4 NVIDIA sim image to Ubuntu 24.04 (Noble), Gazebo Jetty, PX4 v1.17.0, and FlightGear 2024.1.6 AppImage with dual smoke verification.

**Architecture:** Evolve `Dockerfiles/PX4JammySimNvidia.dockerfile` on `nvidia/cuda:13.1.2-base-ubuntu24.04`, replace Harmonic with `gz-jetty`, pin PX4 `v1.17.0`, install FG AppImage under `/opt/flightgear` with `/usr/local/bin/fgfs` wrapper (`APPIMAGE_EXTRACT_AND_RUN=1`), then rename jammy→noble files and update runner scripts.

**Tech Stack:** Docker, NVIDIA CUDA base, Gazebo Jetty (`gz-jetty`), PX4 Autopilot v1.17.0, FlightGear 2024.1.6 AppImage, xvfb

**Spec:** `docs/superpowers/specs/2026-07-25-px4-noble-jetty-flightgear-design.md`

## Global Constraints

- Naming: **noble** (not jammy/ubuntu24) for files, scripts, image tag `px4-noble-sim-ros`
- Base: `nvidia/cuda:13.1.2-base-ubuntu24.04`
- PX4: `v1.17.0`
- Gazebo: `gz-jetty` (not `gz-harmonic`)
- FlightGear: `https://mirrors.ibiblio.org/flightgear/ftp/release-2024.1/flightgear-2024.1.6-linux-amd64.AppImage`
- Verify FG at **build** and **runtime**; build fails if `fgfs --version` fails under xvfb
- **No git commits** unless the user explicitly asks
- Nested repo: `Dockerfiles/` is its own git repo (`PX4dockerfiles`); `runSimJammy.sh` lives in `general_infrastructure/`
- Leave Noetic Dockerfile/scripts untouched

---

### Task 1: Noble Dockerfile content

**Files:**
- Modify then rename: `Dockerfiles/PX4JammySimNvidia.dockerfile` → `Dockerfiles/PX4NobleSimNvidia.dockerfile`

**Steps:**

- [ ] **Step 1:** Change `FROM` to `nvidia/cuda:13.1.2-base-ubuntu24.04`
- [ ] **Step 2:** Replace `openjdk-11-*` with `openjdk-17-jdk` / `openjdk-17-jre`; fix `update-alternatives` for java-17
- [ ] **Step 3:** Add pip PEP 668 flags: `python3 -m pip install --break-system-packages ...` (all pip invocations)
- [ ] **Step 4:** Replace `gz-harmonic` with `gz-jetty` in the OSRF apt block
- [ ] **Step 5:** Add FlightGear deps + install block after sim packages, e.g.:

```dockerfile
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    xvfb libfuse2t64 fuse3 libgl1 libglu1-mesa libx11-6 libxext6 libxrandr2 libxi6 \
    libxcursor1 libxinerama1 libxxf86vm1 libasound2t64 libpulse0 libnss3 libgtk-3-0 \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/flightgear \
 && wget -q -O /opt/flightgear/flightgear-2024.1.6-linux-amd64.AppImage \
      https://mirrors.ibiblio.org/flightgear/ftp/release-2024.1/flightgear-2024.1.6-linux-amd64.AppImage \
 && chmod +x /opt/flightgear/flightgear-2024.1.6-linux-amd64.AppImage \
 && printf '%s\n' '#!/bin/bash' 'export APPIMAGE_EXTRACT_AND_RUN=1' \
      'exec /opt/flightgear/flightgear-2024.1.6-linux-amd64.AppImage "$@"' \
      > /usr/local/bin/fgfs \
 && chmod +x /usr/local/bin/fgfs \
 && xvfb-run -a fgfs --version
```

If `--version` is unsupported, use a documented fallback that still proves the binary starts (e.g. timeout + help). If FUSE/AppImage fails, extract with `--appimage-extract` to `/opt/flightgear/squashfs-root` and point `fgfs` at the extracted binary.

- [ ] **Step 6:** Change PX4 checkout from `v1.15.4` to `v1.17.0`
- [ ] **Step 7:** Keep JSBSim; if `dpkg` fails on noble, replace with a noble-compatible artifact or install from source only if required for the image to build
- [ ] **Step 8:** `git mv` dockerfile to `PX4NobleSimNvidia.dockerfile` (inside `Dockerfiles/` repo)

---

### Task 2: Rename helper scripts + runner

**Files:**
- Rename: `Dockerfiles/PX4_jammy_sim_build.sh` → `Dockerfiles/PX4_noble_sim_build.sh`
- Rename: `Dockerfiles/PX4_jammy_sim_bash.sh` → `Dockerfiles/PX4_noble_sim_bash.sh`
- Rename: `runSimJammy.sh` → `runSimNoble.sh` (general_infrastructure root)
- Create: `Dockerfiles/PX4_noble_sim_verify_fg.sh` (runtime FG smoke)

**Steps:**

- [ ] **Step 1:** Build script content:

```bash
docker build --tag px4-noble-sim-ros --file ./PX4NobleSimNvidia.dockerfile .
```

- [ ] **Step 2:** Bash script: same as jammy but image `px4-noble-sim-ros`
- [ ] **Step 3:** `runSimNoble.sh`:

```bash
#!/bin/bash
export PX4_SITL_DOCKER_NAME=px4-noble-sim-ros
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:latest
docker kill $PX4_SITL_DOCKER_NAME
./run_px4_sitl_docker.sh 'make px4_sitl gz_x500'
```

- [ ] **Step 4:** Runtime verify script runs container with X11 mounts and `fgfs --version` (exit non-zero on failure)
- [ ] **Step 5:** Remove leftover jammy-named files

---

### Task 3: Build image and verify

**Steps:**

- [ ] **Step 1:** From `Dockerfiles/`: `./PX4_noble_sim_build.sh` (long; ensure Docker/GPU host ready)
- [ ] **Step 2:** Confirm build log contains successful FG `--version` smoke
- [ ] **Step 3:** Run `./PX4_noble_sim_verify_fg.sh` (or equivalent) for runtime smoke
- [ ] **Step 4:** Spot-check: `docker run --rm px4-noble-sim-ros bash -lc 'lsb_release -rs; gz topic --help >/dev/null; test -x /usr/local/bin/fgfs'`

---

### Task 4: Project docs

**Files:**
- Modify: `UPDATES.md` (new top entry, bump version style per project convention)
- Modify: `README.md` only if architecture still says Jammy/Harmonic-only for this image

**Steps:**

- [ ] **Step 1:** Add UPDATES entry documenting noble/Jetty/PX4 1.17.0/FG AppImage/renames
- [ ] **Step 2:** Adjust README sim bullet if it only describes Noetic classic and should mention the noble Jetty image path
- [ ] **Step 3:** Do **not** commit

---

## Spec coverage

| Spec requirement | Task |
|------------------|------|
| noble renames | 2 |
| CUDA 24.04 base | 1 |
| gz-jetty | 1 |
| PX4 v1.17.0 | 1 |
| FG AppImage install | 1 |
| Build-time FG verify | 1 |
| Runtime FG verify | 2, 3 |
| UPDATES/README | 4 |
| No commits | Global |

## Execution note

User requested: execute with Grok; **do not commit**. Prefer inline execution in this session.
