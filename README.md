# Deep Robotics Lite3 for Robonix

This repository supports the same Robonix deployment in two explicit modes:

- `hardware`: the physical Lite3, Livox MID-360, and Orbbec camera.
- `simulation`: a headless MuJoCo Lite3 in the multi-room SceneSmith
  One-Bedroom Apartment 187, with simulated RGB-D/LiDAR, slam_toolbox, Nav2,
  Explore, Robonix Scene, and a
  Three.js/WebGL 3D viewer.

The mode is mandatory so simulation can never inherit the physical robot's
network endpoints. Hardware uses `robonix_manifest.yaml`; simulation uses the
independent, checked-in `robonix_manifest.simulation.yaml`.

## Local environment

Create the ignored local environment file once:

```bash
cp .env.example .env
$EDITOR .env
```

Set `VLM_BASE_URL`, `VLM_API_KEY`, and `VLM_MODEL`; both deployment modes
start Pilot, and startup rejects missing VLM configuration. The launch scripts
export these values to Pilot and the Scene container. Do not commit `.env`.

The scripts discover Robonix at the sibling directory `../robonix`. Only
uncomment `ROBONIX_SOURCE_DIR` in `.env` when the checkout is elsewhere.

## Simulation

The reviewed sibling repositories and revisions are listed in
`sim/vendor.lock`. With those checkouts under `~/cxk`, build once. `rbnx build` clones URL
packages into `rbnx-boot/cache/` before the MuJoCo container is built:

```bash
cd ~/cxk/robot-deep_robotics-lite3
./build.sh --mode simulation
```

To advance already cached GitHub packages later, update them explicitly and
rebuild:

```bash
rbnx update -f robonix_manifest.simulation.yaml
./build.sh --mode simulation
```

Start the complete stack and browser viewer:

```bash
./start.sh --mode simulation --viewer web
```

From Windows, keep the SSH tunnel open:

```powershell
ssh -N -L 18080:127.0.0.1:8080 -L 18091:127.0.0.1:8091 -L 50107:127.0.0.1:50107 codex
```

Open the MuJoCo apartment at `http://127.0.0.1:18080/`, the Mapping UI at
`http://127.0.0.1:18091/`, and the official Robonix Scene page at
`http://127.0.0.1:50107/`.

The default mapping profile uses `service-map-rbnx@main` with
`slam_toolbox`; the previous RTAB-Map profile remains commented in
`robonix_manifest.simulation.yaml`. Build a persistent map by driving with
`./sim/teleop.sh`, then save it as `lite3_house_187` in the Mapping UI.
See `sim/README.md` for cold-start global localization from an alternate
spawn pose.

Run a complete navigation acceptance test while the stack is active:

```bash
./sim/verify-navigation.sh
```

It sends a Nav2 goal 0.60 m away at a 30 degree bearing, then checks mapping,
TF, non-zero velocity, odometry displacement, goal success, upright posture,
final stop, and simulation time. Optional arguments select distance and
bearing:

```bash
./sim/verify-navigation.sh 0.8 -20
```

Stop everything cleanly:

```bash
./stop.sh --mode simulation
```

See `sim/README.md` for the architecture, ROS interfaces, validation commands,
and troubleshooting details.

## Physical robot

Create the ignored credentials file and edit the hardware settings:

```bash
cp .env.example .env
$EDITOR .env
./build.sh --mode hardware
./start.sh --mode hardware
```

Stop it with:

```bash
./stop.sh --mode hardware
```

Hardware mode preserves the checked-in Lite3, MID-360, Orbbec, mapping,
navigation, speech, and skill configuration. Soma loads `soma.yaml` and the
local `urdf/Lite3.urdf`; update sensor transforms after physical calibration.
