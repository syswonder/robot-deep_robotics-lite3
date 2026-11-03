# Lite3 complete MuJoCo simulation

`sim/` is one complete simulation subsystem. The supported workflow is always
started from the repository root:

```bash
./build.sh --mode simulation
./start.sh --mode simulation --viewer web
./stop.sh --mode simulation
```

There are no separate deployment stages. `manage.sh` is the single internal
entry point used by the root scripts. It applies pinned upstream patches,
builds the controller and simulation image, starts or stops the runtime, and
runs acceptance checks.

## Layout

```text
sim/
├── manage.sh          unified build/runtime/validation helper
├── compose.yaml       MuJoCo/controller/ROS bridge container
├── runtime/           physics, bridge, sensors, and container source
├── providers/         Robonix lidar and camera capability providers
├── scene/             lightweight official Robonix Scene image
├── patches/           pinned upstream integration patches
├── validation/        bridge, sensor, and navigation acceptance checks
├── web3d/             Three.js room viewer and assets
├── verify-navigation.sh
└── vendor.lock        reviewed upstream revisions
```

## Architecture

The checked-in `robonix_manifest.simulation.yaml` contains the Lite3 simulated
chassis, simulated lidar and camera providers, slam_toolbox, Nav2, Explore,
Robonix Scene, Executor, Pilot, and Liaison. Mapping uses the official
`service-map-rbnx@main` branch; the commented RTAB-Map profile
remains in the simulation manifest. A startup validation rejects
physical target IP and port fields. Startup also requires `VLM_BASE_URL`,
`VLM_API_KEY`, and `VLM_MODEL` in the ignored root `.env`.

The `lite3-sim` container runs the authoritative MuJoCo physics, the official
x86 RL controller, ROS 2 bridge, simulated sensors, and port 8080 Web3D server.
Separate host-network containers run mapping, Nav2, Explore, and Scene. Atlas,
Soma, Executor, Pilot, Liaison, and `rbnx boot` run directly on the codex host.

All ROS components use domain 42, Fast DDS over UDP, and `/clock`. The bridge publishes odometry, TF, joints, IMU, lidar, point
cloud, RGB-D, and camera calibration. Nav2 returns commands through `/cmd_vel`
to the same simulated chassis primitive used by Robonix.

Physics uses a 1 ms timestep and absolute wall-clock deadlines, targeting
RTF 1.0 (`elapsed /clock time / elapsed monotonic wall time`). Short sensor or
rendering stalls are recovered over subsequent physics steps. Each catch-up
step still polls ROS, reads the latest motor command, sends controller state,
and yields to the external RL controller. Backlog is capped at 50 ms so a long
process suspension cannot trigger an unbounded burst of stale control steps.
Pause does not accumulate a physics backlog. Sustained CPU overload or stalls
longer than the catch-up cap can still reduce RTF.

Camera and point-cloud messages use typed byte arrays so ROS does not validate
each payload byte in Python while holding the GIL. This keeps sensor encoding
from blocking the physics loop without reducing sensor resolution or rates.
The pinned controller patch also drains queued UDP state packets on each
receive tick and uses the newest valid state. Reading only one packet per
tick at a nominal 1 kHz creates growing feedback latency after scheduling
jitter. Rebuild with `./build.sh --mode simulation` when that patch changes.

## Browser access

From Windows:

```powershell
ssh -N -L 18080:127.0.0.1:8080 -L 18091:127.0.0.1:8091 -L 50107:127.0.0.1:50107 codex
```

- MuJoCo House 187: `http://127.0.0.1:18080/`
- Mapping and map library: `http://127.0.0.1:18091/`
- Robonix Scene: `http://127.0.0.1:50107/`
- Scene 2D map: `http://127.0.0.1:50107/2d`

## Build and save the slam_toolbox map

The imported apartment has collision geometry and a safe spawn, but no
slam_toolbox pose graph. Start in the checked-in `map_mode: mapping`, then:

1. Run `./sim/teleop.sh` in a second codex terminal.
2. Use `w/s` to move, `a/d` to turn, and `x` to stop. Visit the living
   room, bedroom, bathroom, doorways, and return near the start so slam_toolbox
   can close the loop.
3. Open the Mapping UI, verify the occupancy grid, choose Save, and name it
   `lite3_house_187`.

The saved artifacts are under
`maps/lite3_house_187/`: `occupancy.pgm`, `occupancy.yaml`,
`occupancy.png`, `posegraph.posegraph`, `posegraph.data`, and
`meta.yaml`.

## Cold-start global localization

Stop the stack. In the active mapping block of
`robonix_manifest.simulation.yaml`, change `map_mode: mapping` to:

```yaml
map_mode: localization
map_id: lite3_house_187
```

To simulate a carried robot, set a different known collision-free pose in
`.env`:

```dotenv
LITE3_SPAWN_X=3.92
LITE3_SPAWN_Y=1.00
LITE3_SPAWN_YAW=1.57
```

Restart the stack. The map-localizer first runs the branch's stationary Top-K
scan matcher; an accepted result is handed to slam_toolbox. If the scan is
ambiguous, AMCL becomes the fallback and requires some motion. Watch status in
the Mapping UI. Remove the three spawn overrides and restore
`map_mode: mapping` before starting a new map.

## Validation

With the stack running, execute a real Nav2 goal and validate mapping, TF,
velocity, displacement, final posture, action success, final stop, and
simulation time:

```bash
./sim/verify-navigation.sh
./sim/verify-navigation.sh 0.8 -20
```

Run the sensor-specific acceptance check with:

```bash
./sim/manage.sh sensor-acceptance
```

This warms up for 7 seconds, then measures 30 seconds of `/clock` against a
monotonic wall clock. Both the overall RTF and each 10-second window must be
at least 0.85. There is no fixed upper RTF limit, allowing future accelerated
simulation modes to pass this check. Sensor contracts, stationary drift,
standing height, and tilt are checked as well. Run this check with the robot
standing and no active navigation goal.

Lower-level bridge validation is available as:

```bash
./sim/manage.sh bridge-smoke
```
