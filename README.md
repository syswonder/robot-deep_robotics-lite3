# Deep Robotics Lite3

Minimal Robonix deployment for a Deep Robotics Lite3 with a Livox MID-360
and an Orbbec Gemini 330-series RGB-D camera.

The deployment contains only the robot packages needed for chassis control,
sensing, RTAB-Map mapping, and Nav2 navigation. RTAB-Map uses the same
`syswonder/service-map-rbnx` package as the Ranger Mini V3 deployment.

## Configure

Install Robonix and the ROS 2 dependencies required by the packages, then
create the local credentials file:

```bash
cp .env.example .env
$EDITOR .env
```

`.env` is ignored by Git. Do not put VLM credentials in the manifest.

## Deploy

```bash
./build.sh
./start.sh
```

Stop the deployment with:

```bash
./stop.sh
```

The three scripts accept additional arguments and pass them directly to the
corresponding `rbnx build`, `rbnx boot`, or `rbnx shutdown` command.

## Included packages

- Lite3 quadruped chassis and robot description
- Livox MID-360 lidar
- Orbbec RGB-D camera
- PointCloud2-to-LaserScan conversion
- RTAB-Map mapping
- Nav2 navigation

Soma loads `soma.yaml` and the deployment-local `urdf/Lite3.urdf`. Sensor
mount transforms in that URDF should be updated after physical calibration.
