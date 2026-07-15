# Deep Robotics Lite3

Minimal Robonix deployment for a Deep Robotics Lite3 with a Livox MID-360
and an Orbbec Gemini 330-series RGB-D camera.

The deployment contains only the robot packages needed for chassis control,
sensing, mapping, and Nav2 navigation. Mapping uses the same
`syswonder/service-map-rbnx` package as the Ranger Mini V3 deployment, with
RTAB-Map selected as its backend.

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

`stop.sh` is idempotent and also recovers interrupted boots: it reaps only
processes and ProcessManager records belonging to this deployment, then checks
that the Lite3 feedback port (UDP 43897) is free before returning success.

The three scripts accept additional arguments and pass them directly to the
corresponding `rbnx build`, `rbnx boot`, or `rbnx shutdown` command.

Tencent speech also requires these values in the ignored `.env` file:

```dotenv
TENCENT_ASR_APPID=
TENCENTCLOUD_SECRET_ID=
TENCENTCLOUD_SECRET_KEY=
```

## Robonix client

Start the robot deployment first, then run the client on the computer whose
microphone and speaker should be used:

```bash
cd ~/cxk/robonix-client
source .venv/bin/activate
robonix-client --robot-host <lite3-jetson-ip>
```

Open `http://127.0.0.1:7860`, connect to Atlas on port `50051`, and select
`audio_client_bridge` for both input and output on the Audio page. The bridge
uses port `60002` and should only be reachable over a trusted LAN or Tailscale.

## Included packages

- Lite3 quadruped chassis and robot description
- Livox MID-360 lidar
- Orbbec RGB-D camera
- PointCloud2-to-LaserScan conversion
- Mapping service with the RTAB-Map backend
- Nav2 navigation

Soma loads `soma.yaml` and the deployment-local `urdf/Lite3.urdf`. Sensor
mount transforms in that URDF should be updated after physical calibration.
