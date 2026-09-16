# perception-service

Local perception and localization service for a quadruped robot. It runs on NVIDIA Jetson Orin,
captures an Intel RealSense D435i and will publish operator video plus typed perception/localization
state. Mission authority remains in `robot-agent` at `192.168.1.206`.

## Current status

This repository is implementing V0.1. RGB, stereo IR and depth acquisition from a D435i are running;
the three operator endpoints publish real H.264/RTP through gst-rtsp-server with bounded low-latency
queues. The backend selects Jetson H.264 when `nvv4l2h264enc` is available and otherwise uses an
explicit x264 zerolatency fallback. Camera reconnect, graceful shutdown and basic live metrics are
implemented. Detailed camera discovery and bounded, independently timestamped accelerometer/gyroscope
capture are implemented in source; Jetson build and hardware acceptance remain required. Hardware-
encoder validation and the controlled unplug/replug recovery test also remain V0.1 work.

The deployed D435i profile uses accelerometer 100 Hz and gyroscope 200 Hz. If librealsense cannot
resolve an enabled IMU profile, the long-running service logs the failure and retries in degraded
video-only mode so the operator RTSP feeds remain available; `imu-info` and hardware acceptance
still fail until the IMU profile is corrected.

## Build and test on a host

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

The host preset does not require a D435i or Jetson multimedia packages. To configure all V0.1
hardware adapters on a provisioned Jetson:

```bash
cmake --preset jetson-debug
cmake --build --preset jetson-debug
ctest --preset jetson-debug
```

The build never downloads dependencies automatically. See [setup](docs/setup.md).

The currently enabled user service exposes:

```text
rtsp://<JETSON_IP>:8554/camera/rgb
rtsp://<JETSON_IP>:8554/camera/ir_left
rtsp://<JETSON_IP>:8554/camera/ir_right
```

Jetson diagnostics must run while the service is stopped because only one process should own the
D435i pipeline:

```bash
./build/jetson-local/camera-info config/default.yaml
./build/jetson-local/imu-info config/default.yaml 10
./scripts/jetson/validate-v01-hardware.sh --confirm-service-interruption jetson-local
```

## Deploy from the development PC

The default target is `percpc@192.168.1.220` with the project at
`/home/percpc/workspace/perception-service`. From Linux or WSL, preview the transfer first:

```bash
./scripts/jetson/deploy.sh --dry-run
```

Then synchronize source, build/test on the Jetson and restart the existing user service:

```bash
./scripts/jetson/deploy.sh --build --test --restart
```

From Windows PowerShell, use the WSL wrapper:

```powershell
.\scripts\jetson\deploy.ps1 -DryRun
.\scripts\jetson\deploy.ps1 -Build -Test -Restart
```

From Command Prompt, the batch wrapper enables detailed console output and saves the same session to
`out\deploy-logs\deploy-<timestamp>.log`:

```bat
scripts\jetson\deploy.bat -DryRun
scripts\jetson\deploy.bat -Build -Test -Restart
```

If SSH key authentication has not been configured yet, add `-InteractiveAuth`; SSH connection
multiplexing keeps the password/passphrase prompt to one per deployment:

```bat
scripts\jetson\deploy.bat -InteractiveAuth -DryRun
scripts\jetson\deploy.bat -InteractiveAuth -Build -Test -Restart
```

The batch window pauses after either success or failure so logs remain visible when it is launched by
double-click. Set `PERCEPTION_DEPLOY_NO_PAUSE=1` before calling it only for automated or already-open
terminal workflows.

When launched by double-click without arguments, the batch file shows a menu for dry-run, staged
sync/build/test without restart, full deploy, source-only sync or cancel. These interactive menu
choices automatically use `-InteractiveAuth`, so the Jetson SSH password or key passphrase can be
entered once and reused for the complete operation. Use staged deployment for the first IMU rollout,
then run the hardware acceptance script before allowing it to restart the service.

The deploy excludes `.git`, build output, `.runtime`, caches, recordings and generated media. It
does not delete stale remote files unless `--delete` is explicitly supplied. Override the target
with command-line options or the `PERCEPTION_JETSON_*` environment variables documented by
`./scripts/jetson/deploy.sh --help`. SSH public-key authentication is recommended because deploys
use non-interactive `BatchMode` by default.

## Documentation

- [Architecture](docs/architecture.md)
- [Setup and build](docs/setup.md)
- [Streaming](docs/streaming.md)
- [Inter-service boundary](docs/inter-service.md)
- [Localization model](docs/localization.md)
- [Operations and health](docs/operations.md)
- [Development roadmap](docs/roadmap.md)
- [Fundamental architecture](perception-service-fundamental.md), the canonical baseline
