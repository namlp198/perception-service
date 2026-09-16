# perception-service

Local perception and localization service for a quadruped robot. It runs on NVIDIA Jetson Orin,
captures an Intel RealSense D435i and will publish operator video plus typed perception/localization
state. Mission authority remains in `robot-agent` at `192.168.1.206`.

## Current status

This repository is implementing V0.1. RGB, depth and IMU acquisition from a D435i are implemented;
the operator endpoints publish real H.264/RTP through gst-rtsp-server with bounded low-latency
queues. The deployed Orin Nano has no hardware video encoder, so production RTSP uses a validated
low-latency `x264enc` path; `nvv4l2h264enc` remains an optional path for Orin NX/AGX targets.
Camera reconnect, graceful shutdown and basic live metrics are
implemented. Detailed camera discovery and bounded, independently timestamped accelerometer/gyroscope
capture are implemented in source; Jetson build and hardware acceptance remain required. Hardware-
encoder validation and the controlled unplug/replug recovery test also remain V0.1 work.

The D435i profile uses accelerometer 100 Hz and gyroscope 200 Hz. One process owns the D435i, but
video and IMU are two independent sessions on that device: a video-only `rs2::pipeline` carries
RGB/depth, and the Motion Module is opened through its own `rs2::sensor` with a callback that queues
every accel/gyro sample. They are deliberately not combined into one pipeline: librealsense's pipeline
aggregator withholds every frameset until each enabled stream (including accel and gyro) has produced
a frame, so a silent IMU would also silence RGB/depth — which is exactly what the 2026-09-16 unified
build did on the live Jetson. Missing/stale IMU data makes `imu.ready=false` and `ekf.ready=false`
and triggers a slow, bounded Motion Module restart (`camera.imu.restart_interval_ms`, default 30 s;
`0` disables) that never touches the video pipeline. IMU data remains mandatory before EKF is allowed
to run and for full hardware acceptance. Readiness verification runs after systemd has started the
service, not as `ExecStartPost`, so a failed acceptance probe cannot terminate healthy RGB/depth
streaming. A full deploy therefore ends in one of three states: hard failure (build/sync/restart or
RGB/depth RTSP down), success, or exit code 3 = service restarted with RGB/depth live but the
mandatory IMU acceptance unmet.

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
rtsp://<JETSON_IP>:8554/camera/depth_visual
```

`depth_visual` uses RealSense Viewer-style histogram equalization and a Jet palette over the
configured 0.2–5.0 m range for operator viewing. Values outside the range and invalid zero depth are
black. It does not replace or modify the original metric Z16 frame used by depth processing. IR
capture and RTSP mounts are disabled by default to reduce USB and encoder load.

Jetson diagnostics must run while the service is stopped because only one process should own the
D435i pipeline:

```bash
./build/jetson-local/camera-info config/default.yaml         # add --hardware-reset to re-enumerate the D435i
./build/jetson-local/imu-info config/default.yaml 10            # [accel_fps] [gyro_fps] [video|no-video] optional
./scripts/jetson/validate-v01-hardware.sh --confirm-service-interruption jetson-local
./scripts/jetson/verify-running-service.sh
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
