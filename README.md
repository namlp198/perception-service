# perception-service

Local perception and localization service for a quadruped robot. It runs on NVIDIA Jetson Orin,
captures an Intel RealSense D435i and will publish operator video plus typed perception/localization
state. Mission authority remains in `robot-agent` at `192.168.1.206`.

## Current status

This repository is implementing V0.1. RGB, stereo IR and depth acquisition from a D435i are running;
the three operator endpoints publish real H.264/RTP through gst-rtsp-server with bounded low-latency
queues. The backend selects Jetson H.264 when `nvv4l2h264enc` is available and otherwise uses an
explicit x264 zerolatency fallback. Camera reconnect, graceful shutdown and basic live metrics are
implemented. Dedicated IMU acquisition, detailed discovery output and hardware-encoder validation on
a host exposing the Jetson encoder devices remain V0.1 work.

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

## Documentation

- [Architecture](docs/architecture.md)
- [Setup and build](docs/setup.md)
- [Streaming](docs/streaming.md)
- [Inter-service boundary](docs/inter-service.md)
- [Localization model](docs/localization.md)
- [Operations and health](docs/operations.md)
- [Development roadmap](docs/roadmap.md)
- [Fundamental architecture](perception-service-fundamental.md), the canonical baseline
