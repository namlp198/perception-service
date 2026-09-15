# AGENTS.md

## Project

`perception-service` runs on NVIDIA Jetson Orin with Ubuntu 22.04. Its primary sensor is an Intel
RealSense D435i. The mission counterpart is `robot-agent` at `192.168.1.206`.

Read `perception-service-fundamental.md` before making architectural changes. The focused documents
under `docs/` explain individual concerns but do not override the fundamental document.

## Ownership

`perception-service` owns D435i capture, RGB/IR/depth/IMU processing, RTSP video for operators,
depth perception, VO/VIO, state estimation, localization confidence and perception health.

`robot-agent` owns mission execution, waypoint policy, payload control, the vendor quadruped API and
the final mission action. The vendor platform retains gait, balance and immediate reflex avoidance.

## Architecture

```text
Camera SDK -> Camera abstraction -> CameraFrameSet
                                  |-> Streaming -> RTSP -> UI
                                  |-> Depth perception
                                  `-> VO/VIO -> EKF -> typed transport -> robot-agent
```

RTSP is only an operator-observability channel. It is never a machine-data input to mission logic.

## Dependency boundaries

- Keep every `rs2::*` type inside `src/camera/`.
- Keep every GStreamer type inside `src/streaming/`.
- Keep network and serialization types inside future transport adapters.
- Algorithms communicate through project-owned models.
- Do not add mission state machines, waypoint policy or vendor locomotion control here.
- Do not treat metric Z16 depth as lossy H.264 video.
- Keep VO/VIO and depth perception as separate branches.
- Keep all real-time queues bounded and drop the oldest item when full.

## Coding

- Use C++20, RAII and explicit ownership.
- Prefer `std::unique_ptr`; do not use raw owning pointers or global mutable state.
- Use `std::chrono`, `std::filesystem`, `enum class` and unit-bearing names.
- Use configuration instead of hard-coded runtime settings.
- Use `spdlog` in production modules; console streams are acceptable in command-line tools only.
- Treat project warnings as errors and add tests with behavioral changes.
- Do not add a dependency without recording why it is needed.
- Keep changes small, reviewable and compatible with offline dataset replay.

## Build

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Use `jetson-debug` only on a provisioned Jetson. Never make host-only tests require a camera.
