# Architecture

This document is a focused view of the canonical `perception-service-fundamental.md`.

## Responsibility boundary

Jetson Orin turns sensor measurements into local environmental understanding, continuous pose
estimates, confidence and health. `robot-agent` decides whether a mission continues, slows, pauses,
recovers or stops. The vendor quadruped executes locomotion, balance, gait and immediate reflex
avoidance. There must be exactly one final mission authority: `robot-agent`.

## Data flow

```text
D435i -> RealSenseCamera -> CameraFrameSet
                              |-> RTSP video -> operator UI
                              |-> depth -> environment state
                              `-> stereo/RGB + IMU -> VO/VIO -> EKF -> localization state
                                                               ^
                                                GNSS + vendor odometry
```

The streaming, depth and localization paths are peers. Streaming must never sit inside an algorithm
or supply machine data to `robot-agent`.

## Stable contracts

- RealSense SDK values are converted immediately to project-owned frame types.
- GStreamer values stay behind the streaming interface.
- Transport adapters serialize project-owned output models; core code does not depend on a wire
  format, ROS message, protobuf class or socket type.
- Raw metric depth stays lossless in the perception branch. A colorized depth stream may be produced
  separately for people.
- All queues are bounded to two or three items and discard the oldest item under pressure.
- Sensor timestamp, capture timestamp, frame number and stream identity remain available.

Only V0.1 modules are present now. Future module directories are created when implementation begins,
not as empty placeholders.
