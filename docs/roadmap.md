# Development roadmap

Version numbers follow `perception-service-fundamental.md` sections 32-36 and 56 and are not
renumbered. What changes here is the **execution order**, because the numbering is a naming scheme,
not a dependency graph: the fundamental document itself (section 6) splits the D435i into two
independent branches, and the two branches do not depend on each other's data.

```text
V0.1 done
  -> shared foundation: geometry/ (intrinsics, extrinsics, transforms), dataset record/replay
  -> V0.6a transport inbound + contract        (input of V0.5, so it comes first)
  -> V0.5  EKF, quality states, DR budget      (this is what removes the RTK FLOAT mission stop)
  -> V0.6b robot-agent consumes LocalizationState
  -> V0.4  VO/VIO spike, only if the V0.5 budget proves insufficient
  -> V1.0  Level 2 acceptance
  parallel, lower priority: V0.2 depth foundation -> V0.3 environment perception (drop-off)
```

Localization branch (Level 2A) carries the project's actual problem from section 7: a mission that
stops whenever RTK leaves FIXED. Perception branch (Level 2B) adds local safety awareness and is
valuable, but it does not address that problem.

## V0.1 — Sensor and UI streaming foundation — DONE (2026-09-19)

Repository bootstrap; camera discovery; RGB, depth and IMU capture; local viewer; H.264 RTSP
appropriate to target capability; bounded queues and metrics; camera/client reconnect; graceful
shutdown; systemd.

Delivered and live-verified on the Jetson: detailed D435i discovery (`camera-info`, `imu-info`),
RGB and colorized depth operator RTSP (`/camera/rgb`, `/camera/depth_visual`, RealSense
Viewer-style Jet palette over 0.2-5.0 m), bounded independently timestamped accelerometer
(100 Hz) and gyroscope (200 Hz) capture, five-second count/rate/age/drop metrics, RTSP session
expiry for clients that leave without TEARDOWN, and operator-only wedge recovery
(`camera-info --hardware-reset`, `reset-camera.sh`).

Architecture settled during V0.1: one process owns the D435i, but video and IMU are two independent
sessions on one device instance. librealsense's pipeline aggregator withholds every frameset until
each enabled stream has produced a frame, so a combined pipeline let a silent IMU also silence
RGB/depth. A video-only pipeline plus a Motion Module sensor session with a slow bounded restart
keeps RGB/depth independent of IMU health, while IMU liveness stays mandatory for `imu.ready`,
`ekf.ready` and full acceptance.

The deployed Orin Nano has no NVENC, so production RTSP uses a validated low-latency `x264enc`
path; `nvv4l2h264enc` remains an optional path for Orin NX/AGX only.

Closing evidence: reboot verification and controlled USB unplug/replug recovery without a process
restart were both confirmed by the operator on 2026-09-19.

IR left/right acquisition and RTSP are disabled by operator choice to reduce USB and encoder load.
The code paths remain; V0.4 needs synchronized raw IR stereo and will re-enable them.

## V0.6a — Transport and inbound measurements (before V0.5)

Protocol version 1: JSON over TCP on port 50053, same framing as payload-service, versioned, with
timestamps and age on every measurement. GNSS/RTK is polled directly from payload-service, not
through robot-agent, so the EKF sees the sample's own timestamp, age and per-field freshness. Vendor
heading is polled from robot-agent's `/api/v1/status`. See [inter-service](inter-service.md).

Delivered and deployed: the contract, both decoders, the request router, the listener and its two
poll threads, the health bridge over the V0.1 camera snapshot, configuration and host tests. The
vendor decoder is pinned to a live capture and reads attitude, yaw rate and body-frame velocity, not
just heading. Started after streaming, so a busy port or an absent peer can never take operator video
down. The listener is enabled; the poll clients stay off until V0.5 consumes them, because each adds
a 5 Hz load to a live mission service.

## Shared foundation — DONE (2026-09-19)

Serves both branches, which is why it is not inside either one. `geometry/`: frames and rigid
transforms (`camera_optical -> camera_body -> base_link`), the mount extrinsics configuration, and
pinhole deprojection into robot-frame point clouds. Dataset record and replay per section 51: a
version-1 capture format, a writer, a reader, a `dataset-record` tool and a `ReplayCamera` that
serves recordings through the same `ICamera` interface as the live D435i, so V0.2-V0.5 can be
developed and regression-tested without a camera, a robot or field time. See
[geometry and dataset replay](geometry-and-replay.md).

## V0.5 — Robust localization

Fuse GNSS/RTK, IMU and vendor odometry; formalize `map`, `odom` and `base_link`; implement quality
states and a dead-reckoning budget. VO/VIO is a fourth input that extends the budget, not a
prerequisite: section 36 lists all four inputs and none of them is gated on the others. Developed
and regression-tested against recorded datasets, then validated live.

## V0.6b — robot-agent integration

`robot-agent` consumes `LocalizationState` and applies degraded-mode mission policy instead of
stopping on RTK FLOAT. Health and perception state publish over the same contract.

## V0.4 — VO/VIO (conditional spike)

Stereo/RGB and IMU produce local pose, velocity and covariance for short-term continuity during GNSS
degradation. Started only if the V0.5 dead-reckoning budget proves insufficient in measurement, and
scoped as a spike first, because the known prerequisites are non-trivial: this D435i
(serial 207122078394) carries no IMU calibration, IR stereo must be re-enabled and its USB/encoder
load re-measured, and drift needs RTK FIXED as ground truth from a recorded dataset.

## V0.2 — Depth foundation (parallel branch)

Invalid-depth removal, clipping and filters followed by point-cloud generation and robot-frame
transforms. Filters are implemented over project-owned buffers, not `rs2::*` filters, so replay runs
without librealsense. Point clouds have no direct mission consumer; this milestone exists to feed
V0.3.

## V0.3 — Environment perception (parallel branch)

Ground, free space, near-field obstacle risk, drop-off, slope, roughness, traversability and
confidence. Outputs describe risk; they do not issue low-level steering commands, and they do not
duplicate the vendor's immediate reflex avoidance (section 34). Drop-off detection is the highest-
value item here and is scheduled after the localization branch by operator decision.

## V1.0 — Mission-ready perception

Meet the Level 2 acceptance scenario: RTK FIXED -> FLOAT -> FIXED without mandatory mission stop,
within configured safety budgets and with smooth global-pose recovery.
