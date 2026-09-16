# Development roadmap

## V0.1 — Sensor and UI streaming foundation

Repository bootstrap; camera discovery; RGB, stereo IR, depth and IMU capture; local viewer; H.264
RTSP appropriate to target capability; bounded queues and metrics; camera/client reconnect; graceful
shutdown; systemd.
Completion requires successful unplug/replug recovery without a process restart.

Current implementation status (2026-09-16): bootstrap and RGB/IR/depth capture are complete; the
RGB and the colorized depth operator RTSP endpoints are confirmed on Jetson. IR acquisition and
RTSP are now disabled by operator choice to reduce USB and encoding load. The depth palette is being
updated to a RealSense Viewer-style Jet scale and requires redeployment confirmation.
Detailed discovery, four-view local visualization, and bounded timestamped accel/gyro capture are
implemented in source.
Live discovery confirms the attached D435i exposes 100/200/400 Hz accelerometer and 200/400 Hz
gyroscope profiles; the original unsupported 63 Hz accelerometer request was corrected to 100 Hz.
The unified video+IMU pipeline was proven on the live Jetson to block RGB/depth whenever the IMU
delivers nothing (pipeline aggregator semantics). Source now runs a video-only pipeline plus an
independent Motion Module sensor session with a slow bounded restart. IMU liveness is mandatory for
EKF/full acceptance, but missing IMU samples do not stop the RGB/depth publishing branch. The actual Orin Nano target has no NVENC, so the impossible
`nvv4l2h264enc` gate was replaced by a real low-latency x264 encode preflight. V0.1 remains open until
this revision passes Jetson build, strict running
verification, reboot verification and controlled camera unplug/replug recovery.

## V0.2 — Depth foundation

Invalid-depth removal, clipping and filters followed by point-cloud generation and robot-frame
transforms.

## V0.3 — Environment perception

Ground, free space, near-field obstacle risk, drop-off, slope, roughness, traversability and confidence.
Outputs describe risk; they do not issue low-level steering commands.

## V0.4 — VO/VIO

Stereo/RGB and IMU produce local pose, velocity and covariance for short-term continuity during GNSS
degradation.

## V0.5 — Robust localization

Fuse GNSS/RTK, VO/VIO, IMU and vendor odometry; formalize `map`, `odom` and `base_link`; implement
quality states and a dead-reckoning budget.

## V0.6 — robot-agent integration

Approve a typed, versioned bidirectional protocol and publish localization, perception and health to
`robot-agent` while accepting normalized remote measurements.

## V1.0 — Mission-ready perception

Meet the Level 2 acceptance scenario: RTK FIXED -> FLOAT -> FIXED without mandatory mission stop,
within configured safety budgets and with smooth global-pose recovery.
