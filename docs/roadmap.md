# Development roadmap

## V0.1 — Sensor and UI streaming foundation

Repository bootstrap; camera discovery; RGB, stereo IR, depth and IMU capture; local viewer; Jetson
hardware H.264 RTSP; bounded queues and metrics; camera/client reconnect; graceful shutdown; systemd.
Completion requires successful unplug/replug recovery without a process restart.

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
