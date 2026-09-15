# Localization model

Level 2 navigation consumes estimated pose, covariance and localization quality instead of treating
RTK FIXED as a binary mission gate.

Continuous VO/VIO, IMU and vendor odometry drive the smooth, locally accurate `odom -> base_link`
relationship. Absolute GNSS/RTK observations correct the global `map -> odom` relationship. When RTK
returns, innovation and covariance checks feed a gradual EKF correction; the pose must never snap.

Quality states are `HIGH`, `MEDIUM`, `LOW`, `DEAD_RECKONING` and `CRITICAL`. The service reports the
state and supporting covariance/health. Only `robot-agent` maps that report to mission action.

Dead reckoning is bounded by time and distance since the last reliable global fix, VO tracking quality,
covariance and IMU health. It is a controlled degraded mode, not a permanent GNSS replacement.
