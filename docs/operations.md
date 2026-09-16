# Operations and health

V0.1 must observe capture FPS, received/dropped frames, frame age, per-stream FPS and drops, encoder
latency and encoder status. CPU, GPU, memory and temperature join the health model as deployment work
matures. A running process is not proof of a healthy sensor pipeline.

Expected camera-loss behavior is: report degraded health, pause streaming, reconnect with a bounded
retry interval and resume without restarting the process. SIGINT and SIGTERM must cause orderly
pipeline shutdown. RTSP clients may disconnect and reconnect independently.

The user deployment can be inspected with:

```bash
systemctl --user status perception-service-user.service
ss -ltn 'sport = :8554'
./scripts/test_rtsp.sh 127.0.0.1 /camera/rgb
```

Camera ownership is exclusive during diagnostics. Do not run RealSense Viewer, `camera-info` or
`imu-info` alongside the service. For the bounded maintenance workflow that restores the previously
active service even when a diagnostic fails:

```bash
./scripts/jetson/validate-v01-hardware.sh --confirm-service-interruption jetson-local
```

The service logs cumulative IMU sample/drop counts, effective accelerometer/gyroscope rates and last
sample ages every five seconds. D435i samples retain separate sensor and host-capture timestamps;
they are not fabricated into same-time accel/gyro pairs.

If the configured video+IMU request cannot be resolved, the service emits an error and retries once
with IMU disabled. This degraded mode intentionally keeps RGB and both IR RTSP endpoints online;
it is not an IMU acceptance success. Run `imu-info` separately while the service is stopped to
diagnose the exact motion profile.

## PC-to-Jetson deployment

The supported development loop keeps editing on the PC and performs target builds and camera tests
on the Jetson. The rsync deploy preserves remote `build/` and `.runtime/` trees so native packages
and incremental ARM build output are not copied from or removed by the PC.

```bash
# Always inspect a new target/path first.
./scripts/jetson/deploy.sh --dry-run

# Normal source deploy with target verification before service restart.
./scripts/jetson/deploy.sh --build --test --restart

# Inspect the latest service log remotely after deployment.
ssh percpc@192.168.1.220 \
  bash /home/percpc/workspace/perception-service/scripts/jetson/collect-logs.sh
```

On Windows, `scripts\jetson\deploy.bat` always enables detailed rsync progress/statistics and writes
a timestamped PowerShell transcript under `out\deploy-logs\`. The log includes selected deployment
settings, rsync's itemized file changes, remote configure/build/test output and systemd status after
restart.

Key-based SSH remains the unattended default. During initial setup,
`deploy.bat -InteractiveAuth ...` permits one password or key-passphrase prompt and reuses that SSH
connection for rsync, build, tests and restart. Password text is not echoed into the transcript.
When started by double-click, the batch file pauses before closing on both success and failure. Set
`PERCEPTION_DEPLOY_NO_PAUSE=1` only for automation that must return immediately.
With no command-line arguments it first presents Dry-run, staged Sync/Build/Test without restart,
Full deploy, Source-only and Cancel choices; all operational choices enable the one-prompt interactive
SSH mode automatically. Stage the first IMU-enabled build, then use the hardware acceptance script to
control the service interruption and restart.

Use `--delete` only when the remote source tree must exactly match the PC. Excluded runtime, build,
cache and recording paths remain protected. A failed sync, build or test stops the workflow before
the service restart; the currently running service is therefore left untouched.

Field recordings must preserve RGB, both IR streams, metric depth, IMU, calibration and timestamps;
later phases add GNSS, vendor odometry and estimator output. Large datasets belong under ignored
storage, never in Git. Every field failure should become an offline replay regression test.
