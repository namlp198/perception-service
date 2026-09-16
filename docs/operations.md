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

`test_rtsp.sh` passes only when real H.264 media arrives. It retries up to three times two seconds
apart (`PERCEPTION_RTSP_PROBE_ATTEMPTS`, `PERCEPTION_RTSP_PROBE_RETRY_DELAY_S`) because a shared
gst-rtsp-server media is torn down after its last client leaves and prepared again for the next
one; a probe that reconnects immediately after another probe (for example `verify-running-service.sh
streaming` followed at once by `full`) can land in that window even though `stream.frames_published`
keeps rising. Note that `stream.frames_published` counts frames handed to the RTSP path whether or
not a client is attached, so it never proves that a client received media — only the probe does.

Camera ownership is exclusive during diagnostics. Do not run RealSense Viewer, `camera-info` or
`imu-info` alongside the service. For the bounded maintenance workflow that restores the previously
active service even when a diagnostic fails:

```bash
./scripts/jetson/validate-v01-hardware.sh --confirm-service-interruption jetson-local
```

The service logs cumulative IMU sample/drop counts, effective accelerometer/gyroscope rates and last
sample ages every five seconds. D435i samples retain separate sensor and host-capture timestamps;
they are not fabricated into same-time accel/gyro pairs.

RGB/depth run in a video-only `rs2::pipeline`; the Motion Module runs in its own `rs2::sensor`
session opened on the pipeline's own `rs2::device` instance (a second instance of the same camera
runs its own global-time keeper and fails to claim the depth interface every 100 ms, flooding the
journal with `failed to claim usb interface 0, is busy`). Two warnings remain and are benign:
occasional `messenger-libusb.cpp control_transfer ... Resource temporarily unavailable` (the
global-timestamp reader polling the hardware clock while the bus is busy; it retries) and, on this
unit, `IMU Calibration is not available` — D435i serial 207122078394 carries no IMU calibration, so
default intrinsics/extrinsics apply until `rs-imu-calibration.py` writes one (a VIO/EKF accuracy
item, not a streaming one).

Enabling accel/gyro inside the video pipeline is not allowed: the
pipeline aggregator withholds every frameset until each enabled stream has delivered a frame, so an
IMU that never samples also blocks RGB/depth (observed live on 2026-09-16: `camera.frames_received=0`
plus the three-capture reconnect loop). Missing or stale motion reports
`imu.ready=false ekf.ready=false` and restarts only the Motion Module every
`camera.imu.restart_interval_ms` (default 30 s, `0` disables); video capture and RTSP are never
stopped for an IMU fault. This is not an IMU fallback for estimation: EKF remains locked until both
sensors are fresh. Readiness probes deliberately run outside the systemd unit startup transaction.
`restart-service.sh` first requires the streaming check (RGB + depth media) to pass — a hard failure
otherwise — and then runs the strict full check; when only the IMU part is unmet it exits with code 3
so the deploy reports "service restarted, RGB/depth live, IMU acceptance FAILED" instead of a generic
failure, while leaving the service running.

If `imu-info` and the service both report zero accel/gyro samples although `camera-info` lists the
`Accel`/`Gyro` profiles, the known cause (resolved 2026-09-16) is a Motion Module left in a wedged
state that survives every process restart and clears only when the camera re-enumerates over USB.
The deployed Jetson runs a source-built librealsense 2.58.3 with the RSUSB backend (`lsusb -t`
shows every D435i interface as `Driver=usbfs`); after a re-enumeration the same service build
streamed accel 100.9 Hz / gyro 200.0 Hz alongside 30 fps RGB/depth with `imu.ready=true`. The
signature of the wedged state is `motion_callbacks=0` on every `startup_timeout` while
`Motion Module started sensor='Motion Module' accel=[...] gyro=[...]` shows the exact profiles were
opened without error, and librealsense's own WARN log (routed to the journal) stays quiet. Recovery:

```bash
./scripts/jetson/reset-camera.sh --confirm-service-interruption jetson-local   # hardware reset + verify
```

The reset is operator-only and never automatic because it interrupts RGB/depth for several
seconds. `imu-info` accepts overrides for a bounded experiment (service stopped):

```bash
./build/jetson-local/imu-info config/default.yaml 10 100 200 no-video   # Motion Module alone
./build/jetson-local/imu-info config/default.yaml 10 200 200 no-video   # equal accel/gyro rates
./build/jetson-local/imu-info config/default.yaml 10 100 200 video      # alongside the video pipeline
```

Repeated open/close cycles of one session and a full service session were shown not to wedge the
module; the trigger earlier that day was never isolated (candidates: the abandoned two-pipeline
design that ended in `No device connected`, or the 4 s Motion Module restart loop it used). If
librealsense used the native kernel backend instead, the classic checks apply: `lsmod | grep
hid_sensor`, `/dev/iio:device*` presence and permissions, `sudo dmesg | grep -iE 'hid|iio'`, and
Intel's patched `hid-sensor-*` modules; rebuilding with `FORCE_RSUSB_BACKEND=ON` is the usual fix.

After any reboot or deployment, do not accept `systemctl active` alone. Require live RGB/depth,
non-zero accel/gyro counters and H.264 encoding with:

```bash
./scripts/jetson/verify-running-service.sh
```

The default `full` mode fails unless video, H.264, accelerometer and gyroscope are all live.
Use `streaming` only to diagnose the independent RGB/depth branch; it may pass while clearly
reporting that IMU/EKF is not ready.

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
