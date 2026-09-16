# Streaming

RTSP is an observability channel for the desktop UI. It is separate from the typed machine-data link
to `robot-agent`.

## V0.1 endpoints

```text
rtsp://<JETSON_IP>:8554/camera/rgb
rtsp://<JETSON_IP>:8554/camera/depth_visual
```

Optional later endpoints include `/debug/terrain` and `/debug/vo`.

The deployed target is a Jetson Orin Nano, which has no hardware video encoder. Its production path is
`appsrc -> videoconvert -> x264enc -> h264parse -> rtph264pay` via GStreamer and gst-rtsp-server.
The user unit runs a real low-latency x264 encode probe before startup. The optional
`nvvidconv -> NVMM -> nvv4l2h264enc` path remains available in source for Orin NX/AGX targets that
actually contain NVENC. Frames enter a bounded fresh-data queue; a slow encoder drops the oldest
frame.

Low-latency client example:

```bash
gst-launch-1.0 rtspsrc location=rtsp://<JETSON_IP>:8554/camera/rgb \
  protocols=tcp latency=0 drop-on-latency=true \
  ! rtph264depay ! h264parse ! decodebin ! autovideosink sync=false
```

Z16 depth never enters this lossy video path. A UI depth stream first creates a normalized/colorized
image with RealSense Viewer-style histogram equalization and a Jet palette over the configured
`min_distance_m` to `max_distance_m` range while the untouched metric data continues to depth
processing. Zero/invalid and out-of-range depth is rendered black. IR-left and IR-right
acquisition/mounts are disabled by default.

RTSP timestamps are presentation timestamps only. Sensor synchronization uses timestamps preserved in
`CameraFrameSet`.

Do not encode accelerometer or gyroscope values into depth pixels for machine use. The current H.264
RTSP stream has no application metadata contract, and pixel overlays are lossy. A UI may later burn
human-readable IMU text into `depth_visual`; algorithms must consume timestamped depth and IMU from
the typed machine-data path and associate or interpolate samples by sensor timestamp.
