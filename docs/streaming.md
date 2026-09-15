# Streaming

RTSP is an observability channel for the desktop UI. It is separate from the typed machine-data link
to `robot-agent`.

## V0.1 endpoints

```text
rtsp://<JETSON_IP>:8554/camera/rgb
rtsp://<JETSON_IP>:8554/camera/ir_left
rtsp://<JETSON_IP>:8554/camera/ir_right
```

Optional later endpoints include `/camera/depth_visual`, `/debug/terrain` and `/debug/vo`.

The preferred production path is
`appsrc -> videoconvert -> nvvidconv -> NVMM -> nvv4l2h264enc -> h264parse -> rtph264pay` via GStreamer
and gst-rtsp-server. When the Jetson encoder element is unavailable and
`allow_software_fallback: true`, the service selects `x264enc tune=zerolatency speed-preset=ultrafast`
and logs the downgrade. Hardware encoding must be verified rather than inferred from a pipeline
string. Frames enter a bounded fresh-data queue; a slow encoder drops the oldest frame.

Low-latency client example:

```bash
gst-launch-1.0 rtspsrc location=rtsp://<JETSON_IP>:8554/camera/rgb \
  protocols=tcp latency=0 drop-on-latency=true \
  ! rtph264depay ! h264parse ! decodebin ! autovideosink sync=false
```

Z16 depth never enters this lossy video path. A UI depth stream first creates a normalized/colorized
image while the untouched metric data continues to depth processing.

RTSP timestamps are presentation timestamps only. Sensor synchronization uses timestamps preserved in
`CameraFrameSet`.
