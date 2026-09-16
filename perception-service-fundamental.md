# Perception Service — Fundamental Architecture & Development Guide

> **Repository:** `perception-service`  
> **Compute:** NVIDIA Jetson Orin  
> **OS:** Ubuntu 22.04  
> **Primary sensor:** Intel RealSense D435i  
> **Primary language:** C++20  
> **Build:** CMake + CMakePresets  
> **Primary AI coding agent:** OpenAI Codex  
> **Future coding agent:** Claude Code  
> **Mission counterpart:** `robot-agent` at `192.168.1.206`  
> **Document role:** Architecture baseline, engineering constitution, phased implementation roadmap.

---

# 1. Project Objective

`perception-service` is the local perception and localization subsystem of the quadruped robot platform.

It runs entirely on the NVIDIA Jetson Orin and is responsible for converting raw sensor measurements into reliable environment and localization state.

The service is **not** the mission controller.

The mission controller is a separate service:

```text
robot-agent
192.168.1.206
```

The fundamental ownership model is:

```text
Jetson Orin / perception-service
    = understand the local world
    = estimate robot motion / pose
    = publish perception + localization state
    = provide video streams for UI

robot-agent / 192.168.1.206
    = own mission lifecycle
    = own waypoint logic
    = own vendor robot API
    = own payload control
    = decide continue / slow / pause / recover / stop

Vendor quadruped
    = locomotion
    = balance
    = gait
    = immediate vendor obstacle avoidance
```

This boundary must remain stable as the system evolves.

---

# 2. System Context

```text
┌──────────────────────────── Jetson Orin ─────────────────────────────┐
│                                                                      │
│                       perception-service                             │
│                                                                      │
│   D435i                                                              │
│     │                                                                │
│     ▼                                                                │
│   Capture                                                            │
│     │                                                                │
│     ▼                                                                │
│   Internal FrameSet                                                  │
│     │                                                                │
│     ├──────────► Streaming ─────────────► RTSP ─────────► UI          │
│     │                                                                │
│     ├──────────► Depth Perception                                    │
│     │                                                                │
│     ├──────────► VO / VIO                                            │
│     │                                                                │
│     └──────────► EKF / State Estimation                              │
│                         │                                            │
│                         ▼                                            │
│                LocalizationState                                     │
│                PerceptionState                                       │
│                Health / Confidence                                   │
└─────────────────────────┬────────────────────────────────────────────┘
                          │
                          │ Ethernet / structured protocol
                          ▼
              ┌──────────────────────────────┐
              │ robot-agent                  │
              │ 192.168.1.206                │
              │                              │
              │ Mission Manager              │
              │ Waypoint Manager             │
              │ Vendor Robot API             │
              │ Payload Manager              │
              │ Mission Recovery Logic       │
              └──────────────┬───────────────┘
                             │
                             ▼
                    Vendor Quadruped Robot
```

Video streaming is a separate observability channel and is not the machine-data path used for mission logic.

---

# 3. Responsibility of `perception-service`

The Jetson service owns:

```text
RealSense D435i acquisition
RGB capture
IR stereo capture
Depth capture
IMU capture
Frame synchronization
Timestamp handling
Camera calibration / intrinsics / extrinsics
UI video restream
Depth preprocessing
Point cloud generation
Ground / free-space estimation
Terrain / slope / drop-off analysis
Visual Odometry
Visual-Inertial Odometry
Sensor fusion / EKF
Localization confidence
Perception health
Publishing structured perception/localization results
```

The service does **not** own:

```text
mission state machine
waypoint execution policy
vendor locomotion API
payload command orchestration
final motion authority
```

---

# 4. Responsibility of `robot-agent`

`robot-agent` at `192.168.1.206` is the high-level robot supervisor.

It owns:

```text
Mission start / pause / resume / stop
Waypoint sequencing
Mission recovery
Vendor quadruped API
Payload control
Robot telemetry aggregation
Safety policy at mission level
Decision logic based on localization confidence
Decision logic based on perception risk
```

Conceptually:

```text
perception-service:
    "Where am I?"
    "How did I move?"
    "What is around me?"
    "How trustworthy is this estimate?"

robot-agent:
    "What should the mission do now?"
```

---

# 5. Vendor Robot Boundary

The vendor quadruped already provides locomotion functionality.

Treat it as an intelligent mobile base:

```text
robot-agent
    │
    │ desired motion / vendor API command
    ▼
Vendor Quadruped
    ├── balance
    ├── gait
    ├── foot placement
    ├── motor control
    └── immediate obstacle avoidance
```

Do not create a competing low-level locomotion controller in `perception-service`.

---

# 6. D435i Sensor Roles

The D435i provides multiple channels:

```text
D435i
├── RGB
├── Infrared Left
├── Infrared Right
├── Depth
├── Accelerometer
└── Gyroscope
```

The stereo pair is:

```text
IR Left + IR Right
```

not two RGB cameras.

Long term, the sensor serves two independent functional branches.

```text
                         D435i
                           │
                 ┌─────────┴─────────┐
                 ▼                   ▼
          Localization Branch   Perception Branch
                 │                   │
          Stereo/RGB + IMU          Depth/RGB
                 │                   │
                 ▼                   ▼
              VO / VIO          Free Space
                 │              Ground
                 ▼              Terrain
           Local Odometry       Drop-off
                 │              Semantic data
                 ▼                   │
                EKF                  ▼
                 │            Environment State
                 └───────────┬───────┘
                             ▼
                   Structured Output
```

The two questions are different:

```text
Localization:
    Where am I / how did I move?

Perception:
    Can I safely go there?
```

---

# 7. Current Mission Problem

The current Level 1 system is strongly dependent on RTK/GNSS:

```text
RTK/GNSS
   ↓
Position
   ↓
Waypoint Navigation
   ↓
Vendor Robot
```

The problematic behavior is roughly:

```text
RTK FIXED
   ↓
mission continues

RTK FLOAT / DGNSS
   ↓
mission pauses
```

Level 2 should eliminate this binary dependency.

---

# 8. Level 2 Localization Goal

Future navigation should consume an estimated state instead of raw GNSS.

```text
GNSS / RTK ─────────────┐
                        │
D435i VO/VIO ───────────┼──► EKF / State Estimator ───► Estimated Pose
                        │
D435i IMU ──────────────┤
                        │
Vendor odometry ────────┘
```

Navigation should use:

```text
Estimated Pose + Covariance + Localization Quality
```

not:

```text
Raw GNSS Position only
```

---

# 9. Quality-Aware Localization

Do not use:

```cpp
if (rtk_status == FIXED)
    runMission();
else
    pauseMission();
```

Use localization quality states instead:

```text
HIGH
    GNSS FIXED
    VO/VIO healthy
    full confidence

MEDIUM
    GNSS FLOAT
    VO/VIO healthy
    local estimate dominant

LOW
    GNSS degraded
    covariance increasing
    reduced mission envelope

DEAD_RECKONING
    GNSS lost
    VO/VIO + IMU still healthy
    limited time / distance

CRITICAL
    GNSS poor
    VO/VIO poor
    mission should stop safely
```

The service reports quality.

`robot-agent` decides the mission behavior.

---

# 10. map / odom / base_link

Future localization should maintain explicit frame semantics.

```text
map
 │
 │ slow global correction
 ▼
odom
 │
 │ smooth continuous local motion
 ▼
base_link
```

`odom` should be driven mainly by continuous sources:

```text
VO/VIO
IMU
Vendor odometry
```

Properties:

```text
smooth
continuous
locally accurate
drifting over time
```

`map` is anchored by absolute data such as RTK/GNSS.

Properties:

```text
global
absolute
may degrade
may jump
may temporarily disappear
```

---

# 11. RTK Recovery Rule

When RTK returns from FLOAT/LOST to FIXED, do not snap the estimated pose.

Incorrect:

```text
Estimated X = 100.0
RTK X       = 100.8

100.0 ─────► 100.8
```

Preferred:

```text
RTK returns
   ↓
innovation check
   ↓
covariance evaluation
   ↓
EKF correction
   ↓
gradual convergence
```

This prevents false mission steering spikes caused by apparent pose teleportation.

---

# 12. V0.1 — First Concrete Objective

The first implementation phase should be intentionally simple:

> Capture the RealSense streams reliably on Jetson Orin and restream operator-visible video to the UI.

V0.1 is **not** yet an EKF release.

V0.1 is **not** yet a terrain-intelligence release.

Its job is to create the sensor and streaming backbone that all later perception work will reuse.

---

# 13. V0.1 Capture Requirements

Capture at minimum:

```text
RGB
IR Left
IR Right
Depth
```

Also prepare IMU capture from the beginning:

```text
Accelerometer
Gyroscope
```

Even if IMU is not consumed in V0.1, keeping the acquisition path ready prevents later architectural rework.

---

# 14. V0.1 Capture Pipeline

```text
D435i
  │
  ▼
librealsense2
  │
  ▼
RealSenseCamera
  │
  ▼
CameraFrameSet
```

After `RealSenseCamera`, no downstream module should depend on `rs2::*` types.

This is a critical architectural rule.

---

# 15. Internal Frame Types

Example:

```cpp
struct ImageFrame
{
    std::uint64_t timestamp_ns {};
    std::uint64_t frame_number {};

    int width {};
    int height {};

    cv::Mat image;
};
```

```cpp
struct DepthFrame
{
    std::uint64_t timestamp_ns {};
    std::uint64_t frame_number {};

    int width {};
    int height {};

    float depth_scale_m {};

    cv::Mat depth;
};
```

```cpp
struct AccelerometerSample
{
    std::uint64_t sensor_timestamp_ns {};
    std::uint64_t capture_timestamp_ns {};
    Eigen::Vector3f acceleration_mps2 {};
};

struct GyroscopeSample
{
    std::uint64_t sensor_timestamp_ns {};
    std::uint64_t capture_timestamp_ns {};
    Eigen::Vector3f angular_velocity_rps {};
};

struct ImuBatch
{
    std::vector<AccelerometerSample> accelerometer;
    std::vector<GyroscopeSample> gyroscope;
};
```

```cpp
struct CameraFrameSet
{
    std::uint64_t capture_timestamp_ns {};

    ImageFrame rgb;
    ImageFrame ir_left;
    ImageFrame ir_right;
    DepthFrame depth;

    std::optional<ImuBatch> imu;
};
```

The internal frame model is the contract between sensor acquisition and all later consumers.
Accelerometer and gyroscope samples retain independent timestamps and rates; do not fabricate a
same-time pair by combining whichever two values happened to arrive most recently.

---

# 16. Streaming Is a Separate Branch

Correct architecture:

```text
CameraFrameSet
     │
     ├────────► Streaming ─────► UI
     │
     ├────────► VO/VIO
     │
     └────────► Depth Perception
```

Do not make RTSP part of the perception algorithm itself.

Do not make `robot-agent` consume RTSP for mission logic.

---

# 17. Recommended V0.1 Streaming Technology

Use:

```text
GStreamer
+
gst-rtsp-server
+
Jetson hardware H.264 encoder
```

Conceptual pipeline:

```text
raw frame
   ↓
appsrc
   ↓
nvvidconv / format conversion
   ↓
NVMM
   ↓
nvv4l2h264enc
   ↓
h264parse
   ↓
rtph264pay
   ↓
gst-rtsp-server
```

Reasons:

```text
native C/C++ integration
mature RTP/RTSP stack
works well on embedded Linux
hardware encode on Jetson
low CPU use
compatible with VLC / LibVLCSharp / FFmpeg / GStreamer
simple LAN deployment
```

For the current desktop UI architecture, RTSP + H.264 is the preferred baseline.

---

# 18. Recommended RTSP Endpoints

```text
rtsp://<JETSON_IP>:8554/camera/rgb
rtsp://<JETSON_IP>:8554/camera/ir_left
rtsp://<JETSON_IP>:8554/camera/ir_right
```

Optional later:

```text
rtsp://<JETSON_IP>:8554/camera/depth_visual
rtsp://<JETSON_IP>:8554/debug/terrain
rtsp://<JETSON_IP>:8554/debug/vo
```

The debug streams are optional observability products, not algorithm inputs.

---

# 19. RTSP vs WebRTC

Use RTSP first when:

```text
LAN
Desktop UI
LibVLCSharp / VLC client
Simple point-to-point monitoring
Predictable network
```

Consider WebRTC later when:

```text
Browser UI
NAT traversal
Internet deployment
Adaptive bitrate
Very low interactive latency
Two-way media
```

WebRTC adds:

```text
signaling
ICE
STUN/TURN
session management
```

Therefore the default decision is:

```text
V0.1 = RTSP + H.264
```

---

# 20. Raw Depth Must Not Be Treated as H.264 Video

Metric depth is machine data.

Do not encode raw Z16 depth with lossy H.264 if later algorithms need exact depth values.

Use two different paths.

For UI:

```text
Depth Z16
   ↓
Normalize / colorize
   ↓
RGB visualization
   ↓
H.264
   ↓
RTSP
```

For perception:

```text
Depth Z16
   ↓
Depth Processing
   ↓
Point Cloud
   ↓
Terrain / Free Space / Drop-off
```

---

# 21. Timestamping and Synchronization

Preserve at minimum:

```text
sensor timestamp
capture timestamp
frame number
stream type
```

The localization/perception branch must preserve sensor time relationships.

Do not use RTSP presentation timestamps as the authoritative perception synchronization source.

---

# 22. Bounded Queues

All real-time queues must be bounded.

Recommended:

```text
Capture
   ↓
Queue size 2..3
   ↓
Consumer
```

If the consumer falls behind:

```text
drop oldest frame
```

Do not allow an unbounded backlog.

For robot systems:

> Fresh data is more valuable than complete stale data.

---

# 23. V0.1 Metrics

Measure:

```text
camera.capture_fps
camera.frames_received
camera.frames_dropped
camera.frame_age_ms

stream.rgb.fps
stream.rgb.frames_encoded
stream.rgb.frames_dropped

stream.ir_left.fps
stream.ir_right.fps

stream.encoder_latency_ms
```

Future metrics:

```text
vo.latency_ms
vo.tracking_quality
perception.depth_latency_ms
ekf.latency_ms
localization.covariance
```

---

# 24. V0.1 Failure Handling

Handle:

```text
camera missing at startup
USB disconnect
USB reconnect
stream timeout
encoder failure
RTSP client disconnect
RTSP client reconnect
SIGINT
SIGTERM
```

Desired behavior:

```text
camera lost
   ↓
health = degraded
   ↓
streaming pauses
   ↓
reconnect loop
   ↓
camera restored
   ↓
pipeline resumes
```

Ordinary camera reconnection should not require a process restart.

---

# 25. Initial Repository Structure

```text
perception-service/
├── AGENTS.md
├── README.md
├── CMakeLists.txt
├── CMakePresets.json
├── .gitignore
├── .clang-format
├── .clang-tidy
│
├── config/
│   ├── default.yaml
│   ├── d435i.yaml
│   └── streaming.yaml
│
├── docs/
│   ├── architecture.md
│   ├── setup.md
│   ├── streaming.md
│   └── inter-service.md
│
├── include/
│   └── perception/
│       ├── core/
│       │   ├── application.hpp
│       │   └── config.hpp
│       │
│       ├── camera/
│       │   ├── camera.hpp
│       │   ├── camera_types.hpp
│       │   └── realsense_camera.hpp
│       │
│       ├── streaming/
│       │   ├── stream_types.hpp
│       │   ├── stream_publisher.hpp
│       │   ├── rtsp_server.hpp
│       │   └── stream_manager.hpp
│       │
│       ├── pipeline/
│       │   └── capture_pipeline.hpp
│       │
│       └── health/
│           └── camera_health.hpp
│
├── src/
│   ├── core/
│   │   ├── application.cpp
│   │   └── config.cpp
│   │
│   ├── camera/
│   │   └── realsense_camera.cpp
│   │
│   ├── streaming/
│   │   ├── rtsp_server.cpp
│   │   └── stream_manager.cpp
│   │
│   ├── pipeline/
│   │   └── capture_pipeline.cpp
│   │
│   └── health/
│       └── camera_health.cpp
│
├── app/
│   ├── perception_service.cpp
│   └── tools/
│       ├── camera_info.cpp
│       ├── camera_viewer.cpp
│       └── stream_test.cpp
│
├── tests/
│   ├── unit/
│   └── integration/
│
├── scripts/
│   ├── setup.sh
│   ├── build.sh
│   ├── test.sh
│   ├── run.sh
│   └── test_rtsp.sh
│
└── deploy/
    └── systemd/
        └── perception-service.service
```

Do not create large numbers of empty future modules in V0.1.

---

# 26. Mature Repository Structure

Later:

```text
include/perception/
├── core/
├── camera/
├── streaming/
├── depth/
├── geometry/
├── imu/
├── odometry/
├── localization/
├── terrain/
├── obstacle/
├── health/
├── pipeline/
└── transport/
```

Equivalent implementation directories exist under `src/`.

`transport/` is the future structured communication layer with `robot-agent`.

---

# 27. Camera Abstraction

```cpp
class ICamera
{
public:
    virtual ~ICamera() = default;

    virtual bool initialize() = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;

    virtual bool capture(CameraFrameSet& frame_set) = 0;
};
```

Only the RealSense backend should directly use:

```text
rs2::pipeline
rs2::config
rs2::frameset
rs2::frame
```

---

# 28. Streaming Abstraction

```cpp
enum class StreamId
{
    Rgb,
    InfraredLeft,
    InfraredRight,
    DepthVisual
};
```

```cpp
class IStreamPublisher
{
public:
    virtual ~IStreamPublisher() = default;

    virtual bool start() = 0;
    virtual void stop() = 0;

    virtual bool publish(
        StreamId stream,
        const ImageFrame& frame) = 0;
};
```

Initial implementation:

```text
RtspServer : IStreamPublisher
```

Future alternatives can be added without modifying camera capture.

---

# 29. Configuration Example

```yaml
camera:
  type: realsense
  serial: ""

  rgb:
    enabled: true
    width: 640
    height: 480
    fps: 30

  depth:
    enabled: true
    width: 640
    height: 480
    fps: 30

  infrared_left:
    enabled: false
    width: 640
    height: 480
    fps: 30

  infrared_right:
    enabled: false
    width: 640
    height: 480
    fps: 30

  imu:
    enabled: true
    accelerometer_fps: 100
    gyroscope_fps: 200
    queue_capacity: 512

streaming:
  rtsp:
    enabled: true
    bind_address: "0.0.0.0"
    port: 8554
    codec: "h264"
    hardware_encoder: true

    rgb:
      enabled: true
      path: "/camera/rgb"

    infrared_left:
      enabled: false
      path: "/camera/ir_left"

    infrared_right:
      enabled: false
      path: "/camera/ir_right"

    depth_visual:
      enabled: true
      path: "/camera/depth_visual"
      min_distance_m: 0.2
      max_distance_m: 5.0

robot_agent:
  enabled: false
  host: "192.168.1.206"
  port: 0
```

The machine-data protocol and port remain intentionally undecided until the interface is designed.

---

# 30. V0.1 Milestones

## Milestone 0 — Repository Bootstrap

Deliver:

```text
C++20
CMake
CMakePresets
AGENTS.md
spdlog
yaml-cpp
GoogleTest
clang-format
clang-tidy
```

Acceptance:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

---

## Milestone 1 — Camera Discovery

Create:

```text
camera_info
```

Report:

```text
model
serial
firmware
USB mode
stream profiles
intrinsics
depth scale
```

---

## Milestone 2 — Multi-Stream Capture

Capture:

```text
RGB
IR Left
IR Right
Depth
IMU
```

Convert immediately to `CameraFrameSet`.

Acceptance:

> No `rs2::*` type leaves the camera subsystem.

---

## Milestone 3 — Local Viewer

Create:

```text
camera_viewer
```

Display:

```text
RGB
IR Left
IR Right
Depth Visual
```

Purpose:

> Validate acquisition locally before networking.

---

## Milestone 4 — Hardware RTSP

Publish:

```text
/camera/rgb
/camera/ir_left
/camera/ir_right
```

Test from another machine:

```bash
ffplay rtsp://<JETSON_IP>:8554/camera/rgb
```

Also test with VLC / LibVLCSharp.

Acceptance target:

```text
stable target FPS
bounded latency
hardware encoder active
client reconnect works
```

---

## Milestone 5 — Reliability

Implement:

```text
bounded queues
frame-drop metrics
camera timeout detection
USB reconnect
encoder restart
graceful shutdown
health state
```

Acceptance test:

```text
Start service
Unplug D435i
Reconnect D435i
Stream recovers without restarting process
```

---

## Milestone 6 — systemd

Deploy:

```text
perception-service.service
```

Target:

```text
Jetson boots
   ↓
service starts
   ↓
camera initialized
   ↓
RTSP available
```

---

# 31. V0.1 Definition of Done

```text
Jetson boots
    ↓
perception-service starts
    ↓
D435i initialized
    ↓
RGB + IR stereo + Depth + IMU acquired
    ↓
RTSP video available to UI
    ↓
latency / FPS metrics available
    ↓
client reconnect safe
    ↓
camera reconnect safe
```

No mission logic is required in V0.1.

---

# 32. V0.2 — Depth Foundation

Add:

```text
depth/
geometry/
```

Pipeline:

```text
Depth
  ↓
Invalid-depth removal
  ↓
Range clipping
  ↓
Temporal filtering
  ↓
Spatial filtering
  ↓
Optional hole filling
  ↓
Point Cloud
  ↓
Robot-frame transform
```

---

# 33. V0.3 — Environment Perception

Add:

```text
Ground estimation
Free-space estimation
Near-field obstacle risk
Drop-off detection
Slope
Roughness
Traversability
```

Example conceptual model:

```cpp
struct EnvironmentState
{
    bool ground_valid {};
    bool drop_off_detected {};

    float nearest_obstacle_m {};
    float slope_deg {};
    float roughness_score {};
    float traversability_score {};
    float confidence {};
};
```

These are machine-data outputs, not video.

---

# 34. Avoid Duplicating Vendor Obstacle Control

Do not initially create a second low-level avoidance controller.

Bad architecture:

```text
perception-service says TURN LEFT
vendor avoidance says TURN RIGHT
```

Possible result:

```text
controller conflict
oscillation
unpredictable motion
```

Preferred hierarchy:

```text
perception-service:
    "forward risk = high"

robot-agent:
    slow / pause / replan

vendor robot:
    execute locomotion + immediate reflex avoidance
```

---

# 35. V0.4 — VO / VIO

Add:

```text
odometry/
imu/
```

Pipeline:

```text
Stereo/RGB
    +
IMU
    ↓
VO / VIO
    ↓
Local Odometry
    ↓
Pose + Velocity + Covariance
```

Primary objective:

> Maintain continuous short-term motion estimate when RTK quality degrades.

---

# 36. V0.5 — EKF / State Estimation

Add:

```text
localization/
```

Inputs:

```text
GNSS / RTK
VO / VIO
D435i IMU
Vendor odometry
```

Output concept:

```cpp
struct LocalizationState
{
    std::uint64_t timestamp_ns {};

    Pose map_pose;
    Pose odom_pose;

    Velocity velocity;
    Covariance covariance;

    LocalizationQuality quality;
};
```

The EKF belongs on Jetson Orin as part of `perception-service`.

---

# 37. Localization Confidence Manager

```cpp
enum class LocalizationQuality
{
    High,
    Medium,
    Low,
    DeadReckoning,
    Critical
};
```

Example interpretation:

```text
HIGH
    RTK FIXED
    VO/VIO OK

MEDIUM
    RTK FLOAT
    VO/VIO OK

LOW
    GNSS degraded
    covariance rising

DEAD_RECKONING
    GNSS absent
    VO/VIO + IMU still valid

CRITICAL
    global and local localization unreliable
```

`robot-agent` consumes this state and applies mission policy.

---

# 38. Dead-Reckoning Budget

VO/VIO is not a permanent replacement for GNSS.

Track:

```text
time since last reliable global fix
distance since last reliable global fix
VO tracking quality
localization covariance
IMU health
```

Concept:

```text
GNSS degraded
    ↓
start dead-reckoning budget
    ↓
VO/VIO continues
    ↓
covariance grows
    ↓
confidence acceptable?
    ├── YES → continue reporting degraded localization
    └── NO  → report CRITICAL
```

`robot-agent` decides whether mission continuation is still allowed.

---

# 39. Inter-Service Communication

Future communication is bidirectional:

```text
perception-service
       ↕
robot-agent
192.168.1.206
```

This interface is completely separate from RTSP.

---

# 40. perception-service → robot-agent

Candidate messages:

```text
LocalizationState
LocalizationQuality
PerceptionState
EnvironmentState
SensorHealth
ServiceHealth
```

Example conceptual JSON only:

```json
{
  "timestamp_ns": 0,
  "localization": {
    "quality": "MEDIUM",
    "x_m": 0.0,
    "y_m": 0.0,
    "yaw_rad": 0.0,
    "covariance_xy": 0.0
  },
  "environment": {
    "nearest_obstacle_m": 2.4,
    "drop_off": false,
    "traversability": 0.91
  },
  "health": {
    "camera": "OK",
    "vo": "OK",
    "ekf": "OK"
  }
}
```

The final schema should be typed and versioned.

---

# 41. robot-agent → perception-service

Possible future inputs:

```text
RTK position
RTK fix type
GNSS covariance
GNSS age
Vendor odometry
Robot velocity
Body state
Mission context
Time synchronization data
```

If these measurements are physically connected to or already normalized by `robot-agent`, forward them to Jetson for fusion.

The ownership rule remains:

```text
measurement source can be remote
processing / fusion remains on Jetson
```

---

# 42. Transport Protocol

Do not reuse RTSP for machine state.

The structured interface needs:

```text
typed messages
timestamps
schema versioning
low latency
bidirectional communication
reconnect behavior
health/status
back-pressure policy
```

Candidates to evaluate later:

```text
gRPC + Protobuf
ZeroMQ
ROS2 DDS
TCP + Protobuf
UDP for selected high-rate telemetry
```

Do not commit prematurely.

The core models must be transport-independent.

---

# 43. Internal Output Models

Example:

```cpp
struct PerceptionState
{
    std::uint64_t timestamp_ns {};
    EnvironmentState environment;
    SensorHealth sensor_health;
};
```

```cpp
struct ServiceHealth
{
    CameraStatus camera;
    StreamingStatus streaming;
    VoStatus vo;
    EkfStatus ekf;
};
```

Transport adapters serialize project-defined models.

Algorithms should never depend on protobuf, ROS or socket types.

---

# 44. Future Mission Loop

```text
perception-service
      │
      ├── LocalizationState
      ├── PerceptionState
      └── Health
              │
              ▼
           Network
              │
              ▼
         robot-agent
              │
      ┌───────┼────────┐
      ▼       ▼        ▼
   Mission  Waypoint  Payload
   Manager  Manager   Manager
      │
      ▼
 Mission Safety Policy
      │
      ▼
 Vendor Robot API
      │
      ▼
 Quadruped Robot
```

There must be one final mission command authority: `robot-agent`.

---

# 45. Level 2 Acceptance Test

```text
MISSION START

RTK FIXED
   ↓
robot follows waypoint
   ↓
RTK becomes FLOAT
   ↓
mission does not immediately stop
   ↓
VO/VIO + IMU dominate local localization
   ↓
perception-service reports degraded confidence
   ↓
robot-agent applies degraded-mode mission policy
   ↓
robot continues within configured safety budget
   ↓
RTK returns FIXED
   ↓
outlier/jump rejected or down-weighted
   ↓
EKF corrects global pose smoothly
   ↓
mission continues
   ↓
waypoint reached
```

This is the key Level 2 outcome.

---

# 46. Level 2A and 2B

## Level 2A — Localization Fallback

```text
D435i VO/VIO
   +
IMU
   +
RTK/GNSS
   +
Vendor odometry
   ↓
EKF
   ↓
Continuous localization
```

Goal:

> RTK FIXED → FLOAT → FIXED without mandatory mission stop.

## Level 2B — Depth Perception

```text
Depth
   ↓
Free Space
   ↓
Ground / Terrain / Drop-off
   ↓
Navigation Safety State
```

Goal:

> Give `robot-agent` richer local mission awareness than GNSS alone.

---

# 47. Logging

Use `spdlog`.

Avoid `printf` and `std::cout` in production modules.

Example:

```text
[INFO] perception-service starting
[INFO] D435i detected serial=...
[INFO] RGB 640x480@30
[INFO] IR Left 640x480@30
[INFO] IR Right 640x480@30
[INFO] Depth 640x480@30
[INFO] RTSP listening on 0.0.0.0:8554
[WARN] frame age exceeded threshold
[ERROR] RealSense disconnected
[INFO] reconnecting RealSense
[INFO] RealSense restored
```

Future:

```text
[WARN] RTK FIXED -> FLOAT
[INFO] localization mode = VO_DOMINANT
[WARN] covariance rising
[INFO] RTK FIXED recovered
[INFO] global correction converging
```

---

# 48. Health Monitoring

Health is first-class data.

Track:

```text
camera connection
camera FPS
frame age
stream FPS
stream latency
encoder status
CPU
GPU
memory
temperature
VO quality
EKF innovation
GNSS age
localization covariance
```

A running process is not necessarily a healthy perception subsystem.

---

# 49. Dependencies

Initial baseline:

```text
C++20
CMake
librealsense2
OpenCV
Eigen3
yaml-cpp
spdlog
GoogleTest
GStreamer
gst-rtsp-server
```

Future only when justified:

```text
CUDA
TensorRT
PCL
ROS2
gRPC
Protobuf
ZeroMQ
MCAP
```

---

# 50. ROS2 Policy

ROS2 must not become the core internal data model.

Preferred:

```text
perception core
    │
    ├── standalone service
    ├── robot-agent transport adapter
    └── ROS2 adapter [future]
```

If ROS2 is later used for `robot_localization`, tooling, visualization or integration, keep it outside core algorithm interfaces.

---

# 51. Dataset and Replay

Record enough information to reproduce field failures:

```text
RGB
IR Left
IR Right
Depth
IMU
timestamps
intrinsics
extrinsics
GNSS
vendor odometry
localization state
```

Workflow:

```text
field issue
   ↓
record dataset
   ↓
replay offline
   ↓
reproduce
   ↓
fix
   ↓
regression test
```

Large datasets must not be committed to Git.

---

# 52. Threading Model

V0.1:

```text
Capture Thread
    │
    ▼
CameraFrameSet
    │
    └── bounded streaming queue
             │
             ▼
         GStreamer
```

Later:

```text
Capture
   │
   ├── Streaming
   ├── VO/VIO
   └── Depth Processing
            │
            ▼
          Fusion
```

Every queue must remain bounded.

---

# 53. Coding Rules

Use:

```text
C++20
RAII
std::unique_ptr for ownership
std::chrono for time
std::filesystem for paths
enum class
[[nodiscard]] where meaningful
explicit units in names
```

Avoid:

```text
raw owning pointers
global mutable state
hard-coded runtime settings
unbounded queues
SDK types leaking into algorithms
network types leaking into core models
printf / std::cout in production
```

Prefer:

```cpp
double distance_m;
std::chrono::milliseconds timeout;
```

not:

```cpp
double distance;
int timeout;
```

---

# 54. AI Coding Agent Rules

`AGENTS.md` is the engineering constitution for Codex.

Critical rules:

```text
1. Do not change architectural boundaries without explicit instruction.
2. rs2::* remains inside camera backend.
3. GStreamer types remain inside streaming subsystem.
4. Core perception models are transport-independent.
5. robot-agent communication is separate from RTSP.
6. perception-service does not own vendor mission control.
7. EKF/localization processing remains on Jetson.
8. Add tests for new algorithms.
9. Do not add dependencies without justification.
10. Keep changes small and reviewable.
```

Future `CLAUDE.md` should reference the same architecture instead of duplicating it.

---

# 55. Recommended AGENTS.md Core

```markdown
# AGENTS.md

## Project

perception-service runs on NVIDIA Jetson Orin / Ubuntu 22.04.

Primary sensor:
- Intel RealSense D435i

Mission counterpart:
- robot-agent
- 192.168.1.206

## Ownership

perception-service owns:
- D435i capture
- RGB/IR/depth/IMU processing
- RTSP video for UI
- depth perception
- VO/VIO
- EKF/localization
- localization confidence
- perception health

robot-agent owns:
- mission execution
- waypoint state machine
- vendor quadruped API
- payload control
- final mission action

## Architecture

Camera SDK
    ↓
Camera abstraction
    ↓
Internal FrameSet
    ├── Streaming
    ├── Depth Perception
    └── VO/VIO
             ↓
            EKF
             ↓
      Localization State
             ↓
    Transport Adapter
             ↓
       robot-agent

RTSP is only for UI/operator video.

## Dependency boundaries

- rs2::* only in camera backend
- GStreamer types only in streaming subsystem
- transport types only in adapters
- algorithms use project-defined types

## Coding

- C++20
- RAII
- no raw owning pointers
- no global mutable state
- std::chrono
- std::filesystem
- config, not hard-coded parameters
- spdlog
- warnings as errors
- bounded queues

## Build

cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

---

# 56. Development Roadmap

## V0.1 — Sensor + UI Streaming Foundation

```text
D435i
 ↓
RGB + IR Stereo + Depth + IMU
 ↓
Internal CameraFrameSet
 ↓
RTSP H.264
 ↓
UI
```

Deliverables:

```text
camera discovery
multi-stream capture
local viewer
hardware H.264
RTSP endpoints
metrics
USB reconnect
systemd
```

## V0.2 — Depth Foundation

```text
Depth preprocessing
 ↓
Point Cloud
 ↓
Coordinate Transform
```

## V0.3 — Local Environment Perception

```text
Ground
Free Space
Obstacle Risk
Drop-off
Slope
Roughness
Traversability
```

## V0.4 — VO/VIO

```text
Stereo/RGB + IMU
 ↓
VO/VIO
 ↓
Local Odometry
 ↓
Covariance
```

## V0.5 — EKF / Robust Localization

```text
GNSS
 +
VO/VIO
 +
IMU
 +
Vendor Odom
 ↓
EKF
 ↓
map / odom / base_link
 ↓
Localization Confidence
```

## V0.6 — robot-agent Integration

```text
perception-service
      │
      ├── LocalizationState
      ├── PerceptionState
      └── Health
              │
              ▼
        robot-agent
        192.168.1.206
              │
              ▼
        Mission Logic
              │
              ▼
        Vendor Robot API
```

## V1.0 — Mission-Ready Perception

```text
D435i
  │
  ├── Streaming ─────────────► UI
  │
  ├── VO/VIO
  │      │
  │      ▼
  │   Local Odom
  │      │
  ├── Depth Perception
  │      │
  │      ▼
  │ Environment State
  │
  └───────────────┐
                  ▼
                EKF ◄──── GNSS
                  ▲
                  └────── Vendor Odom
                  │
                  ▼
         Localization State
                  │
                  ▼
             robot-agent
                  │
                  ▼
          Mission Manager
                  │
                  ▼
         Vendor Quadruped API
```

---

# 57. Final Engineering Principles

1. All D435i perception/localization processing stays on Jetson Orin.
2. `robot-agent` at `192.168.1.206` is the mission authority.
3. Video streaming is for UI/observability, not mission machine data.
4. Machine-to-machine data must be structured and versioned.
5. RealSense SDK types must not leak beyond the camera backend.
6. GStreamer types must not leak beyond the streaming subsystem.
7. Raw metric depth must not be treated as ordinary lossy video.
8. VO/VIO and depth perception are separate functional branches.
9. EKF/state estimation belongs inside `perception-service` on Jetson.
10. Navigation should eventually consume fused estimated pose, not raw GNSS.
11. RTK FLOAT is degraded localization, not automatically lost localization.
12. RTK recovery must be fused smoothly, never snapped.
13. `map`, `odom`, and `base_link` semantics must remain explicit.
14. All real-time queues are bounded.
15. Fresh data is more valuable than stale complete data.
16. Confidence/covariance and health are first-class outputs.
17. Vendor robot retains gait, balance and immediate locomotion control.
18. Do not create a competing low-level obstacle controller.
19. Field replay and reproducibility are mandatory.
20. Core algorithms remain transport-independent.
21. Codex and Claude Code must follow repository boundaries.

---

# 58. Final Architectural Statement

The product of `perception-service` is not simply a camera feed.

Its real product is:

```text
Reliable local environmental understanding
+
Continuous local motion / pose estimation
+
Localization confidence
+
Known sensor and processing health
```

The system boundary is:

```text
                 SENSOR DATA
                     │
                     ▼
              perception-service
                     │
         ┌───────────┴───────────┐
         ▼                       ▼
 Human Observability       Machine Intelligence
         │                       │
       RTSP                PerceptionState
         │                 LocalizationState
         ▼                 Health / Confidence
        UI                       │
                                 ▼
                           robot-agent
                           192.168.1.206
                                 │
                                 ▼
                           Mission Logic
                                 │
                                 ▼
                       Vendor Quadruped API
```

The single most important architecture rule is:

> **Jetson Orin understands the world and estimates robot state.**  
> **`robot-agent` decides what the mission should do.**  
> **The vendor quadruped executes locomotion.**

This boundary should remain stable from V0.1 through V1.0.
