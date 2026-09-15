# Inter-service communication

`perception-service` and `robot-agent` communicate over a future bidirectional typed protocol. The
interface is independent of RTSP and its final transport has deliberately not been selected.

Candidate outbound models are `LocalizationState`, `LocalizationQuality`, `PerceptionState`,
`EnvironmentState`, `SensorHealth` and `ServiceHealth`. Candidate inbound measurements are GNSS/RTK,
GNSS covariance and age, vendor odometry, robot velocity/body state and time-synchronization data.

The wire protocol must provide timestamps, schema versioning, reconnect behavior, health and a defined
back-pressure policy. gRPC/Protobuf, TCP/Protobuf, ZeroMQ, ROS2 DDS and selective UDP remain candidates.
Regardless of transport, fusion remains on Jetson and core models remain transport-independent.

The configured host is `192.168.1.206`. Its port remains zero and integration remains disabled until a
typed, versioned contract is approved.
