# Inter-service communication

`perception-service` (Jetson Orin, `192.168.1.220`) exchanges typed data with two mission-side
peers on separate Ubuntu machines. Protocol version 1 is defined here and in
`include/perception/transport/protocol.hpp`; the decision record is `decisions/0000_INDEX.md`
entry 0041 in the monorepo root.

RTSP remains an operator-observability channel only. Nothing on this boundary carries video.

## Transport choice

JSON over TCP, one request per connection: the caller connects, writes one JSON object,
half-closes its write side, and reads one JSON object terminated by `\n`. This is deliberately the
same framing `payload-service` already uses, so `robot-agent` can reuse
`internal/grpc/client/payload_client.py` with a different host and port instead of gaining a second
client style. `proto/` in the monorepo holds five-line skeletons only; gRPC was never implemented
here and is not required by this contract.

Every reply carries `protocol_version`. A request that declares a different version is refused with
an error instead of being coerced. Every measurement carries its own source timestamp and age,
because a state estimator cannot use a value whose age it cannot see.

## Endpoints

```text
perception-service  192.168.1.220:50053  JSON/TCP  server (robot-agent calls it)
payload-service            <host>:50052  JSON/TCP  polled by perception for GNSS/RTK
robot-agent                <host>:5080   HTTP/JSON polled by perception for vendor heading
```

`perception-service` is a server for state and a client for measurements. Both poll clients are
optional at runtime: a peer that stops answering degrades localization quality and must never stall
capture or streaming, exactly as a silent IMU must never stop RGB/depth.

## Inbound: what perception reads

**GNSS/RTK from payload-service, not through robot-agent.** GNSS lives in payload-service's
`rtk_gnss` device; robot-agent only proxies it. Polling payload-service directly keeps the sample's
own `timestamp_unix_ms`, `age_ms` and per-field freshness intact, which a second hop would blur.

```json
{"action": "gnss.get_status"}
```

The reply envelope is payload-service's own: `{"status":"ok","connected":true,"sample_count":N,
"fix":{...}|null}`. A null `fix` means "no fix yet", which is normal, not a transport fault.
`decode_gnss_status` reads position, course, heading, `fix_mode` (4 = `rtk_fixed`, 5 = `rtk_float`),
satellites, HDOP, speed, and the optional v2 freshness block (`position_valid`, `fix_mode_valid`,
`heading_valid` and their per-field ages). Older payload-service builds omit the freshness block;
the sample stays usable with sample-level timestamp and age only.

**Vendor heading from robot-agent.** `GET /api/v1/status` returns the cached vendor stream snapshot.
`decode_vendor_motion` reads `streams.motion_status`, requires `received_at_utc` (a sample whose age
is unknown must not reach the estimator), and accepts both the nested `Items.MotionStatus` and the
flat `Items` shape, as robot-agent and RoboStation both do. Angles and rates are RADIANS on the
wire; a bare pass-through puts a ~57x error into heading.

A live capture from robot `192.168.1.206` on 2026-09-19 fixed the decoded field set:

| Field | Meaning | EKF use |
| --- | --- | --- |
| `Roll`, `Pitch`, `Yaw` | body attitude, rad | heading, attitude constraint |
| `OmegaZ` | yaw rate, rad/s | yaw propagation |
| `LinearX`, `LinearY` | body-frame velocity, m/s | vendor odometry |
| `Height` | body height, m | context only |

Yaw is required; every other field is optional and flagged (`has_attitude`, `has_velocity`,
`has_yaw_rate`, `has_height`), so firmware that omits one degrades that single input instead of
discarding a valid heading. A half-present velocity pair is not a velocity. An exact `0.0` velocity
from a stationary robot is a measurement, not a missing field.

`source_time` sits beside `received_at_utc` but is robot-local wall time — observed 8 h ahead of UTC
— and is never used as an age. `streams.position_status` is empty on this robot (zero frames
received), so `motion_status` is the vendor odometry source.

## Outbound: what perception serves

| Action | Reply |
| --- | --- |
| `service.health` | service identity plus the health block |
| `perception.get_health` | camera FPS/drops, accel/gyro Hz and age, `imu_ready`, `ekf_ready` |
| `perception.get_localization` | `LocalizationState`: `available`, `timestamp_unix_ms`, `quality`, `map_pose`, `odom_pose`, velocity, covariance, dead-reckoning elapsed/budget |
| `perception.get_environment` | V0.3 output; replies `available: false` until that branch exists |
| `perception.set_map_origin` | sets the `map` origin from the mission's first waypoint |

`quality` is `global_fix`, `dead_reckoning` or `lost`. Until the EKF exists and has converged,
`perception.get_localization` reports `available: false` and `quality: "lost"` rather than a
fabricated zero pose, so `robot-agent` can be written against the final shape immediately.

## Number form on the wire

Real-valued fields always serialize with a decimal point (`0.0`, never `0`); integer fields stay
integers. This is enforced by the project-owned JSON writer and covered by unit tests. The rule is
not cosmetic: the 2026-07-23 Motion Control Yaw failure was caused solely by integer zeroes
replacing working float zeroes. Object field order is preserved as written for the same reason.

The codec is project-owned rather than a third-party parser precisely because this lexical form and
field order are part of the contract, and because host tests must keep running with no added
dependency.

## Configuration

`transport:` in `config/default.yaml`. The listener is enabled — it is inert until something calls
it. Both poll clients ship disabled on purpose: each adds a 5 Hz request load to a live mission
service (payload-service serves the robot's RTK, robot-agent runs missions) for a consumer that does
not exist until the V0.5 EKF, so they are enabled deliberately and per test. Validation
rejects a transport port equal to the RTSP port, a poll interval above 5 s, a request timeout more
than five times its poll interval, and a staleness timeout below the poll interval.

## Runtime behavior

The listener runs on its own thread and serves one connection at a time. A slow or hostile client
can therefore delay other clients of port 50053, but never capture, RTSP or the poll threads. Each
connection gets `transport.request_timeout_ms` for its read and write, requests above
`transport.max_request_bytes` are refused, and both framings are accepted: a caller that half-closes
and a caller that terminates its request with a newline. Shutdown is immediate — a self-pipe wakes
the accept loop rather than waiting for the next client.

Each enabled peer gets one poll thread that keeps only the newest sample. There is no inbound queue
by design: a stale measurement has no value to a state estimator. A measurement is reported as
unavailable when it has not arrived within `staleness_timeout_ms`, and GNSS is additionally rejected
when the peer's own `age_ms` exceeds that budget — a sample that arrived promptly but was already old
at the source is just as unusable. A peer that stops answering increments its failure counter and
degrades localization quality; it never stalls the service. A peer answering normally with no fix yet
counts as a request, not a failure.

Per-interval journal lines report `transport.listening`, requests served/rejected, connection errors
and, for each peer, `enabled`, `fresh`, `samples`, `failures` and `age_ms`. Peer response reads are
bounded at 1 MiB, separately from the request limit, because robot-agent's status snapshot carries
every vendor stream.

Sockets are POSIX-only. On a platform without them (the Windows host build) `start()` returns false
with a logged warning and the service continues; the contract and both decoders remain fully
host-testable because `RequestRouter` and the HTTP framing helpers touch no socket.

## Status

Protocol version 1 is implemented and deployed to the Jetson: the contract, both inbound decoders
(the vendor one pinned to a live capture), the request router, the listener on 50053, the two poll
threads, the health bridge over the V0.1 camera snapshot, the configuration and host tests. One thing
remains: `LocalizationState` stays `available: false` until the V0.5 EKF fills it. See
[roadmap](roadmap.md) V0.5.
