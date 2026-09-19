# Geometry and dataset replay

Shared foundation for both branches of the roadmap: the depth/perception branch needs robot-frame
points, and the localization branch needs the same mount extrinsics and the ability to replay a
recording offline. Neither belongs to one milestone, so both live here.

## Frames

```text
camera_optical  x right, y down, z forward   (the optical convention the sensor reports in)
      | fixed axis swap, applied in code
camera_body     x forward, y left, z up      (robot convention, still at the lens)
      | configured mount: geometry.camera_mount
base_link       robot body frame
```

The optical-to-body swap is a fixed rotation and is deliberately **not** configurable: making it a
setting invites someone to "fix" a wrong-looking cloud by rotating the mount instead, which leaves
two errors that cancel only for one orientation. Forgetting the swap rotates every point by 90
degrees, which is why the camera contributes two frames rather than one.

`geometry.camera_mount` describes `camera_body` relative to `base_link`: translation in metres and
Z-Y-X Euler angles in degrees, the same convention as the vendor's Roll/Pitch/Yaw report. These are
survey values, not tuning knobs. Configuration validation rejects angles outside +/-360 degrees as
data-entry errors.

`Transform3` is project-owned 3x3 rotation plus translation rather than Eigen: the existing frame
types already carry plain arrays, and keeping it dependency-free keeps the host test build free of
extra packages. Eigen enters with the EKF, where its solvers earn the dependency. Rotations are
inverted by transposing, never by general matrix inversion. `apply` rotates and translates a point;
`rotate` is separate because velocities and directions must not pick up the translation.

## Point clouds

`deproject_depth` turns one metric depth frame into points in `camera_optical`, and
`transform_in_place` moves a cloud into another frame and records the new `frame_id` on the cloud
itself, so a consumer cannot silently mix optical-frame and robot-frame points.

Zero in Z16 is librealsense's "no measurement", never a point at the sensor origin; it is dropped
and counted in `rejected_invalid`. Readings outside `geometry.point_cloud` range are counted in
`rejected_out_of_range`. Both counters exist so an empty cloud can be told apart from a cloud that
was never populated.

Distortion coefficients are reported by `has_distortion` but not applied: the D435i reports depth
with an unmodified Brown-Conrady model whose coefficients are zero, and applying a model the sensor
did not use would add error rather than remove it.

Filtering (invalid-depth removal beyond the zero test, clipping, temporal and spatial filters) is
V0.2 and is not implemented here.

## Dataset format, version 1

```text
<directory>/manifest.json   device, intrinsics, mount extrinsics, creation time, notes
<directory>/index.jsonl     one JSON object per frame set, in capture order
<directory>/blob.bin        concatenated raw payloads, referenced by offset and length
```

Image and depth payloads are stored byte-for-byte as captured. Metric Z16 is never re-encoded: it is
measurement data, not video, and a round-trip test asserts bit-exactness including the `65535` edge
value. Accelerometer and gyroscope samples keep their own timestamps in the index; the capture
design forbids fabricating a same-time pair, and the dataset must not reintroduce one.

The writer refuses to open a directory that already holds a dataset. Recordings are field evidence,
not scratch files.

Intrinsics are copied into the manifest at record time so a replay can deproject with no camera
present. `dataset-record` warns when the device reported no depth intrinsics, rather than leaving
that to be discovered offline.

A bare name given to the helper script records into the project's `datasets/` directory, which is
already excluded from Git and from the rsync deploy; an absolute path is used as given, for
recording onto external storage. Datasets are large and belong under ignored storage, never in Git.

## Recording

One process owns the D435i, so the service must release it first. Running the recorder while the
service streams fails with `failed to claim usb interface, interface 0, is busy` followed by a
power-state error — the camera is present, just held. The helper script stops the service, records,
restarts it and verifies, restoring streaming even if the recording fails or is interrupted:

```bash
./scripts/jetson/record-dataset.sh --confirm-service-interruption 2026-09-19-yard 30
```

To drive the recorder directly, stop the service yourself first:

```bash
./build/jetson-local/dataset-record config/default.yaml datasets/2026-09-19-yard 30
```

A recording costs roughly **45 MB/s** (about 1.5 MB per frame set at 640x480 RGB + depth, 30 fps),
so 30 s is about 1.3 GB and a minute is about 2.6 GB. The helper script checks free space against
that rate before it stops the service. Datasets are payload, not logs: budget storage accordingly.

The recorder reads intrinsics from device discovery and stores them in the manifest. librealsense
names streams `Color` and `Depth`, so selection is case-insensitive; it also warns when the stored
intrinsics belong to a resolution other than the configured one, because their focal length and
principal point would then be quietly wrong for deprojection.

It records for the given number of seconds, or until Ctrl+C, printing frame and payload counts every
second.

## Replay

`ReplayCamera` implements the same `ICamera` interface as the live camera, so everything downstream
of capture — depth processing, VO/VIO, localization — runs unchanged against a recording, with no
hardware. `loop` restarts at the first frame; `real_time` paces frames by their recorded capture
timestamps instead of returning immediately, capped at one second so a gap in a recording cannot
stall a run. A finished dataset reports exhaustion rather than repeating its last frame, which is how
a replay run terminates.

`dataset-info` verifies a recording offline without a camera: it replays through the same `ICamera`
path the live service uses, prints the manifest, frame count and measured mean FPS, then deprojects
one frame with the manifest's own intrinsics and transforms it into `base_link`. It exits non-zero
when the dataset has no depth intrinsics or produces no points, so "the recording is usable" is a
check rather than an assumption:

```bash
./build/jetson-local/dataset-info datasets/2026-09-19-yard 0
```

The workflow this exists for: field issue -> record dataset -> replay offline -> reproduce -> fix ->
keep the dataset as a regression test.
