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

Field recordings must preserve RGB, both IR streams, metric depth, IMU, calibration and timestamps;
later phases add GNSS, vendor odometry and estimator output. Large datasets belong under ignored
storage, never in Git. Every field failure should become an offline replay regression test.
