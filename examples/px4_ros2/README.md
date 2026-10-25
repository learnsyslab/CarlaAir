# PX4 SITL + ROS 2 with CarlaAir

Fly a CarlaAir drone with PX4 SITL in lockstep and publish its sensors on
ROS 2 (tested with ROS 2 Jazzy / Python 3.12 and the v0.1.7 release).

| File | Purpose |
|---|---|
| `run_carlaair.sh` | Start the packaged CarlaAir with a given AirSim `settings.json` |
| `px4/make_px4_rootfs.sh` | Build a PX4 `etc` overlay that works with AirSim's HIL link |
| `px4/22180_airsim_quad` | PX4 airframe for AirSim's quadrotor |
| `px4_sensor_bridge.py` | ROS 2 node: `/clock`, IMU, images, ground truth |
| `airsim_rpc.py` | Minimal AirSim msgpack-RPC client (the `airsim` pip package does not run on Python 3.12) |
| `focus_guard.py` | Keeps keyboard focus off the CarlaAir window (view-only window) |
| `config/example_*` | Example AirSim settings and bridge configuration |

## Run

```bash
# 1. CarlaAir first: AirSim is the TCP server on port 4560.
bash run_carlaair.sh --settings config/example_settings.json --map Town10HD
# 2. PX4 (built with the none_iris SITL target) from an overlay of its etc/.
bash px4/make_px4_rootfs.sh <PX4>/build/px4_sitl_default/etc /tmp/px4_rootfs
(cd /tmp/px4_run && PX4_SIM_MODEL=airsim_quad <PX4>/build/px4_sitl_default/bin/px4 \
   -d -s /tmp/px4_rootfs/init.d-posix/rcS /tmp/px4_rootfs)
# 3. Sensors on ROS 2; MAVROS on udp://:14540@127.0.0.1:14557 as usual.
python3 px4_sensor_bridge.py --config config/example_bridge.yaml
```

Requires `python3-msgpack`, `python3-numpy` and `python3-yaml`.

## Things that are not obvious

- **One PX4 connection per session.** AirSim accepts PX4 once; restart
  CarlaAir and PX4 together.
- **Settings file.** The release ignores `-settings=`; `run_carlaair.sh` runs
  the engine with a private `HOME` that holds `Documents/AirSim/settings.json`.
- **Stopping.** The engine ignores SIGTERM; stop it with SIGKILL.
- **Vulkan device lost (Xid 31).** Unthrottled rendering crashed the GPU;
  `run_carlaair.sh` enables VSync and a 60 FPS limit.
- **Built-in keyboard flight.** WASD/Space/Shift in the CarlaAir window start
  CarlaAir's own drone control, which fights PX4 and can abort the engine
  ("Vehicle does not have a valid GPS home location"). Run `focus_guard.py
  <engine pid>` or use `--offscreen`.
- **PX4 overlay** (`make_px4_rootfs.sh`):
  - AirSim provides one IMU, so the multi-IMU EKF2 defaults are removed;
  - `simulator_mavlink` starts in the foreground: AirSim sends epoch
    timestamps, and work items scheduled before that time jump never run
    (manual control stays at 0 Hz, so POSCTL cannot climb);
  - an extra MAVLink instance (`-u 14581 -o 14541`) serves AirSim's control
    link (`ControlPortLocal/Remote` in settings), which gives AirSim the home
    position.
- **Magnetometer.** Far from AirSim's default origin PX4 refused to arm
  ("heading estimate not stable"); keep `OriginGeopoint` at the default.
- **Camera FOV.** `CaptureSettings.FOV_Degrees` is not applied; the bridge sets
  it with `simSetCameraFov` (`cameras.fov`).
- **Image latency.** AirSim stamps images when it returns them, about 76 ms
  after rendering at 960x600 on an RTX 5090. Set `cameras.latency_ms`; measure
  it by fitting the image rotation rate (homographies) against the gyro.
- **Clock.** Lockstep runs at `ClockSpeed` 1.0; `simContinueForTime` hangs
  with a PX4 vehicle.
- **Spawn position.** Vehicle `X/Y/Z` are NED metres from the map's
  PlayerStart. In Town10HD, `airsim = carla + (172.20, -183.86)` and
  `airsim_z = -carla_z + 27.45` (see `COORDINATE_SYSTEMS.md`).
