# Sawyer + PS2 / USB Gamepad Teleop

ROS Noetic workspace for teleoperating a Rethink Robotics **Sawyer** arm with a
**PS2-style USB gamepad**, running fully inside Docker.

Pipeline: `Gamepad (sticks + buttons)` → `RelaxedIK` → `intera_interface` → `Sawyer`.

Hold **L1** or **L2** while moving the sticks to drive the arm. Press **Y** to open/close the gripper.

## Control scheme

| Input | Action |
|-------|--------|
| Hold **L1** + left stick | move end-effector in **X / Y** |
| Hold **L1** + right stick | **Z** (up/down) and **yaw** |
| Hold **L2** + right stick | **roll / pitch** (Z frozen) |
| **Y** | toggle gripper open / close |
| **A** | (startup) move to HOME first, safely |
| **B** | (startup) skip home, start from current position |
| **Start** | return to HOME mid-session |
| **Select** | re-anchor IK at current position |
| **X** | reset orientation to home |
| keyboard `r` | same as **Start** |

## RUN the code

#### step 1: clone the repo and its dependencies

The teleop needs the RelaxedIK solver and (optionally) the Robotiq gripper driver:

```bash
export REPO_PATH=$HOME/ps2_sawyer_ws
git clone https://github.com/dqhuangnus/Sawyer_teleop_with_ps2-controller.git $REPO_PATH
cd $REPO_PATH
```

#### step 2: build the Docker image

The Dockerfile lives at `src/SAWYER/Dockerfile` and builds ROS Noetic + the Intera SDK +
Sawyer MoveIt + pygame:

```bash
docker build -t sawyer_ps2controller:latest -f src/SAWYER/Dockerfile .
```
#### step 2.5 (only if permission denied)
If this gives a **permission denied** error when running docker commands, add your user to
the docker group:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

Once fixed, repeat **step 2** again.

#### step 3: run the container

The gamepad is passed through as `/dev/input/js0` — plug it in **before** starting the
container. `/dev/bus/usb` is mounted so the RealSense camera enumerates inside the
container.

> RelaxedIK and `ps2_ik_teleop` are built into the image under `/root/catkin_ws`.
> Do **not** bind-mount `relaxed_ik_core` over it (it would shadow the compiled `.so`).

```bash
xhost +local:root

docker run -it \
  --name sawyer_ps2 \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/input:/dev/input \
  -v /dev/bus/usb:/dev/bus/usb \
  --device=/dev/input/js0 \
  -w /root/catkin_ws \
  sawyer_ps2controller:latest
```

#### step 4: configure and source the ROS environment

Everything lives in `/root/catkin_ws`. Edit `intera.sh` — set `robot_hostname` to the
robot's IP and `your_ip` to your computer's IP. Then source the workspace and the script:

##### terminal 1:

```bash
cd /root/catkin_ws
nano intera.sh
source devel/setup.bash
source intera.sh
```

Test ROS comms:

```bash
rostopic list
```

#### step 5: run the teleop

Open a **new host terminal** and exec into the same container:

##### terminal 2:
```bash
docker exec -it sawyer_ps2 bash
cd /root/catkin_ws
source devel/setup.bash
source intera.sh
python3 src/ps2_ik_teleop/scripts/test_ps2.py
```

On startup, press **A** to move to HOME safely, or **B** to start from the current position.
Then hold **L1** / **L2** and use the sticks to drive the arm.

If the robot E-stops, release it and re-enable:

```bash
rosrun intera_interface enable_robot.py -e
```

## Data collection (Basler + uSkin → tactile-ACT episodes)

Recording is **built into the teleop node** (`test_ps2.py`): drive the arm with the
gamepad, and capture synchronised sensor + robot state into `episode_*.hdf5` files in
the **tactile_act_real** format.

| Sensor | Package (in image) | Interface | HDF5 keys |
|--------|--------------------|-----------|-----------|
| Basler ×3 (GigE) | `pypylon` | by IP over `--net=host` | `image_left/right/top` (T,300,480,3) |
| uSkin (2 fingers) | `websocket-client` → `xela_server` | SocketCAN → `ws://localhost:5000` | `tactile_1/2` (T,5,24,3) |
| Intel RealSense | `ros-noetic-realsense2-camera` (apt) | USB → ROS topics | `image_realsense` (T,480,640,3) + `depth_realsense` (T,480,640) |

Basler IPs default to `image_left` `192.168.1.200`, `image_right` `192.168.1.210`,
`image_top` `192.168.1.220`.

Plus `action_pos`, `action_quat`, `gripper`, `joint_state`, `timestamp`.

### Rates and alignment

Two clocks, locked together:

| | rate | meaning |
|---|---|---|
| control / dataset | **20 Hz** | one HDF5 row per 50 ms step |
| uSkin tactile | **100 Hz** | **5 sub-samples per row** → the `5` in `tactile_1/2` (T,**5**,24,3) |

`~tactile_rate` must be an integer multiple of `~record_rate` — that multiple *is*
the history dimension (100/20 = 5), so the two can't be changed independently
without changing the dataset shape (the recorder raises if they don't divide).

Each row is anchored to a nominal slot `t0 + k/20`, an exact grid that never drifts.
Every source is referenced to that same slot: tactile is resampled onto the 100 Hz
grid ending at the slot by **zero-order hold** (newest sample at-or-before each
point — never interpolated forward from the future), and cameras contribute their
latest frame *plus its capture time*. So one row = one instant, and you can **verify**
that rather than trust it:

| key | meaning |
|---|---|
| `timestamp` (T,) | nominal slot time — the exact 20 Hz grid |
| `timestamp_read` (T,) | when the robot state was actually read |
| `tactile_ts` (T,5) | arrival time of each tactile sub-sample |
| `tactile_valid` (T,5) | `False` = zero-filled, no real data for that point |
| `image_*_ts`, `depth_realsense_ts` (T,) | frame capture time → `timestamp - image_left_ts` is that frame's true lag |

File attrs record `measured_rate_hz` (actual, not the requested one),
`missed_steps` (sampling overran and slots were skipped) and `stale_steps`
(some source was backfilled from cache). Backfilled frames are **counted**, not
passed off as fresh — check these before trusting an episode.

**One-time:** the XELA server is proprietary — put `xela_server` + `xServ.ini` under
`external/Xela/` before building (it gets installed to `/usr/local/bin` + `/etc/xela`).
Basler capture works without it.

**Host prep for tactile** (uSkin is on the CAN bus) — run `setup.sh` on the **host**
before starting the container. It resets `can0`, brings it up at 1 Mbps with
auto-recovery, then sniffs 3 s of `candump` to confirm the sensor is actually talking:
```bash
./setup.sh
```
It exits with an error if the INNO-MAKER adapter isn't plugged in (`lsusb` should show
`1d50:606f`). RealSense is USB — `--privileged` + `-v /dev/bus/usb:/dev/bus/usb` in the
`docker run` above is all it needs; no host prep.

**Persist episodes to the host** — add to your `docker run`:
```bash
  -v $REPO_PATH/collected_data:/root/collected_data \
```

**Start the RealSense driver** (own terminal — it's a separate ROS node):
```bash
docker exec -it sawyer_ps2 bash
source /opt/ros/noetic/setup.bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
`align_depth:=true` is **required**: it's what publishes
`/camera/aligned_depth_to_color/image_raw`, so colour pixel (u,v) and depth pixel
(u,v) correspond. The arg defaults to `false` — without it that topic never appears
and only colour gets recorded (the node warns). Check with `rostopic hz`.

**Collect** — in terminal 2 (the same container), start the tactile server first, then
run the teleop as usual:
```bash
xela_server -f /etc/xela/xServ.ini --port 5000 --ip 0.0.0.0 &   # only needed for uSkin
python3 src/ps2_ik_teleop/scripts/test_ps2.py
```
Keys while teleoperating: **r** start episode · **f** finish + save · **d** discard ·
**h** return to HOME. Episodes land in `/root/collected_data/`.

ROS params: `~camera_ips` (Basler), `~record_rate` (20 Hz), `~tactile_rate` (100 Hz),
`~save_dir`. RealSense records by default (`~record_realsense`); it subscribes to
`~realsense_color_topic` / `~realsense_depth_topic` (resolution and fps are set by
`rs_camera.launch`, not here). If the driver isn't running the node warns and records
everything else rather than aborting.

## NOTE:

if created container then dont recreate again in the future just, remember the container ID:
sudo dokcer ps
sudo docker start <container_name>
sudo docker exec -it <container_name> bash

### `[ctrl] No joystick detected!`
- Plug the gamepad in **before** starting the container.
- Confirm the host sees it: `ls /dev/input/js0` should exist.
