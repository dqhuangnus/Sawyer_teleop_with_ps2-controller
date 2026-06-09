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
| Intel RealSense *(off by default)* | `pyrealsense2` (not installed yet) | USB | `image_realsense` + `depth_realsense` |

Plus `action_pos`, `action_quat`, `gripper`, `joint_state`, `timestamp`.

**One-time:** the XELA server is proprietary — put `xela_server` + `xServ.ini` under
`external/Xela/` before building (it gets installed to `/usr/local/bin` + `/etc/xela`).
Basler capture works without it.

**Host prep for tactile** (uSkin is on the CAN bus):
```bash
sudo ip link set can0 up type can bitrate 1000000
```

**Persist episodes to the host** — add to your `docker run`:
```bash
  -v $REPO_PATH/collected_data:/root/collected_data \
```

**Collect** — in terminal 2 (the same container), start the tactile server first, then
run the teleop as usual:
```bash
xela_server -f /etc/xela/xServ.ini --port 5000 --ip 0.0.0.0 &   # only needed for uSkin
python3 src/ps2_ik_teleop/scripts/test_ps2.py
```
Keys while teleoperating: **r** start episode · **f** finish + save · **d** discard ·
**h** return to HOME. Episodes land in `/root/collected_data/`. Camera IPs / rate are
ROS params (`~camera_ips`, `~record_rate`); `~record_realsense` enables RealSense
(after `pip install pyrealsense2`).

## NOTE:

if created container then dont recreate again in the future just, remember the container ID:
sudo dokcer ps
sudo docker start <container_name>
sudo docker exec -it <container_name> bash

### `[ctrl] No joystick detected!`
- Plug the gamepad in **before** starting the container.
- Confirm the host sees it: `ls /dev/input/js0` should exist.
