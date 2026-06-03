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
git clone https://github.com/EvaEkhteyary/Sawyer_teleop_with_ps2-controller.git $REPO_PATH
cd $REPO_PATH
```

> `src/relaxed_ik_core` (Rust IK solver) and the Sawyer packages are already vendored in
> `src/`. Robotiq gripper support is optional — if no gripper is connected the node logs a
> warning and continues.

#### step 2: build the Docker image

The Dockerfile lives at `src/SAWYER/Dockerfile` and builds ROS Noetic + the Intera SDK +
Sawyer MoveIt + pygame:

```bash
docker build -t sawyer_ps2controller:latest -f src/SAWYER/Dockerfile .
```
#### step 2.5 (only if
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

```bash
xhost +local:root

docker run -it --rm \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/input:/dev/input \
  --device=/dev/input/js0 \
  -v $REPO_PATH:/root/ps2_sawyer_ws \
  -v $REPO_PATH/src/relaxed_ik_core:/root/catkin_ws/src/relaxed_ik_core \
  -w /root/ps2_sawyer_ws \
  sawyer_ps2controller:latest
```

#### step 4: configure and source the ROS environment

Edit `intera.sh` — set `robot_hostname` to the robot's IP and `your_ip` to your computer's
IP. Then source it:

##### terminal 1:

```bash
nano intera.sh
source devel/setup.bash
source intera.sh
roscore

```

Test ROS comms:

```bash
rostopic list
```

#### step 5: run the teleop

##### terminal 2:
```bash
source intera.sh
python3 src/ps2_ik_teleop/scripts/test_ps2.py
```

On startup, press **A** to move to HOME safely, or **B** to start from the current position.
Then hold **L1** / **L2** and use the sticks to drive the arm.

If the robot E-stops, release it and re-enable:

```bash
docker start -ai <container_ID>
rosrun intera_interface enable_robot.py -e
```

## NOTE:

if created container then dont recreate again in the future just, remember the container ID:
sudo dokcer ps
sudo docker start <container_name>
sudo docker exec -it <container_name> bash

### `[ctrl] No joystick detected!`
- Plug the gamepad in **before** starting the container.
- Confirm the host sees it: `ls /dev/input/js0` should exist.

