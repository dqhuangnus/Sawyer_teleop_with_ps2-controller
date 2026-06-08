FROM ros:noetic
 
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
 
# -----------------------------
# 1. Install system + ROS deps
# -----------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    python3-rosdep \
    python3-wstool \
    python3-vcstools \
    # python3-argparse \
    python3-pip \
    ros-noetic-catkin \
    ros-noetic-xacro \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-cv-bridge \
    ros-noetic-control-msgs \
    # ros-noetic-joystick-drivers \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-trajectory-msgs \
    ros-noetic-rospy-message-converter \
    ros-noetic-moveit \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-msgs \
    ros-noetic-joint-state-publisher \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# focal's stock pip (20.0.2) doesn't understand the manylinux_2_28 tag on the
# pypylon wheel — upgrade pip ONLY (NOT setuptools: a newer setuptools breaks
# catkin's interrogate_setup_dot_py.py via the old importlib_metadata).
RUN python3 -m pip install --upgrade pip

# Python deps. numpy bumped to 1.24.4 (old 1.17.4 is incompatible with h5py).
#   pygame           -> gamepad input
#   pypylon          -> Basler GigE cameras (Basler cp38 wheel; rc not on PyPI)
#   websocket-client -> uSkin tactile (xela_server @ ws://localhost:5000)
#   h5py             -> episode recording   (cv2 from apt python3-opencv)
# RealSense (pyrealsense2) intentionally not installed — capture off by default.
RUN python3 -m pip install \
        "numpy==1.24.4" \
        "pyRobotiqGripper==1.0.1" \
        "pygame" \
        "https://github.com/basler/pypylon/releases/download/2.0.0rc1/pypylon-2.0.0rc1-cp38-cp38-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl" \
        "websocket-client==1.8.0" \
        "h5py==3.11.0"

# -----------------------------
# 1.5 Rust toolchain (to build RelaxedIK)
# -----------------------------
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 
# -----------------------------
# 2. Create catkin workspace
# -----------------------------
WORKDIR /root/catkin_ws
RUN mkdir -p src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
 
# -----------------------------
# 3. Sawyer + Intera SDK
# -----------------------------
RUN git clone https://github.com/RethinkRobotics/sawyer_robot.git /root/catkin_ws/src/sawyer_robot
 
# intera_common requires wstool rosinstall file
RUN printf '%s\n' \
    "- git:" \
    "    local-name: intera_common" \
    "    uri: https://github.com/RethinkRobotics/intera_common.git" \
    "    version: master" > /root/catkin_ws/src/sawyer.rosinstall
 
RUN cd /root/catkin_ws && \
    wstool init src /root/catkin_ws/src/sawyer.rosinstall && \
    wstool update -t src
 
RUN git clone https://github.com/RethinkRobotics/intera_sdk.git /root/catkin_ws/src/intera_sdk
 
# -----------------------------
# 4. Add Sawyer MoveIt
# -----------------------------
RUN cd /root/catkin_ws && \
    wstool merge -t src https://raw.githubusercontent.com/RethinkRobotics/sawyer_moveit/master/sawyer_moveit.rosinstall && \
    wstool update -t src
 
# -----------------------------
# 5. Add Robotiq
# -----------------------------
# RUN git clone https://github.com/ros-industrial/robotiq.git /root/catkin_ws/src/robotiq
 
# -----------------------------
# 6. Setup rosdep
# -----------------------------
RUN rosdep init || true
RUN rosdep update
RUN cd /root/catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y
 
# -----------------------------
# 6.5 RelaxedIK solver + teleop package
# -----------------------------
COPY src/relaxed_ik_core /root/catkin_ws/src/relaxed_ik_core
RUN cd /root/catkin_ws/src/relaxed_ik_core && cargo build --release

COPY src/ps2_ik_teleop /root/catkin_ws/src/ps2_ik_teleop

# -----------------------------
# 6.6 XELA / uSkin tactile server (proprietary, supplied by user)
# -----------------------------
# uSkin websocket server (ws://localhost:5000) is vendor software, not in git.
# Place the binary + config under ./external/Xela/ before building:
#     external/Xela/xela_server -> /usr/local/bin/xela_server  (~57 MB)
#     external/Xela/xServ.ini    -> /etc/xela/xServ.ini
# Start at runtime:  xela_server -f /etc/xela/xServ.ini --port 5000 --ip 0.0.0.0
# Build still succeeds without it — only tactile capture is unavailable.
COPY external/Xela/ /tmp/xela/
RUN if [ -f /tmp/xela/xela_server ]; then \
        install -m 0755 /tmp/xela/xela_server /usr/local/bin/xela_server && \
        mkdir -p /etc/xela && \
        ([ -f /tmp/xela/xServ.ini ] && cp /tmp/xela/xServ.ini /etc/xela/xServ.ini || true) && \
        echo "[build] XELA server installed"; \
    else \
        echo "[build] external/Xela/xela_server absent — tactile server not installed"; \
    fi && rm -rf /tmp/xela

# -----------------------------
# 7. Build catkin workspace
# -----------------------------
RUN bash -c "\
    source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -j$(nproc)"
 
 
# -----------------------------
# 8. Utilities
# -----------------------------
RUN apt-get update && apt-get install -y \
    net-tools nano vim iputils-ping \
    && rm -rf /var/lib/apt/lists/*
 
COPY intera.sh /root/catkin_ws/intera.sh
 
CMD ["bash"]
