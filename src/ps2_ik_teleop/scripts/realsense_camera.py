"""Intel RealSense (color + aligned depth) via the realsense2_camera ROS driver.

The camera is NOT opened here — it is driven as a separate ROS node, started
before the teleop with:

    roslaunch realsense2_camera rs_camera.launch align_depth:=true

`align_depth:=true` is what publishes /camera/aligned_depth_to_color/image_raw,
so colour pixel (u,v) and depth pixel (u,v) correspond. Without it that topic
never appears and only colour is recorded.

This class just subscribes and caches the latest frame, exposing the same
start_bg / get_color / get_depth / stop API as BaslerCameraManager, so the
recorder treats every camera the same way.

Color: (HEIGHT, WIDTH, 3) uint8 BGR
Depth: (HEIGHT, WIDTH)    uint16 millimetres
"""

import threading

import numpy as np


COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"


class RealSenseCamera:
    def __init__(self, color_topic=COLOR_TOPIC, depth_topic=DEPTH_TOPIC,
                 wait_timeout=5.0):
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.wait_timeout = float(wait_timeout)
        self._color = None
        self._depth = None
        # capture times from the driver's message headers (not arrival time),
        # so recorded frames can be checked against the control-step timestamp
        self._color_ts = 0.0
        self._depth_ts = 0.0
        self._lock = threading.Lock()
        self._subs = []

    def start_bg(self):
        """Subscribe to the driver's topics. Raises if the driver isn't up."""
        import rospy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge

        self._bridge = CvBridge()
        # Fail loudly here rather than silently recording all-zero frames: if
        # rs_camera.launch was never started, the topic never arrives.
        try:
            rospy.wait_for_message(self.color_topic, Image, timeout=self.wait_timeout)
        except Exception:
            raise RuntimeError(
                "no message on %s after %.1fs — is the driver running?\n"
                "    roslaunch realsense2_camera rs_camera.launch align_depth:=true"
                % (self.color_topic, self.wait_timeout))

        self._subs = [
            rospy.Subscriber(self.color_topic, Image, self._on_color, queue_size=1),
            rospy.Subscriber(self.depth_topic, Image, self._on_depth, queue_size=1),
        ]

        # Depth is optional-but-expected: warn instead of dying, so a colour-only
        # session still records rather than aborting mid-setup.
        try:
            rospy.wait_for_message(self.depth_topic, Image, timeout=self.wait_timeout)
        except Exception:
            rospy.logwarn("[realsense] no %s — relaunch with align_depth:=true "
                          "or depth will not be recorded", self.depth_topic)

    def _on_color(self, msg):
        try:
            # driver publishes rgb8; ask cv_bridge for bgr8 to match Basler frames
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self._lock:
            self._color = np.asarray(img)
            self._color_ts = msg.header.stamp.to_sec()

    def _on_depth(self, msg):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return
        with self._lock:
            self._depth = np.asarray(img).astype(np.uint16)
            self._depth_ts = msg.header.stamp.to_sec()

    def get_color(self):
        with self._lock:
            return self._color.copy() if self._color is not None else None

    def get_depth(self):
        with self._lock:
            return self._depth.copy() if self._depth is not None else None

    def get_color_with_ts(self):
        with self._lock:
            return (self._color.copy() if self._color is not None else None,
                    self._color_ts)

    def get_depth_with_ts(self):
        with self._lock:
            return (self._depth.copy() if self._depth is not None else None,
                    self._depth_ts)

    def has_data(self):
        with self._lock:
            return self._color is not None

    def stop(self):
        for s in self._subs:
            try:
                s.unregister()
            except Exception:
                pass
        self._subs = []
