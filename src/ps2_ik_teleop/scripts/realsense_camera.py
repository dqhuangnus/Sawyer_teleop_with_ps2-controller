"""Intel RealSense (color + aligned depth) with a background latest-frame cache.

Mirrors the BaslerCameraManager API (start_bg / get_color / get_depth / stop)
so the recorder treats every camera the same way. Depth is aligned to the
color stream, so colour pixel (u,v) and depth pixel (u,v) correspond.

Color: (HEIGHT, WIDTH, 3) uint8 BGR
Depth: (HEIGHT, WIDTH)    uint16 millimetres
"""

import threading
import time

import numpy as np


class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        import pyrealsense2 as rs  # lazy import
        self._rs = rs
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self._pipe = None
        self._align = None
        self._color = None
        self._depth = None
        self._lock = threading.Lock()
        self._running = False

    def _open(self):
        rs = self._rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        pipe.start(cfg)
        for _ in range(5):          # warm up — first frames are unreliable
            pipe.wait_for_frames()
        self._pipe = pipe
        self._align = rs.align(rs.stream.color)
        print("[realsense] streaming %dx%d @ %dfps (color+depth aligned)"
              % (self.width, self.height, self.fps))

    def start_bg(self):
        self._open()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            try:
                fs = self._pipe.wait_for_frames(5000)
                fs = self._align.process(fs)
                c = fs.get_color_frame()
                d = fs.get_depth_frame()
                if not c or not d:
                    continue
                color = np.asanyarray(c.get_data())
                depth = np.asanyarray(d.get_data())
                with self._lock:
                    self._color = color
                    self._depth = depth
            except Exception:
                time.sleep(0.05)

    def get_color(self):
        with self._lock:
            return self._color.copy() if self._color is not None else None

    def get_depth(self):
        with self._lock:
            return self._depth.copy() if self._depth is not None else None

    def has_data(self):
        with self._lock:
            return self._color is not None

    def stop(self):
        self._running = False
        if self._pipe is not None:
            try:
                self._pipe.stop()
            except Exception:
                pass
            self._pipe = None
            self._align = None
