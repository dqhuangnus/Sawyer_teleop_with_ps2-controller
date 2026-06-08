"""Sensor readers for data collection: uSkin tactile + Basler GigE cameras.

Both classes run a background thread that keeps the latest reading so the
recorder can sample them at a fixed rate without blocking.

  TactileReader  - uSkin via websocket (xela_server @ ws://localhost:5000).
                   Keeps a per-finger ring buffer so callers can pull a
                   `history_len`-frame window matching the tactile-ACT format
                   tactile_1 / tactile_2 of shape (history_len, n_taxels, 3).

  BaslerCameraManager - pypylon wrapper. In-camera binning + software scale
                   bring the 1920x1200 sensor down to 480x300 (= the
                   image_left/right/top shape in the training dataset).
"""

import json
import threading
import time
from collections import deque

import numpy as np


# ──────────────────────────────────────────────────────────────────────────
# uSkin tactile (websocket)
# ──────────────────────────────────────────────────────────────────────────
class TactileReader:
    """Background websocket client for the XELA/uSkin server.

    The server pushes JSON packets at ~100 Hz; keys "1" and "2" hold the two
    fingers, each with a "calibrated" flat list of n_taxels*3 floats (XYZ per
    taxel). We keep the last `history_len` frames per finger.
    """

    def __init__(self, ws_url="ws://localhost:5000", n_per_finger=24, history_len=5):
        import websocket  # lazy import — only needed when tactile is used
        self._websocket = websocket
        self.ws_url = ws_url
        self.n = int(n_per_finger)
        self.history_len = int(history_len)
        self._buf = {1: deque(maxlen=self.history_len),
                     2: deque(maxlen=self.history_len)}
        self._ts = 0.0
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            try:
                ws = self._websocket.create_connection(self.ws_url, timeout=5)
                while self._running:
                    msg = json.loads(ws.recv())
                    for sid in (1, 2):
                        key = str(sid)
                        if key not in msg:
                            continue
                        cal = msg[key].get("calibrated", [])
                        n = len(cal) // 3
                        if n <= 0:
                            continue
                        arr = np.asarray(cal[:n * 3], dtype=np.float32).reshape(n, 3)
                        # pad / truncate to exactly n taxels
                        if arr.shape[0] >= self.n:
                            arr = arr[: self.n]
                        else:
                            pad = np.zeros((self.n, 3), dtype=np.float32)
                            pad[: arr.shape[0]] = arr
                            arr = pad
                        with self._lock:
                            self._buf[sid].append(arr)
                            self._ts = time.time()
            except Exception:
                if self._running:
                    time.sleep(0.5)  # connection lost — retry

    def _window(self, sid):
        """Return (history_len, n, 3); pads with the oldest frame if short,
        zeros if the finger never reported."""
        frames = list(self._buf[sid])
        out = np.zeros((self.history_len, self.n, 3), dtype=np.float32)
        if not frames:
            return out
        while len(frames) < self.history_len:
            frames.insert(0, frames[0])
        for i, f in enumerate(frames[-self.history_len:]):
            out[i] = f
        return out

    def get_history(self):
        """Return (tactile_1, tactile_2), each (history_len, n_taxels, 3)."""
        with self._lock:
            return self._window(1), self._window(2)

    def force_sum(self):
        """Sum of |F| across both fingers' latest frame (live UI helper)."""
        with self._lock:
            total = 0.0
            for sid in (1, 2):
                if self._buf[sid]:
                    total += float(np.linalg.norm(self._buf[sid][-1], axis=-1).sum())
            return total

    def has_data(self):
        with self._lock:
            return bool(self._buf[1]) or bool(self._buf[2])

    def stop(self):
        self._running = False


# ──────────────────────────────────────────────────────────────────────────
# Basler GigE cameras (pypylon)
# ──────────────────────────────────────────────────────────────────────────
class BaslerCameraManager:
    """Open one InstantCamera per IP and cache the latest BGR frame.

    binning (in-camera 2x2 averaging) + scale bring 1920x1200 -> 480x300.
    Open order matters on shared links: list the better-connected cameras
    first (top camera last) in the config.
    """

    def __init__(self, camera_ips, scale=0.5, binning=2, packet_size=1500,
                 ipd_base_us=5000, fps=None):
        from pypylon import pylon  # lazy import
        import cv2
        self._pylon = pylon
        self._cv2 = cv2
        self.scale = float(scale)
        self.binning = int(binning)
        self.cameras = {}
        self.latest = {}
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        tlf = pylon.TlFactory.GetInstance()

        for idx, (name, ip) in enumerate(camera_ips.items()):
            try:
                di = pylon.DeviceInfo()
                di.SetIpAddress(ip)
                cam = pylon.InstantCamera(tlf.CreateDevice(di))
                cam.Open()

                if self.binning > 1:
                    for attr, val in (("BinningHorizontalMode", "Average"),
                                      ("BinningVerticalMode", "Average")):
                        try:
                            getattr(cam, attr).Value = val
                        except Exception:
                            pass
                    try:
                        cam.BinningHorizontal.Value = self.binning
                        cam.BinningVertical.Value = self.binning
                    except Exception as e:
                        print("[camera] %s: binning=%d rejected (%s)" % (name, self.binning, e))

                cam.GevSCPSPacketSize.Value = packet_size
                try:
                    cam.GevSCPD.Value = ipd_base_us * (idx + 1)
                except Exception:
                    pass
                try:
                    cam.GevHeartbeatTimeout.Value = 10000
                except Exception:
                    pass
                if fps is not None:
                    try:
                        cam.AcquisitionFrameRateEnable.Value = True
                        cam.AcquisitionFrameRate.Value = float(fps)
                    except Exception as e:
                        print("[camera] %s: fps=%s rejected (%s)" % (name, fps, e))

                time.sleep(0.5)
                cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                self.cameras[name] = cam
                self.latest[name] = None
                print("[camera] %s (%s) open: binning=%d packet=%d"
                      % (name, ip, self.binning, packet_size))
            except Exception as e:
                print("[camera] %s (%s) failed: %s" % (name, ip, e))

        self._running = False
        self._grab_all()

    def _grab_all(self):
        cv2 = self._cv2
        for name, cam in self.cameras.items():
            try:
                grab = cam.RetrieveResult(500, self._pylon.TimeoutHandling_Return)
                if grab and grab.IsValid() and grab.GrabSucceeded():
                    img = self.converter.Convert(grab).GetArray()
                    if self.scale != 1.0:
                        h, w = img.shape[:2]
                        img = cv2.resize(img, (int(w * self.scale), int(h * self.scale)),
                                         interpolation=cv2.INTER_AREA)
                    self.latest[name] = img
                if grab:
                    grab.Release()
            except Exception:
                pass

    def start_bg(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            self._grab_all()

    def get(self, name):
        img = self.latest.get(name)
        return img.copy() if img is not None else None

    @property
    def names(self):
        return list(self.cameras.keys())

    def stop(self):
        self._running = False
        for cam in self.cameras.values():
            try:
                cam.StopGrabbing()
                cam.Close()
            except Exception:
                pass
