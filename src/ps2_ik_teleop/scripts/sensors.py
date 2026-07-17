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
    taxel).

    Every frame is stamped on arrival and kept in a time-ordered ring buffer
    (`buffer_sec` of history). Callers ask for a window by TIME
    (`history_at`), not by "last k frames" — the recorder's 20 Hz step must
    line up with the 100 Hz tactile grid regardless of jitter, dropped
    packets, or a server that briefly runs fast/slow.
    """

    def __init__(self, ws_url="ws://localhost:5000", n_per_finger=24, history_len=5,
                 buffer_sec=2.0, nominal_hz=100.0):
        import websocket  # lazy import — only needed when tactile is used
        self._websocket = websocket
        self.ws_url = ws_url
        self.n = int(n_per_finger)
        self.history_len = int(history_len)
        self.nominal_hz = float(nominal_hz)
        # ring buffer of (timestamp, array) — sized by time, not by history_len,
        # so history_at() can look back across a whole control step.
        maxlen = max(int(buffer_sec * self.nominal_hz), self.history_len * 4)
        self._buf = {1: deque(maxlen=maxlen), 2: deque(maxlen=maxlen)}
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
                        now = time.time()
                        with self._lock:
                            self._buf[sid].append((now, arr))
                            self._ts = now
            except Exception:
                if self._running:
                    time.sleep(0.5)  # connection lost — retry

    def _window_at(self, sid, grid):
        """Sample finger `sid` onto the explicit timestamp `grid`.

        Zero-order hold: each grid point takes the newest frame at or before
        it — the physically honest choice for a sensor we can only observe at
        arrival times (never interpolate forward into the future).

        Returns (data, stamps, valid):
            data   (len(grid), n, 3) float32 — zeros where no frame applies
            stamps (len(grid),)      float64 — arrival time of the frame used
                                               (0.0 where invalid)
            valid  (len(grid),)      bool    — False = zero-filled, no real data
        """
        k = len(grid)
        data = np.zeros((k, self.n, 3), dtype=np.float32)
        stamps = np.zeros(k, dtype=np.float64)
        valid = np.zeros(k, dtype=bool)
        frames = self._buf[sid]
        if not frames:
            return data, stamps, valid

        # frames are append-only in time order, so one backward scan fills the
        # whole grid — no per-point search.
        j = len(frames) - 1
        for i in range(k - 1, -1, -1):
            t = grid[i]
            while j >= 0 and frames[j][0] > t:
                j -= 1
            if j < 0:
                break                      # grid point predates all buffered data
            ts, arr = frames[j]
            data[i] = arr
            stamps[i] = ts
            valid[i] = True
        return data, stamps, valid

    def history_at(self, t_end, n=None, dt=None):
        """Window of `n` samples on a `dt`-spaced grid ending at `t_end`.

        Defaults reproduce the tactile-ACT layout: 5 samples at 100 Hz, i.e.
        the 50 ms immediately preceding t_end — exactly one 20 Hz control step.

        Returns (tac1, tac2, stamps, valid) where tac* are (n, n_taxels, 3).
        """
        n = int(n or self.history_len)
        dt = float(dt or (1.0 / self.nominal_hz))
        # oldest -> newest, last point == t_end
        grid = [t_end - (n - 1 - i) * dt for i in range(n)]
        with self._lock:
            d1, s1, v1 = self._window_at(1, grid)
            d2, s2, v2 = self._window_at(2, grid)
        # one stamp per grid point: fingers share the grid, so report the
        # newest arrival backing each point, and call a point valid only if
        # at least one finger actually had data for it.
        stamps = np.maximum(s1, s2)
        return d1, d2, stamps, (v1 | v2)

    def get_history(self):
        """Window ending 'now'. Kept for callers that don't track a slot time."""
        t1, t2, _, _ = self.history_at(time.time())
        return t1, t2

    def force_sum(self):
        """Sum of |F| across both fingers' latest frame (live UI helper)."""
        with self._lock:
            total = 0.0
            for sid in (1, 2):
                if self._buf[sid]:
                    total += float(np.linalg.norm(self._buf[sid][-1][1], axis=-1).sum())
            return total

    def rate_hz(self):
        """Measured arrival rate over the buffer (0.0 if too few frames).

        Lets the recorder verify the server really is at ~nominal_hz instead of
        assuming it.
        """
        with self._lock:
            for sid in (1, 2):
                b = self._buf[sid]
                if len(b) >= 2:
                    span = b[-1][0] - b[0][0]
                    if span > 0:
                        return (len(b) - 1) / span
            return 0.0

    def age(self):
        """Seconds since the last packet (inf if nothing ever arrived)."""
        with self._lock:
            return (time.time() - self._ts) if self._ts else float("inf")

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
        self.latest_ts = {}      # name -> time.time() when the frame was converted
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
                self.latest_ts[name] = 0.0
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
                    self.latest_ts[name] = time.time()
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

    def get_with_ts(self, name):
        """(frame, capture_time) — capture_time 0.0 if never grabbed."""
        img = self.latest.get(name)
        return (img.copy() if img is not None else None,
                float(self.latest_ts.get(name, 0.0)))

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
