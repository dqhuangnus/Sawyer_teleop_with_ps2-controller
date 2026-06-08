"""Synchronous multi-source recorder -> tactile-ACT HDF5.

A single background thread samples every source at a fixed rate (default 20 Hz,
matching the training data control rate) and buffers frames in memory.
`save()` writes them to one episode_*.hdf5 with EXACTLY the dataset layout the
tactile_act_real pipeline expects, plus optional RealSense color/depth.

Strict tactile-ACT layout (per episode, N = num recorded frames):

    image_left / image_right / image_top : (N, 300, 480, 3) uint8   Basler
    tactile_1 / tactile_2                : (N, 5, 24, 3)     float32 uSkin
    action_pos                           : (N, 3)            float64
    action_quat                          : (N, 4)            float64
    gripper                              : (N, 1)            float32
    joint_state                          : (N, 7)            float64
    timestamp                            : (N,)              float64

Optional extra keys (enabled when a RealSense camera is supplied):

    image_realsense                      : (N, 480, 640, 3)  uint8
    depth_realsense                      : (N, 480, 640)     uint16

Notes
-----
* action_pos / action_quat record the *achieved* end-effector pose at each
  step (standard for teleop demonstrations: the followed trajectory is the
  action label). gripper is 1.0 = open, 0.0 = closed (matches eval_real).
* Cameras / tactile run their own background threads; we read their latest
  cached frame at sample time, so missing frames are backfilled, never block.
"""

import os
import threading
import time
from datetime import datetime

import h5py
import numpy as np


SAWYER_JOINT_NAMES = [
    "right_j0", "right_j1", "right_j2", "right_j3",
    "right_j4", "right_j5", "right_j6",
]

# Basler keys, in the order the training dataset stores them.
BASLER_KEYS = ["image_left", "image_right", "image_top"]
BASLER_SHAPE = (300, 480, 3)          # H, W, C after binning+scale


class DataRecorder:
    def __init__(self, limb, gripper, tactile, basler, realsense=None,
                 rate_hz=20, save_dir="/root/collected_data",
                 palm_offset_m=0.0, compression="gzip", compression_level=4):
        self.limb = limb
        self.gripper = gripper
        self.tac = tactile
        self.cam = basler
        self.rs = realsense
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz
        self.save_dir = save_dir
        self.palm_offset = float(palm_offset_m)
        self.comp = compression
        self.clvl = int(compression_level)

        self._frames = []
        self._lock = threading.Lock()
        self._running = False
        self._grip_cache = 1.0   # 1.0 = open

    # ── sampling ──────────────────────────────────────────────────────────
    def _read_gripper(self):
        if self.gripper is None:
            return self._grip_cache
        try:
            self.gripper.readStatus()
            gPO = self.gripper.status.get("gPO", 0)
            self._grip_cache = 1.0 - gPO / 255.0   # 1.0 open, 0.0 closed
        except Exception:
            pass
        return self._grip_cache

    def _sample(self):
        ep = self.limb.endpoint_pose()
        pos, ori = ep["position"], ep["orientation"]
        ja = self.limb.joint_angles()
        if self.tac is not None:
            tac1, tac2 = self.tac.get_history()    # (5,24,3) each
        else:
            tac1 = np.zeros((5, 24, 3), dtype=np.float32)
            tac2 = np.zeros((5, 24, 3), dtype=np.float32)

        frame = {
            "timestamp": time.time(),
            "action_pos": np.array([pos.x, pos.y, pos.z - self.palm_offset], dtype=np.float64),
            "action_quat": np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float64),
            "joint_state": np.array([ja[n] for n in SAWYER_JOINT_NAMES], dtype=np.float64),
            "gripper": np.float32(self._read_gripper()),
            "tactile_1": tac1.astype(np.float32),
            "tactile_2": tac2.astype(np.float32),
            "images": {k: self.cam.get(k) for k in BASLER_KEYS} if self.cam else {},
        }
        if self.rs is not None:
            frame["rs_color"] = self.rs.get_color()
            frame["rs_depth"] = self.rs.get_depth()
        return frame

    def _loop(self):
        next_t = time.time()
        while self._running:
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += self.period
            try:
                fr = self._sample()
                with self._lock:
                    self._frames.append(fr)
            except Exception as e:
                print("[recorder] sample error: %s" % e)

    # ── control ───────────────────────────────────────────────────────────
    def start(self):
        with self._lock:
            self._frames = []
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False
        time.sleep(self.period * 2)   # let the loop flush its last sample

    def __len__(self):
        with self._lock:
            return len(self._frames)

    def force_sum(self):
        return self.tac.force_sum() if self.tac else 0.0

    # ── persistence ───────────────────────────────────────────────────────
    @staticmethod
    def _stack_images(frames, key, shape):
        """Stack one camera's frames, backfilling missing ones with zeros."""
        imgs = [fr["images"].get(key) for fr in frames]
        if all(im is None for im in imgs):
            return None
        ref = next(im for im in imgs if im is not None)
        zero = np.zeros_like(ref)
        return np.stack([im if im is not None else zero for im in imgs]).astype(np.uint8)

    def save(self, tag=None):
        with self._lock:
            frames = list(self._frames)
        if not frames:
            print("[recorder] nothing to save")
            return None

        os.makedirs(self.save_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = "episode_%s%s.hdf5" % (ts, ("_" + tag) if tag else "")
        path = os.path.join(self.save_dir, name)
        N = len(frames)

        with h5py.File(path, "w") as f:
            f.attrs["created"] = ts
            f.attrs["nominal_rate_hz"] = self.rate_hz
            f.attrs["num_frames"] = N

            f.create_dataset("timestamp",   data=np.array([fr["timestamp"] for fr in frames]))
            f.create_dataset("action_pos",  data=np.stack([fr["action_pos"] for fr in frames]))
            f.create_dataset("action_quat", data=np.stack([fr["action_quat"] for fr in frames]))
            f.create_dataset("joint_state", data=np.stack([fr["joint_state"] for fr in frames]))
            f.create_dataset("gripper",     data=np.array([fr["gripper"] for fr in frames],
                                                          dtype=np.float32).reshape(N, 1))
            f.create_dataset("tactile_1",   data=np.stack([fr["tactile_1"] for fr in frames]))
            f.create_dataset("tactile_2",   data=np.stack([fr["tactile_2"] for fr in frames]))

            for key in BASLER_KEYS:
                arr = self._stack_images(frames, key, BASLER_SHAPE)
                if arr is not None:
                    f.create_dataset(key, data=arr, compression=self.comp,
                                     compression_opts=self.clvl)

            if self.rs is not None:
                colors = [fr.get("rs_color") for fr in frames]
                depths = [fr.get("rs_depth") for fr in frames]
                if not all(c is None for c in colors):
                    ref = next(c for c in colors if c is not None)
                    zc = np.zeros_like(ref)
                    f.create_dataset("image_realsense",
                                     data=np.stack([c if c is not None else zc for c in colors]).astype(np.uint8),
                                     compression=self.comp, compression_opts=self.clvl)
                if not all(d is None for d in depths):
                    ref = next(d for d in depths if d is not None)
                    zd = np.zeros_like(ref)
                    f.create_dataset("depth_realsense",
                                     data=np.stack([d if d is not None else zd for d in depths]).astype(np.uint16),
                                     compression=self.comp, compression_opts=self.clvl)

        print("[recorder] saved %d frames -> %s" % (N, path))
        return path
