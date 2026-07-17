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

Alignment / provenance keys (always written):

    timestamp_read                       : (N,)      float64  actual read time
    tactile_ts                           : (N, 5)    float64  per-sub-sample time
    tactile_valid                        : (N, 5)    bool     False = zero-filled
    image_*_ts / depth_realsense_ts      : (N,)      float64  frame capture time

Rates & alignment
-----------------
Two clocks, locked together:

    control / dataset rate   20 Hz   -> one row per 50 ms step
    tactile rate            100 Hz   -> 5 sub-samples per row

tactile_hz must be an integer multiple of rate_hz; that multiple IS the
history_len dimension (100/20 = 5), which is why they cannot be tuned
independently without changing the dataset shape.

Each row is anchored to a nominal slot time t_slot = t0 + k/rate_hz (an exact
grid, so no drift accumulates). Every source is then referenced to that same
t_slot: tactile is resampled onto the 100 Hz grid ending at t_slot by
zero-order hold (newest sample at-or-before each point — never interpolated
forward), and cameras contribute their latest frame together with its capture
time. So a row describes one instant, and *_ts lets you verify that rather
than trust it: `t_slot - image_left_ts` is that frame's true lag.

Notes
-----
* action_pos / action_quat record the *achieved* end-effector pose at each
  step (standard for teleop demonstrations: the followed trajectory is the
  action label). gripper is 1.0 = open, 0.0 = closed (matches eval_real).
* Cameras / tactile run their own background threads; we read their latest
  cached frame at sample time, so missing frames are backfilled, never block.
  Backfilled frames are counted (attrs stale_steps) instead of passing
  silently as if they were fresh.
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
                 rate_hz=20, tactile_hz=100, save_dir="/root/collected_data",
                 palm_offset_m=0.0, compression="gzip", compression_level=4,
                 stale_warn_sec=0.15):
        self.limb = limb
        self.gripper = gripper
        self.tac = tactile
        self.cam = basler
        self.rs = realsense
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz
        self.tactile_hz = float(tactile_hz)
        self.tactile_dt = 1.0 / self.tactile_hz
        self.save_dir = save_dir
        self.palm_offset = float(palm_offset_m)
        self.comp = compression
        self.clvl = int(compression_level)
        self.stale_warn = float(stale_warn_sec)

        # tactile_hz / rate_hz sub-samples per control step — this IS the
        # history_len of the tactile-ACT layout (100/20 = 5), so the two rates
        # cannot be set independently without changing the dataset shape.
        ratio = self.tactile_hz / self.rate_hz
        self.hist_len = int(round(ratio))
        if abs(ratio - self.hist_len) > 1e-6:
            raise ValueError(
                "tactile_hz (%g) must be an integer multiple of rate_hz (%g); "
                "got ratio %.3f" % (self.tactile_hz, self.rate_hz, ratio))
        if self.tac is not None and self.hist_len != self.tac.history_len:
            print("[recorder] note: tactile history_len %d -> %d (from %g/%g Hz)"
                  % (self.tac.history_len, self.hist_len, self.tactile_hz, self.rate_hz))

        self._frames = []
        self._lock = threading.Lock()
        self._running = False
        self._grip_cache = 1.0   # 1.0 = open
        self._missed = 0         # control steps skipped because sampling overran
        self._stale = 0          # steps where some source had no fresh data

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

    def _sample(self, t_slot):
        """Capture one control step anchored at `t_slot`.

        Every source is referenced to the SAME t_slot rather than to "now at
        the moment I happened to read it", so all keys in a row describe the
        same instant. Robot state is read first — it is the only source read
        synchronously, so it defines the step; cameras/tactile are then pulled
        as-of that time from their buffers.
        """
        t_read = time.time()
        ep = self.limb.endpoint_pose()
        pos, ori = ep["position"], ep["orientation"]
        ja = self.limb.joint_angles()

        n_tax = self.tac.n if self.tac is not None else 24
        if self.tac is not None:
            # the hist_len tactile samples spanning (t_slot - step, t_slot]
            tac1, tac2, tac_ts, tac_valid = self.tac.history_at(
                t_slot, n=self.hist_len, dt=self.tactile_dt)
        else:
            tac1 = np.zeros((self.hist_len, n_tax, 3), dtype=np.float32)
            tac2 = np.zeros((self.hist_len, n_tax, 3), dtype=np.float32)
            tac_ts = np.zeros(self.hist_len, dtype=np.float64)
            tac_valid = np.zeros(self.hist_len, dtype=bool)

        images, image_ts = {}, {}
        if self.cam:
            for k in BASLER_KEYS:
                img, ts = self.cam.get_with_ts(k)
                images[k], image_ts[k] = img, ts

        frame = {
            "timestamp": t_slot,      # nominal slot — the regular 20 Hz grid
            "timestamp_read": t_read,  # when the robot state was actually read
            "action_pos": np.array([pos.x, pos.y, pos.z - self.palm_offset], dtype=np.float64),
            "action_quat": np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float64),
            "joint_state": np.array([ja[n] for n in SAWYER_JOINT_NAMES], dtype=np.float64),
            "gripper": np.float32(self._read_gripper()),
            "tactile_1": tac1.astype(np.float32),
            "tactile_2": tac2.astype(np.float32),
            "tactile_ts": tac_ts,
            "tactile_valid": tac_valid,
            "images": images,
            "image_ts": image_ts,
        }
        if self.rs is not None:
            frame["rs_color"], frame["rs_color_ts"] = self.rs.get_color_with_ts()
            frame["rs_depth"], frame["rs_depth_ts"] = self.rs.get_depth_with_ts()

        # A frame every source backfilled from a stale cache is not an aligned
        # sample — count it so save() can report how clean the episode is.
        lags = [t_slot - ts for ts in list(image_ts.values()) if ts > 0]
        if self.rs is not None and frame.get("rs_color_ts", 0) > 0:
            lags.append(t_slot - frame["rs_color_ts"])
        if (lags and max(lags) > self.stale_warn) or (self.tac is not None and not tac_valid.any()):
            self._stale += 1
        return frame

    def _loop(self):
        # Absolute schedule: slot times are derived from t0, so they never drift
        # with accumulated per-iteration cost.
        t0 = time.time()
        k = 0
        while self._running:
            t_slot = t0 + k * self.period
            now = time.time()
            if now < t_slot:
                time.sleep(t_slot - now)
            elif now > t_slot + self.period:
                # Sampling overran its budget. Skip to the next real slot instead
                # of burning CPU catching up and emitting a burst of frames that
                # all share (almost) the same true time.
                skipped = int((now - t_slot) / self.period)
                self._missed += skipped
                k += skipped
                t_slot = t0 + k * self.period
            k += 1
            try:
                fr = self._sample(t_slot)
                with self._lock:
                    self._frames.append(fr)
            except Exception as e:
                print("[recorder] sample error: %s" % e)

    # ── control ───────────────────────────────────────────────────────────
    def start(self):
        with self._lock:
            self._frames = []
        self._missed = 0
        self._stale = 0
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

        # measured rate from the first/last slot — the honest number to report,
        # rather than echoing the rate we asked for
        span = frames[-1]["timestamp"] - frames[0]["timestamp"]
        meas_hz = (N - 1) / span if (N > 1 and span > 0) else 0.0

        with h5py.File(path, "w") as f:
            f.attrs["created"] = ts
            f.attrs["nominal_rate_hz"] = self.rate_hz
            f.attrs["measured_rate_hz"] = meas_hz
            f.attrs["tactile_rate_hz"] = self.tactile_hz
            f.attrs["tactile_history_len"] = self.hist_len
            f.attrs["num_frames"] = N
            f.attrs["missed_steps"] = self._missed
            f.attrs["stale_steps"] = self._stale

            f.create_dataset("timestamp",   data=np.array([fr["timestamp"] for fr in frames]))
            f.create_dataset("timestamp_read",
                             data=np.array([fr["timestamp_read"] for fr in frames]))
            # per-sub-sample tactile times + validity: makes the 100 Hz -> 20 Hz
            # alignment auditable after the fact instead of a claim in a README
            f.create_dataset("tactile_ts",
                             data=np.stack([fr["tactile_ts"] for fr in frames]))
            f.create_dataset("tactile_valid",
                             data=np.stack([fr["tactile_valid"] for fr in frames]))
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
                    f.create_dataset(key + "_ts",
                                     data=np.array([fr["image_ts"].get(key, 0.0)
                                                    for fr in frames]))

            if self.rs is not None:
                colors = [fr.get("rs_color") for fr in frames]
                depths = [fr.get("rs_depth") for fr in frames]
                if not all(c is None for c in colors):
                    ref = next(c for c in colors if c is not None)
                    zc = np.zeros_like(ref)
                    f.create_dataset("image_realsense",
                                     data=np.stack([c if c is not None else zc for c in colors]).astype(np.uint8),
                                     compression=self.comp, compression_opts=self.clvl)
                    f.create_dataset("image_realsense_ts",
                                     data=np.array([fr.get("rs_color_ts", 0.0) for fr in frames]))
                if not all(d is None for d in depths):
                    ref = next(d for d in depths if d is not None)
                    zd = np.zeros_like(ref)
                    f.create_dataset("depth_realsense",
                                     data=np.stack([d if d is not None else zd for d in depths]).astype(np.uint16),
                                     compression=self.comp, compression_opts=self.clvl)
                    f.create_dataset("depth_realsense_ts",
                                     data=np.array([fr.get("rs_depth_ts", 0.0) for fr in frames]))

        print("[recorder] saved %d frames -> %s  (%.2f Hz measured, target %.0f)"
              % (N, path, meas_hz, self.rate_hz))
        if self._missed:
            print("[recorder] WARNING: %d control step(s) skipped — sampling "
                  "overran %.0f Hz" % (self._missed, self.rate_hz))
        if self._stale:
            print("[recorder] WARNING: %d/%d step(s) had stale/missing sensor "
                  "data (see *_ts / tactile_valid)" % (self._stale, N))
        return path
