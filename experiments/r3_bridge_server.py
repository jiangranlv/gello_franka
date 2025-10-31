#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, threading, re
from typing import List, Optional
import numpy as np
import zmq, torch, cv2, h5py
from polymetis import RobotInterface, GripperInterface

# ---------- Config & Limits ----------
# --- Joint limits & velocities ---
FRANKA_JOINT_LIMITS = {
    "q_max": [ 2.8,  1.66,  2.8 , -0.17,  2.8 ,  3.65,  2.8 ],
    "q_min": [-2.8, -1.66, -2.8 , -2.97, -2.8 ,  0.08, -2.8 ],
}
FRANKA_JOINT_VEL = [2.075, 2.075, 2.075, 2.075, 2.51, 2.51, 2.51]

# --- Control timing & smoothing ---
RATE_HZ   = float(os.environ.get("ARM_RATE_HZ", "100"))
PERIOD    = 1.0 / RATE_HZ
VEL_SAFETY_RATIO = 0.7
EMA_ALPHA = float(os.environ.get("ARM_EMA", "0.99"))
MAX_STEP_JOINT = [v * PERIOD * VEL_SAFETY_RATIO for v in FRANKA_JOINT_VEL]

# --- Gripper ranges & dynamics ---
GRIPPER_MIN = float(os.environ.get("GRIPPER_MIN", "0.0"))
GRIPPER_MAX = float(os.environ.get("GRIPPER_MAX", "0.08"))
GRIPPER_SPEED = float(os.environ.get("GRIPPER_SPEED", "1.5"))
GRIPPER_FORCE = float(os.environ.get("GRIPPER_FORCE", "30"))
GRIPPER_MIN_STEP = float(os.environ.get("GRIPPER_MIN_STEP", "0.000001"))
GRIPPER_KEEPALIVE = float(os.environ.get("GRIPPER_KEEPALIVE", "0.25"))
GRIPPER_CMD_HZ = float(os.environ.get("GRIPPER_CMD_HZ", "60"))
GRIPPER_CMD_PERIOD = 1.0 / GRIPPER_CMD_HZ

# --- Gripper mapping behavior ---
W_GAIN = float(os.environ.get("W_GAIN", "1.0"))
SNAP_EPS = float(os.environ.get("SNAP_EPS", "0.001"))

# --- Stream watchdog ---
NO_STREAM_WARN_S = float(os.environ.get("NO_STREAM_WARN_S", "0.5"))

# --- Recording & camera ---
SAVE_FREQ = float(os.environ.get("SAVE_FREQ", "20"))
SAVE_DIR  = os.environ.get("SAVE_DIR", "dataset/cup")
CAM_DEV   = os.environ.get("CAM_DEV", "/dev/video0")
CAM_FOURCC = os.environ.get("CAM_FOURCC", "MJPG")
CAM_SIZE  = (int(os.environ.get("CAM_W", "2560")), int(os.environ.get("CAM_H", "720")))
CAM_FPS   = int(os.environ.get("CAM_FPS", "20"))
CROP_RIGHT_HALF = True
DOWNSAMPLE = 4

# --- Episode length (frames) ---
SAVE_FRAM = 100400

# --- Start pose & alignment ---
START_JOINTS_DEFAULT = [0, 0, 0, -1.57, 0, 1.57, 0]
START_HOLD_S = float(os.environ.get("START_HOLD_S", "0.5"))
ALIGN_TOL = float(os.environ.get("ALIGN_TOL", "0.03"))

# ---------- Utils ----------
def safe_list(x):
    if x is None: return None
    if hasattr(x, "tolist"): return x.tolist()
    return list(x)

def clip_q(q: List[float]) -> List[float]:
    return [max(min(v, qmax), qmin) for v, qmax, qmin in zip(q, FRANKA_JOINT_LIMITS["q_max"], FRANKA_JOINT_LIMITS["q_min"])]

def step_toward(curr: List[float], tgt: List[float], max_steps: List[float]) -> List[float]:
    return [c + np.clip(t - c, -lim, lim) for c, t, lim in zip(curr, tgt, max_steps)]

def is_close(q1, q2, tol=ALIGN_TOL) -> bool:
    return all(abs(a-b) <= tol for a, b in zip(q1, q2))

# ---------- Camera ----------
class CameraReader:
    """V4L2 camera thread with latest-frame buffer."""
    def __init__(self, dev=CAM_DEV, api=cv2.CAP_V4L2, fourcc=CAM_FOURCC, size=CAM_SIZE, fps=CAM_FPS):
        self.dev, self.api, self.fourcc, self.size, self.fps = dev, api, fourcc, size, fps
        self.cap = None
        self._lock = threading.Lock()
        self._stop = False
        self.latest_frame = None
        self._t = None

    def start(self):
        cap = cv2.VideoCapture(self.dev, self.api)
        if not cap.isOpened(): raise RuntimeError(f"cannot open {self.dev}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        self._stop = False
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while not self._stop and self.cap is not None:
            ok, frame = self.cap.read()
            if not ok: time.sleep(0.005); continue
            with self._lock: self.latest_frame = frame

    def get_latest(self):
        with self._lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def stop(self):
        self._stop = True
        if self._t is not None: self._t.join(timeout=1.0)
        if self.cap is not None: self.cap.release()
        cv2.destroyAllWindows()

# ---------- Gripper ----------
class GripperWorker(threading.Thread):
    """Periodic gripper command thread (rate-limited + keepalive)."""
    def __init__(self, gripper: GripperInterface, getter):
        super().__init__(daemon=True)
        self.g = gripper
        self.get_w_in = getter
        self._stop = threading.Event()
        self._last_w = None
        self._last_t = 0.0

    def stop(self): self._stop.set()

    @staticmethod
    def _map_and_snap(w_in: float) -> float:
        if w_in is None or (isinstance(w_in, float) and np.isnan(w_in)):
            w_in = 0.0
        w_cmd = W_GAIN * float(w_in)
        w_cmd = float(min(max(w_cmd, GRIPPER_MIN), GRIPPER_MAX))
        if abs(w_cmd - GRIPPER_MIN) < SNAP_EPS: w_cmd = GRIPPER_MIN
        if abs(w_cmd - GRIPPER_MAX) < SNAP_EPS: w_cmd = GRIPPER_MAX
        return w_cmd

    def run(self):
        while not self._stop.is_set():
            now = time.time()
            if now - self._last_t < GRIPPER_CMD_PERIOD:
                time.sleep(GRIPPER_CMD_PERIOD - (now - self._last_t)); continue
            w_in = self.get_w_in()
            w_cmd = self._map_and_snap(w_in)
            need = (self._last_w is None) or (abs(w_cmd - self._last_w) > GRIPPER_MIN_STEP) or ((now - self._last_t) > GRIPPER_KEEPALIVE)
            if need:
                self.g.goto(w_cmd, GRIPPER_SPEED, GRIPPER_FORCE, False)
                self._last_w, self._last_t = w_cmd, time.time()
            else:
                time.sleep(0.001)

# ---------- Recorder ----------
class EpisodeRecorder:
    """Buffer state/action/image at SAVE_FREQ; dump HDF5 when full()."""
    def __init__(self, save_dir: str, save_freq: float, gripper: GripperInterface):
        self.dir = save_dir
        os.makedirs(self.dir, exist_ok=True)
        self.dt = 1.0 / save_freq
        self.g = gripper
        self.reset_buffers()

    def reset_buffers(self):
        self.qpos_list: List[np.ndarray] = []
        self.action_list: List[np.ndarray] = []
        self.image_list: List[np.ndarray] = []
        self._init_t = -100.0
        self._count = 0
        self._started = False

    def maybe_push(self, robot: RobotInterface, target_q: List[float], w_cmd: float, frame: Optional[np.ndarray]):
        now = time.time()
        if (now - self._init_t) > (self.dt * self._count):
            if not self._started:
                self._init_t = now; self._started = True
            self._count += 1

            q_raw = safe_list(robot.get_joint_positions())
            qpos = np.zeros(8, dtype=np.float32)
            qpos[:7] = np.array(q_raw if q_raw is not None else target_q, dtype=np.float32)

            try:
                w_state = float(self.g.get_state().width)
            except Exception:
                w_state = w_cmd
            qpos[7] = float(min(max(w_state, GRIPPER_MIN), GRIPPER_MAX))
            self.qpos_list.append(qpos)

            act = np.zeros(8, dtype=np.float32)
            act[:7] = np.array(target_q, dtype=np.float32)
            act[7]  = float(w_cmd)
            self.action_list.append(act)

            if frame is not None:
                H, W, C = frame.shape
                if CROP_RIGHT_HALF: frame = frame[:, W//2:, :]
                H, W, C = frame.shape
                frame = cv2.resize(frame, (W//DOWNSAMPLE, H//DOWNSAMPLE), interpolation=cv2.INTER_AREA)
                self.image_list.append(frame.copy())

    def full(self) -> bool:
        return len(self.image_list) == SAVE_FRAM and len(self.image_list) > 0

    def save_and_next(self) -> str:
        pat = re.compile(r"^episode_(\d+)\.hdf5$")
        used = {int(m.group(1)) for f in os.listdir(self.dir) for m in [pat.match(f)] if m}
        idx = 0
        while idx in used: idx += 1
        path = os.path.join(self.dir, f"episode_{idx}.hdf5")
        with h5py.File(path, 'w') as f:
            f.create_dataset('image_diagonal_view', data=self.image_list, compression='gzip', compression_opts=9)
            f.create_dataset('action', data=self.action_list, compression='gzip', compression_opts=9)
            f.create_dataset('qpos',   data=self.qpos_list, compression='gzip', compression_opts=9)
        os.chmod(path, 0o777)
        self.reset_buffers()
        return path

# ---------- Control ----------
class Control:
    """Robot bringup, ZMQ I/O, smoothing, rate-limited control, and recording."""
    def __init__(self):
        ip = os.environ.get("POLYMETIS_SERVER_IP", "127.0.0.1")
        self.robot = RobotInterface(ip_address=ip, port=int(os.environ.get("POLYMETIS_SERVER_PORT", "50051")))
        self.grip  = GripperInterface(ip_address=ip, port=int(os.environ.get("POLYMETIS_GRIPPER_PORT", "50052")))
        if hasattr(self.grip, "home"): self.grip.home()
        self.grip.goto((GRIPPER_MIN + GRIPPER_MAX)/2.0, 0.1, 15, False)

        self.cam = CameraReader(); self.cam.start()
        self.rep, self.pull, self.pub = self._bind_zmq()

        self.target_q = START_JOINTS_DEFAULT[:]
        self.ema_q    = START_JOINTS_DEFAULT[:]
        self.curr_q   = START_JOINTS_DEFAULT[:]

        self._w_lock = threading.Lock()
        self._w_in = 0.0

        self.rec = EpisodeRecorder(SAVE_DIR, SAVE_FREQ, self.grip)

        self._align_to_pose(START_JOINTS_DEFAULT)
        self._handshake_and_optional_align()

        self.robot.start_joint_impedance()
        self.gw = GripperWorker(self.grip, self.get_w_in); self.gw.start()

        self.poller = zmq.Poller(); self.poller.register(self.pull, zmq.POLLIN)
        self.last_cmd_t, self.last_pub_t = time.time(), 0.0

    def _bind_zmq(self):
        ctx = zmq.Context.instance()
        rep  = ctx.socket(zmq.REP);  rep.bind("tcp://*:6000")
        pull = ctx.socket(zmq.PULL); pull.bind("tcp://*:6001")
        pub  = ctx.socket(zmq.PUB);  pub.bind("tcp://*:6002")
        return rep, pull, pub

    def _align_to_pose(self, pose: List[float]):
        curr = safe_list(self.robot.get_joint_positions()) or pose[:]
        if not is_close(curr, pose):
            self.robot.move_to_joint_positions(torch.tensor(pose, dtype=torch.float32))
        time.sleep(START_HOLD_S)

    def _handshake_and_optional_align(self):
        print("[comm] waiting for client init...")
        msg = self.rep.recv_json()
        if not isinstance(msg, dict) or msg.get("type") != "init":
            self.rep.send_json({"status":"ERR","reason":"expect init"}); raise RuntimeError("expect init")
        from_msg = msg.get("virtual_init_arm") if "virtual_init_arm" in msg else msg.get("virtual_init")
        if isinstance(from_msg, (list, tuple)) and len(from_msg) == 7:
            pose = clip_q([float(v) for v in from_msg])
            self._align_to_pose(pose)
            self.target_q = pose[:]; self.ema_q = pose[:]; self.curr_q = pose[:]
        self.rep.send_json({"status":"READY"})
        print("[comm] READY, streaming...")

    def set_w_in(self, x: float):
        with self._w_lock:
            self._w_in = float(x)

    def get_w_in(self) -> float:
        with self._w_lock:
            return self._w_in

    def loop_once(self) -> bool:
        t0 = time.time()

        latest = None
        while True:
            socks = dict(self.poller.poll(timeout=0))
            if self.pull not in socks: break
            payload = self.pull.recv_json(flags=zmq.NOBLOCK)
            latest = payload
        if latest and isinstance(latest, dict) and latest.get("type") == "cmd":
            arm = latest.get("arm") or latest.get("q")
            w   = latest.get("gripper") or latest.get("w")
            if isinstance(arm, (list, tuple)) and len(arm) == 7:
                self.target_q = clip_q([float(v) for v in arm])
            if w is not None:
                try:
                    self.set_w_in(float(w))
                except Exception:
                    pass
            self.last_cmd_t = t0

        if (t0 - self.last_cmd_t) > NO_STREAM_WARN_S:
            print(f"[comm] WARN: no stream for {(t0 - self.last_cmd_t):.2f}s (check client & ports)")
            self.last_cmd_t = t0

        self.ema_q  = [(1-EMA_ALPHA)*e + EMA_ALPHA*t for e, t in zip(self.ema_q, self.target_q)]
        next_q      = step_toward(self.curr_q, self.ema_q, MAX_STEP_JOINT)
        self.robot.update_desired_joint_positions(torch.tensor(next_q, dtype=torch.float32))
        self.curr_q = next_q

        frame = self.cam.get_latest()
        w_cmd_for_action = GripperWorker._map_and_snap(self.get_w_in())
        self.rec.maybe_push(self.robot, self.target_q, w_cmd_for_action, frame)

        if self.rec.full():
            path = self.rec.save_and_next()
            self.grip.goto(GRIPPER_MAX, GRIPPER_SPEED, GRIPPER_FORCE, False)
            time.sleep(0.2)
            self._align_to_pose(START_JOINTS_DEFAULT)
            print(f"[comm] episode saved: {path}")
            return False

        if (t0 - self.last_pub_t) > 0.2:
            self.last_pub_t = t0
            state = {
                "type": "state",
                "ts": t0,
                "q":  safe_list(self.robot.get_joint_positions()),
                "dq": safe_list(self.robot.get_joint_velocities()),
                "w_cmd": float(w_cmd_for_action),
            }
            self.pub.send_json(state)

        dt = time.time() - t0
        if dt < PERIOD: time.sleep(PERIOD - dt)
        return True

    def run(self):
        try:
            while self.loop_once():
                pass
        finally:
            try: self.cam.stop()
            except: pass
            try: self.gw.stop()
            except: pass

class App:
    """Thin entrypoint wrapper."""
    def run(self):
        ctrl = Control()
        ctrl.run()

if __name__ == "__main__":
    App().run()
