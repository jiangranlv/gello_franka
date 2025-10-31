#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, zmq, numpy as np
from gello.dynamixel.driver import DynamixelDriver
import pybullet as p, pybullet_data
import json

# ---------- Config ----------
USE_SIM = False
CLIENT_RATE_HZ = float(os.environ.get("CLIENT_RATE_HZ", "50"))
PERIOD = 1.0 / CLIENT_RATE_HZ
PRINT_TX_EVERY_S = float(os.environ.get("PRINT_TX_EVERY_S", "1.0"))
TEST_GRIPPER_TOGGLE = os.environ.get("TEST_GRIPPER_TOGGLE", "0") == "1"

VIRTUAL_INIT_Q = [0, 0, 0, -1.57, 0, 1.57, 0]
VIRTUAL_INIT_GRIP = 0.02

FRANKA_JOINT_LIMITS = {
    "q_max": [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    "q_min": [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
}

CONFIG = {
    "port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0",
    "joint_ids": (1,2,3,4,5,6,7,8),
    "joint_offsets": (1*np.pi/2, 2*np.pi/2, 2*np.pi/2, 3*np.pi/2, 2*np.pi/2, 3*np.pi/2, 1*np.pi/2),
    "joint_signs": (1, -1, 1, 1, 1, -1, 1),
    "gripper_config": (155.66328125, 199.86328125),
}

FR3_URDF = "/home/galbot/Lin/Projects/Teleoperation/gello/gello_software/fr3-urdf-pybullet-main/franka_description_pybullet/robots/fr3/fr3_pybullet.urdf"

# ---------- Safety & Mapping Helpers ----------
def check_joint_limits(q):
    for i, qi in enumerate(q):
        if qi > FRANKA_JOINT_LIMITS["q_max"][i] or qi < FRANKA_JOINT_LIMITS["q_min"][i]:
            return False
    return True

def dynamixel_to_robot_gripper(joint_angles):
    g = joint_angles[0]
    g = np.interp(
        g,
        (np.deg2rad(CONFIG["gripper_config"][0]), np.deg2rad(CONFIG["gripper_config"][1])),
        (0, 0.08),
    )
    return [g, g]

def dynamixel_to_robot_arm(joint_angles):
    arm = [
        CONFIG["joint_signs"][i] * (joint_angles[i] - CONFIG["joint_offsets"][i])
        for i in range(len(CONFIG["joint_ids"])-1)
    ]
    grip = dynamixel_to_robot_gripper([joint_angles[7]])
    return arm + grip

# ---------- Hardware Driver ----------
def motor_init():
    drv = DynamixelDriver(list(CONFIG["joint_ids"]), port=CONFIG["port"], baudrate=57600)
    for _ in range(10): drv.get_joints()
    return drv

# ---------- Simulation (PyBullet FR3) ----------
def sim_robot_init_fr3(fr3_urdf_path):
    p.connect(p.GUI)
    PYB_DATA = pybullet_data.getDataPath()
    p.setAdditionalSearchPath(PYB_DATA)
    p.setGravity(0, 0, -9.8)
    p.loadURDF(os.path.join(PYB_DATA, "plane.urdf"))
    p.setAdditionalSearchPath(os.path.dirname(fr3_urdf_path))
    robotId = p.loadURDF(fr3_urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
    name2id = {}
    for ji in range(p.getNumJoints(robotId)):
        info = p.getJointInfo(robotId, ji)
        name2id[info[1].decode()] = ji; name2id[info[12].decode()] = ji
    arm_joint_names = [f"fr3_joint{i}" for i in range(1,8)]
    arm_joint_indices = [name2id[n] for n in arm_joint_names]
    finger_joint_indices = [name2id["fr3_finger_joint1"], name2id["fr3_finger_joint2"]]
    p.setJointMotorControlArray(robotId, arm_joint_indices, p.POSITION_CONTROL, targetPositions=VIRTUAL_INIT_Q)
    for fj in finger_joint_indices:
        p.setJointMotorControl2(robotId, fj, p.POSITION_CONTROL, targetPosition=VIRTUAL_INIT_GRIP)
    for _ in range(100): p.stepSimulation()
    return robotId, arm_joint_indices, finger_joint_indices

def apply_sim_joint(joint_positions, robotId, joint_indices):
    p.setJointMotorControlArray(robotId, joint_indices, p.POSITION_CONTROL, targetPositions=joint_positions)
    p.stepSimulation()

# ---------- Client Main ----------
def main():
    # --- Simulation bringup (optional) ---
    sim_pack = None
    if USE_SIM:
        robotId, arm_idx, finger_idx = sim_robot_init_fr3(FR3_URDF)
        sim_pack = (robotId, arm_idx, finger_idx)

    # --- Hardware bringup ---
    drv = motor_init()

    # --- ZMQ / Networking ---
    ctx = zmq.Context.instance()
    req = ctx.socket(zmq.REQ);  req.connect("tcp://127.0.0.1:6000")
    push = ctx.socket(zmq.PUSH); push.connect("tcp://127.0.0.1:6001")

    hello = {"type":"init","virtual_init_arm":VIRTUAL_INIT_Q,"virtual_init_grip":VIRTUAL_INIT_GRIP}
    req.send_json(hello); rep = req.recv_json()
    if rep.get("status") != "READY":
        print("[client] server not ready:", rep); return
    print("[client] server READY. Start streaming.")

    # --- Main loop (read -> map -> sim/tx) ---
    sent = 0
    last_log_t = time.time()

    try:
        while True:
            ang = drv.get_joints()
            cmd = dynamixel_to_robot_arm(ang)
            arm = cmd[:7]; width = float(cmd[7])  # m

            if TEST_GRIPPER_TOGGLE:
                width = 0.01 if (int(time.time()) % 4 < 2) else 0.06

            if not check_joint_limits(arm):
                time.sleep(PERIOD); continue

            if sim_pack:
                robotId, arm_idx, finger_idx = sim_pack
                apply_sim_joint(arm + [width,width], robotId, arm_idx + finger_idx)

            payload = {"type":"cmd","arm":arm,"gripper":width}            
            push.send_json(payload)
            sent += 1

            t = time.time()
            if t - last_log_t >= PRINT_TX_EVERY_S:
                print(f"[client] stream tx ~ {sent / (t - last_log_t):.1f} Hz | w={width:.3f} | q0={arm[0]:.3f}")
                sent = 0; last_log_t = t

            time.sleep(PERIOD)
    except KeyboardInterrupt:
        print("[client] bye")

if __name__ == "__main__":
    main()
