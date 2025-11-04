#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np
import torch

from gello.dynamixel.driver import DynamixelDriver

import pybullet as p
import pybullet_data


# ========================= Switches =========================
USE_SIM = True
USE_REAL = False
REAL_RATE = 50.0
INIT_BLEND_SEC = 2.0

# Rate limiting (per-cycle max step)
MAX_JUMP = 0.05
MAX_GRIPPER_JUMP = 0.005

# ========================= Joint Limits =========================
FRANKA_JOINT_LIMITS = {
    "q_max": [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    "q_min": [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
}
SAFE_DELTA_THRESHOLD = 0.5

# Safety threshold for initial position (after mapping)
MOTOR_INIT_THRESHOLD = 1.7  # rad


# ========================= Handle-to-robot mapping =========================
# wirte your configuration here
CONFIG = {
    "port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0",
    "joint_ids": (1, 2, 3, 4, 5, 6, 7, 8),
    # --- R3 (FR3) ---
    "joint_offsets": (
        1 * np.pi / 2,
        2 * np.pi / 2,
        2 * np.pi / 2,
        3 * np.pi / 2,
        2 * np.pi / 2,
        3 * np.pi / 2,
        1 * np.pi / 2,
    ),
    "joint_signs": (1, -1, 1, 1, 1, -1, 1),
    # Gripper config: handle motor 8 angle (deg) mapped to 0..0.04 (m)
    "gripper_id": (8,),
    "gripper_config": (157.86328125, 199.66328125),
}


# ========================= Utilities: limits/mapping =========================
def check_joint_limits(robot_arm_joint, limit_ratio=0.9):
    """Warn when reaching 90% of joint limits."""
    scaled_q_max = [q * limit_ratio for q in FRANKA_JOINT_LIMITS["q_max"]]
    scaled_q_min = [q * limit_ratio for q in FRANKA_JOINT_LIMITS["q_min"]]
    limit_exceeded = [
        (q > max_val or q < min_val)
        for q, max_val, min_val in zip(robot_arm_joint, scaled_q_max, scaled_q_min)
    ]
    return limit_exceeded


def enforce_joint_limits(robot_arm_joint, limit_exceeded_flag):
    """Clip to physical joint limits and raise if exceeded."""
    if any(limit_exceeded_flag):
        robot_arm_joint_clipped = [
            max(min(q, max_val), min_val)
            for q, max_val, min_val in zip(
                robot_arm_joint, FRANKA_JOINT_LIMITS["q_max"], FRANKA_JOINT_LIMITS["q_min"]
            )
        ]
        raise ValueError(
            f"ERROR: Joint limits exceeded! Clipped angles: {robot_arm_joint_clipped}"
        )
    return robot_arm_joint


def dynamixel_to_robot_gripper(joint_angles):
    """Map handle motor 8 angle to gripper width (meters)."""
    gripper_angle = joint_angles[0]
    gripper_angle = np.interp(
        gripper_angle,
        (np.deg2rad(CONFIG["gripper_config"][0]), np.deg2rad(CONFIG["gripper_config"][1])),
        (0, 0.04),
    )
    return [gripper_angle, gripper_angle]


def dynamixel_to_robot_arm(joint_angles):
    """
    Map handle joints -> robot 7 joints + 2 fingers.
    joint_angles: list[8], unit: rad
    return list[9]: first 7 are arm, last 2 are gripper width (duplicated)
    """
    robot_arm_radians = [
        CONFIG["joint_signs"][i] * (joint_angles[i] - CONFIG["joint_offsets"][i])
        for i in range(len(CONFIG["joint_ids"]) - 1)  # 0..6
    ]
    gripper_angle = joint_angles[7]
    gripper_radians = dynamixel_to_robot_gripper([gripper_angle])
    return robot_arm_radians + gripper_radians


# ========================= Dynamixel init/read =========================
def motor_init():
    try:
        motor_driver = DynamixelDriver(list(CONFIG["joint_ids"]), port=CONFIG["port"], baudrate=57600)
    except FileNotFoundError:
        return None

    for _ in range(10):
        motor_driver.get_joints()  # warmup

    start_motor_joints = motor_driver.get_joints()
    start_robot_joints = dynamixel_to_robot_arm(start_motor_joints)

    if any(abs(angle) > MOTOR_INIT_THRESHOLD for angle in start_robot_joints[:7]):
        raise ValueError(f"Error: Not in zero position! Joint angles: {start_robot_joints[:7]}")

    return motor_driver


def get_command(motor_driver):
    """Read handle, apply limit check, and return target of size 7+2."""
    motor_angles = motor_driver.get_joints()
    robot_cmd = dynamixel_to_robot_arm(motor_angles)
    limit_exceeded_flag = check_joint_limits(robot_cmd[:7])

    try:
        arm_cmd = enforce_joint_limits(robot_cmd[:7], limit_exceeded_flag)
    except ValueError:
        sys.exit(1)

    return arm_cmd + robot_cmd[7:]  # 7 + 2


# ========================= Real robot (Polymetis) =========================
def real_robot_init(initial_qpos=None):
    from polymetis import RobotInterface, GripperInterface

    robot = RobotInterface(ip_address="192.168.1.10")
    gripper = GripperInterface(ip_address="192.168.1.10")

    robot.go_home()
    gripper.goto(width=0, speed=0.2, force=0.1)

    if initial_qpos is not None:
        initial_qpos = torch.tensor(initial_qpos[:7], dtype=torch.float32)
        robot.move_to_joint_positions(initial_qpos)

    robot.start_joint_impedance()
    return robot, gripper


def clamp_step(curr, target, max_jump):
    dq = target - curr
    if abs(dq) > max_jump:
        dq = np.sign(dq) * max_jump
    return curr + dq


def split_arm_gripper(cmd):
    return cmd[:7], cmd[7:]  # arm 7, gripper 2 (width duplicated)


def apply_real_cmd(robot, gripper, cmd):
    """Rate-limit per joint and gripper, then send to hardware (per frame)."""
    arm, grip = split_arm_gripper(cmd)

    if not hasattr(apply_real_cmd, "_prev_arm"):
        apply_real_cmd._prev_arm = robot.get_joint_positions()
    if not hasattr(apply_real_cmd, "_prev_grip"):
        apply_real_cmd._prev_grip = 0.01

    next_arm = [clamp_step(c, t, MAX_JUMP) for c, t in zip(apply_real_cmd._prev_arm, arm)]

    target_width = float(grip[0])  # 0..0.04
    next_width = clamp_step(apply_real_cmd._prev_grip, target_width, MAX_GRIPPER_JUMP)

    robot.update_desired_joint_positions(torch.tensor(next_arm, dtype=torch.float32))
    try:
        gripper.goto(width=next_width, speed=0.1, force=20)
    except Exception:
        pass

    apply_real_cmd._prev_arm = next_arm
    apply_real_cmd._prev_grip = next_width


# ========================= Simulation (FR3 in PyBullet) =========================
def id_to_link_name(body_id, link_index):
    if link_index == -1:
        return "base"
    info = p.getJointInfo(body_id, link_index)
    return info[12].decode()


def self_collision_detected_fr3(robotId):
    """Detect FR3 self-collisions and raise if a non-allowed contact is found."""
    allow_name_pairs = {
        ("fr3_link7", "fr3_hand"),
        ("fr3_link7", "fr3_leftfinger"),
        ("fr3_link7", "fr3_rightfinger"),
        ("fr3_link7_sc", "fr3_hand"),
        ("fr3_link7", "fr3_hand_sc"),
        ("fr3_link7_sc", "fr3_hand_sc"),
        ("fr3_hand", "fr3_leftfinger"),
        ("fr3_hand", "fr3_rightfinger"),
    }

    for c in p.getContactPoints(bodyA=robotId, bodyB=robotId):
        a, b = c[3], c[4]
        if a == -1 or b == -1:
            continue

        name_a = id_to_link_name(robotId, a)
        name_b = id_to_link_name(robotId, b)

        if (name_a, name_b) in allow_name_pairs or (name_b, name_a) in allow_name_pairs:
            continue

        if c[8] > -1e-3:  # contactDistance
            continue

        if name_a.endswith("_sc") or name_b.endswith("_sc"):
            continue

        if {"fr3_hand", "fr3_leftfinger"} == {name_a, name_b}:
            continue
        if {"fr3_hand", "fr3_rightfinger"} == {name_a, name_b}:
            continue
        if {"fr3_leftfinger", "fr3_rightfinger"} == {name_a, name_b}:
            continue

        raise ValueError(
            f"FR3 self-collision detected: {name_a} (id={a}) <-> {name_b} (id={b}), "
            f"penetration={c[8]:.4f} m"
        )


def sim_robot_init_fr3(fr3_urdf_path):
    p.connect(p.GUI)

    PYB_DATA = pybullet_data.getDataPath()
    p.setAdditionalSearchPath(PYB_DATA)
    p.setGravity(0, 0, -9.8)
    p.loadURDF(os.path.join(PYB_DATA, "plane.urdf"))

    fr3_dir = os.path.dirname(fr3_urdf_path)
    p.setAdditionalSearchPath(fr3_dir)

    robotId = p.loadURDF(fr3_urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

    def name_to_index(body_id):
        name2id = {}
        for ji in range(p.getNumJoints(body_id)):
            info = p.getJointInfo(body_id, ji)
            joint_name = info[1].decode()
            link_name = info[12].decode()
            name2id[joint_name] = ji
            name2id[link_name] = ji
        return name2id

    name2id = name_to_index(robotId)
    arm_joint_names = [f"fr3_joint{i}" for i in range(1, 8)]
    arm_joint_indices = [name2id[n] for n in arm_joint_names]
    finger_joint_indices = [name2id["fr3_finger_joint1"], name2id["fr3_finger_joint2"]]

    init_q = [0, 0, 0, -1.57, 0, 1.57, 0]
    p.setJointMotorControlArray(robotId, arm_joint_indices, p.POSITION_CONTROL, targetPositions=init_q)
    for _ in range(100):
        p.stepSimulation()

    cube_path = os.path.join(PYB_DATA, "cube_small.urdf")
    cube_positions = [
        (0.5, 0, 0),
        (0.5, 0, 0.1),
        (0.5, 0, 0.2),
        (0.5, 0, 0.3),
        (0.7, 0, 0),
        (0.7, 0, 0.1),
        (0.7, 0, 0.2),
        (0.7, 0, 0.3),
        (0.7, 0, 0.4),
    ]
    for pos in cube_positions:
        p.loadURDF(cube_path, basePosition=pos)

    return robotId, arm_joint_indices, finger_joint_indices


def apply_sim_joint(joint_positions, robotId, joint_indices):
    p.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_positions,
    )
    p.stepSimulation()


# ========================= Main =========================
def blend_command(curr_cmd, target_cmd, steps):
    for k in range(1, steps + 1):
        alpha = k / steps
        yield [(1 - alpha) * c + alpha * t for c, t in zip(curr_cmd, target_cmd)]


def main():
    motor_driver = motor_init()
    if motor_driver is None:
        return

    sim_robot_ID = None
    arm_joint_indices = []
    finger_joint_indices = []
    if USE_SIM:
        FR3_URDF = "fr3-urdf-pybullet\franka_description_pybullet\robots\fr3\fr3_pybullet.urdf"
        if not os.path.isabs(FR3_URDF):
            FR3_URDF = os.path.abspath(FR3_URDF)
        sim_robot_ID, arm_joint_indices, finger_joint_indices = sim_robot_init_fr3(FR3_URDF)

        p.setJointMotorControl2(sim_robot_ID, finger_joint_indices[0], p.POSITION_CONTROL, targetPosition=0.01)
        p.setJointMotorControl2(sim_robot_ID, finger_joint_indices[1], p.POSITION_CONTROL, targetPosition=0.01)
        for _ in range(50):
            p.stepSimulation()
        self_collision_detected_fr3(sim_robot_ID)

    robot = False
    gripper = False
    if USE_REAL:
        try:
            robot, gripper = real_robot_init(initial_qpos=[0, 0, 0, -1.57, 0, 1.57, 0])
        except Exception:
            if not USE_SIM:
                return
            else:
                USE_REAL_LOCAL = False
        else:
            USE_REAL_LOCAL = True
    else:
        USE_REAL_LOCAL = False

    target_cmd = get_command(motor_driver)  # 7+2

    if USE_SIM:
        curr_arm = [p.getJointState(sim_robot_ID, j)[0] for j in arm_joint_indices]
        curr_fgr = [p.getJointState(sim_robot_ID, j)[0] for j in finger_joint_indices]
        curr = curr_arm + curr_fgr
    else:
        if USE_REAL_LOCAL and robot is not None:
            curr = list(robot.get_joint_positions()) + [0.01, 0.01]
        else:
            curr = [0, 0, 0, -1.57, 0, 1.57, 0, 0.01, 0.01]

    T = int(INIT_BLEND_SEC * REAL_RATE)
    for cmd in blend_command(curr, target_cmd, T):
        if USE_SIM:
            apply_sim_joint(cmd, sim_robot_ID, arm_joint_indices + finger_joint_indices)
        if USE_REAL_LOCAL and robot is not None and gripper is not None:
            apply_real_cmd(robot, gripper, cmd)
        time.sleep(1.0 / REAL_RATE)

    try:
        while True:
            cmd = get_command(motor_driver)  # 7+2

            if USE_SIM:
                apply_sim_joint(cmd, sim_robot_ID, arm_joint_indices + finger_joint_indices)
                self_collision_detected_fr3(sim_robot_ID)

            if USE_REAL_LOCAL and robot is not None and gripper is not None:
                apply_real_cmd(robot, gripper, cmd)

            time.sleep(1.0 / REAL_RATE)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if USE_REAL_LOCAL and robot is not None:
                robot.terminate_current_policy()
        except Exception:
            pass

        if USE_SIM:
            try:
                p.disconnect()
            except Exception:
                pass

        try:
            motor_driver.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
