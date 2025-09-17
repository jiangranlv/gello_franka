import numpy as np
import time
import sys
# from dynamixel.driver import DynamixelDriver
from gello.dynamixel.driver import DynamixelDriver


import pybullet as p
import pybullet_data
import torch

FRANKA_JOINT_LIMITS = {
    "q_max": [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    "q_min": [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    # "q_max": [2.8973, 1.7628, 2.8973, 3.0718, 2.8973, 3.7525, 2.8973],
    # "q_min": [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -3.7525, -2.8973]
}
SAFE_DELTA_THRESHOLD = 0.5  # Safe threshold for joint difference in rad
def check_joint_limits(robot_arm_joint, limit_ratio=0.9):
    scaled_q_max = [q * limit_ratio for q in FRANKA_JOINT_LIMITS["q_max"]]
    scaled_q_min = [q * limit_ratio for q in FRANKA_JOINT_LIMITS["q_min"]]

    limit_exceeded = [(q > max_val or q < min_val) for q, max_val, min_val in
                      zip(robot_arm_joint, scaled_q_max, scaled_q_min)]
    return limit_exceeded
def enforce_joint_limits(robot_arm_joint, limit_exceeded_flag):
    if any(limit_exceeded_flag):
        print("\033[91mWARNING: Joint limits exceeded!\033[0m")
        robot_arm_joint_clipped = [max(min(q, max_val), min_val) for q, max_val, min_val in
                                   zip(robot_arm_joint, FRANKA_JOINT_LIMITS["q_max"], FRANKA_JOINT_LIMITS["q_min"])]
        print(f"clipped joint angles: {robot_arm_joint_clipped}")
        sys.stderr.write(f"\033[91mERROR: Joint limits exceeded! Clipped angles: {robot_arm_joint_clipped}\033[0m\n")
        raise ValueError(f"ERROR: Joint limits exceeded! Clipped angles: {robot_arm_joint_clipped}")  # turn it on if you want the safe error
        # return robot_arm_joint_clipped
    return robot_arm_joint

CONFIG = {    
    "port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0",    
    "joint_ids": (1, 2, 3, 4, 5, 6, 7, 8),

    # R3
    # "joint_offsets": (
    #     1*np.pi/2, 2*np.pi/2, 6*np.pi/2, 3*np.pi/2, 2*np.pi/2, 3*np.pi/2, -1*np.pi/2
    # ),    
    # "joint_signs": (1, 1, 1, 1, 1, -1, 1),

    # Panda
    "joint_offsets": (
        1*np.pi/2, 2*np.pi/2, 6*np.pi/2, 3*np.pi/2, 2*np.pi/2, 3*np.pi/2, 1*np.pi/2
    ),    
    "joint_signs": (1, -1, 1, 1, 1, -1, 1),
    
    # 57 close, 106 open, map to (0,0.02)
    "gripper_id": (8,),
    "gripper_config": (157.775390625, 199.575390625),
}
def dynamixel_to_robot_arm(joint_angles):
    # joint_angles = driver.get_joints() # rad
    robot_arm_radians = [
        CONFIG["joint_signs"][i] * (joint_angles[i] - CONFIG["joint_offsets"][i])
        for i in range(len(CONFIG["joint_ids"])-1)
    ]
    gripper_angle = joint_angles[7]
    gripper_radians = dynamixel_to_robot_gripper([gripper_angle])
    return robot_arm_radians + gripper_radians
def dynamixel_to_robot_gripper(joint_angles):
    gripper_angle = joint_angles[0]
    gripper_angle = np.interp(gripper_angle, (np.deg2rad(CONFIG["gripper_config"][0]), np.deg2rad(CONFIG["gripper_config"][1])), (0, 0.04))
    # return two gripper fingers angle
    return [gripper_angle, gripper_angle]

MOTOR_INIT_THRESHOLD = 1.7
def motor_init():
    try:
        motor_driver = DynamixelDriver(list(CONFIG["joint_ids"]), port=CONFIG["port"], baudrate=57600)
    except FileNotFoundError:
        print("Dynamixel driver not found")
        return
    for _ in range(10):
        motor_driver.get_joints()  # warmup

    start_motor_joints = motor_driver.get_joints()
    robot_joints = dynamixel_to_robot_arm(start_motor_joints)
    for i, (motor, robot) in enumerate(zip(start_motor_joints, robot_joints[:7])):
        print(f"Motor {i}: {motor:.4f} rad, robot: {robot:.4f} rad")
    start_robot_joints = dynamixel_to_robot_arm(start_motor_joints)
    # Check if any joint is above 0.523 rad (30 degrees)
    


    if any(abs(angle) > MOTOR_INIT_THRESHOLD for angle in start_robot_joints):
        raise ValueError(f"Error: Not in zero position! Joint angles: {start_robot_joints}")
    
    
    # 检查是否超出关节限制，而不是固定零位
    # limit_exceeded_flag = check_joint_limits(start_robot_joints[:7])
    # try:
    #     enforce_joint_limits(start_robot_joints[:7], limit_exceeded_flag)
    # except ValueError as e:
    #     print("WARNING: Start position exceeds limits, adjust offsets or start-joints")
    #     raise e

    return motor_driver

def apply_real_joint(joint_positions, robot):
    # todo: jiangran modify this
    # state_log = robot.move_to_joint_positions(joint_positions, time_to_go=2)
    robot.update_desired_joint_positions(joint_positions)
    # robot.update_desired_joint_positions(joint_positions)

def real_robot_init(initial_qpos = None):
    from polymetis import RobotInterface, GripperInterface
    # Initialize robot interface
    robot = RobotInterface(
        ip_address="localhost",
    )
    gripper = GripperInterface(ip_address="localhost")

    # Reset
    robot.go_home()
    gripper.goto(width=0, speed=0.2, force=0.1)
    initial_qpos = torch.Tensor(initial_qpos)[:7]
    robot.move_to_joint_positions(initial_qpos)
    robot.start_joint_impedance()
    return robot

def apply_sim_joint(joint_positions, robotId, joint_indices):
    p.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_positions
    )
    p.stepSimulation()

def sim_robot_init():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf")
    # robotId = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    robotId = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
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
    cube_ids = []
    for pos in cube_positions:
        cube_id = p.loadURDF("cube_small.urdf", basePosition=pos)
        cube_ids.append(cube_id)
    joint_indices = [0, 1, 2, 3, 4, 5, 6, 9, 10]
    # init_position = [0, 0, 0, -1.571, 0, 1.867, 0, 0.02, 0.02]
    # init_position = [0, 0, 0, -1.571, 0, 1.867, 0, 0, 0]
    init_position = [0, 0, 0, -1.57, 0, 1.57, 0, 0, 0]
    
    apply_sim_joint(init_position, robotId, joint_indices)
    for _ in range(100): # warmup
        p.stepSimulation()
        time.sleep(1 / 240.0)
    return robotId

def self_collision_detected(robotId):
    contact_points = p.getContactPoints(bodyA=robotId, bodyB=robotId)
    hand_link_id = 8  # panda_hand
    exclude_links = {6, 9, 10, 11}  # panda_leftfinger, panda_rightfinger, panda_grasptarget
    for contact in contact_points:
        link_a, link_b = contact[3], contact[4]
        if hand_link_id in (link_a, link_b) and (link_a not in exclude_links and link_b not in exclude_links):
            print(f"\033[91mSelf-collision detected! panda_hand collides with another link.\033[0m")
            print(f"LinkA: {link_a}, LinkB: {link_b}, Contact Position: {contact[6]}")
            raise ValueError("ERROR: Self-collision detected between panda_hand and another body part!")


def get_command(motor_driver):
    # q_position = [0, 0, 0, -1.571, 0, 1.867, 0, 0, 0]

    motor_angles = motor_driver.get_joints()
    robot_arm_joint = dynamixel_to_robot_arm(motor_angles)
    limit_exceeded_flag = check_joint_limits(robot_arm_joint[:7])
    try: # check joint limits
        robot_arm_joint_clipped = enforce_joint_limits(robot_arm_joint[:7], limit_exceeded_flag)
    except ValueError as e:
        print(e)
        exit(1)

    return robot_arm_joint_clipped + robot_arm_joint[7:]

def main():
    motor_driver = motor_init()
    if motor_driver is None:
        print("Motor driver not available. Exiting.")
        return
    sim_robot_ID = sim_robot_init()

    initial_command = get_command(motor_driver)
    joint_positions = [p.getJointState(sim_robot_ID, i)[0] for i in range(7)]
    joint_indices = [0, 1, 2, 3, 4, 5, 6, 9, 10]
    apply_sim_joint(initial_command, sim_robot_ID, joint_indices)
    self_collision_detected(sim_robot_ID) # check self-collision

    real_robot = None
    # real_robot = real_robot_init(initial_command) # comment this if you dont use real robot !!!!!!

    if real_robot is not None:
        print("Performing joint impedance control...")
        real_robot.start_joint_impedance()

    time_list = []

    try:
        while True:
            # a = time.perf_counter()
            robot_arm_joint_clipped = get_command(motor_driver)

            joint_positions = [p.getJointState(sim_robot_ID, i)[0] for i in range(7)]
            apply_sim_joint(robot_arm_joint_clipped, sim_robot_ID, joint_indices)
            self_collision_detected(sim_robot_ID) # check self-collision

            # Get joint positions and apply to real robot
            if real_robot is not None:
                joint_positions = real_robot.get_joint_positions()
                # print(f"Current real robot positions: {joint_positions}")
                joint_positions_desired = torch.Tensor(robot_arm_joint_clipped)#[:7]

                a = time.perf_counter()
                apply_real_joint(joint_positions_desired, real_robot)
                b = time.perf_counter()

                time_list.append(b-a)
                if len(time_list)==5000000:
                    break

            # if the robot_arm_joint_clipped is too far away from the current joint_positions_desired, raise safe error
            # todo: use SAFE_DELTA_THRESHOLD
            # delt_position = ...
            # if delt_position ...

            if real_robot is not None:
                real_robot_joint_positions = real_robot.get_joint_positions()

            time.sleep(0.02)

        print("=============")
        print("Over!!!!!!!!!!!!!!!!!!!!!")

    except KeyboardInterrupt:
        if real_robot is not None:
            real_robot.terminate_current_policy()
        p.disconnect()
        if motor_driver is not None:
            motor_driver.close()


if __name__ == "__main__":
    main()