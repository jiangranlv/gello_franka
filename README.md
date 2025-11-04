# GELLO for Franka


## Quick Start
First, confirm that your computer has a real-time kernel and polymetis installed.

```bash
git clone https://github.com/jiangranlv/gello_franka.git
cd gello_franka
```

## Installation

### Virtual Environment (Recommended)

First, install uv if you don't have it:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Create and activate a virtual environment:
```bash
uv venv --python 3.11
source .venv/bin/activate  # Run this every time you open a new shell
git submodule init
git submodule update
uv pip install -r requirements.txt
uv pip install -e .
uv pip install -e third_party/DynamixelSDK/python
```


## Getting Started

### 1. **Run the GELLO Publisher**  
#### Step 1: Determine your GELLO USB ID
      
To proceed, you need to know the USB ID of your GELLO device. This can be determined by running:

```bash
ls /dev/serial/by-id
```

Example output:

```bash
usb-FTDI_USB__-__Serial_Converter_FT7WBG6
```

In this case, the `GELLO_USB_ID` would be `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6`.

#### Step 2: Configure your GELLO 


```bash  
newgrp dialout # Temporary Effect
sudo usermod -aG dialout,plugdev <urser's name> # Takes effect permanently, but requires a reboot
```

If not done already, follow the instructions of the `Create the GELLO configuration and determining joint ID's` section in the main README.md. Use the provided script to configure the GELLO for Franka FR3:

```bash
python3 scripts/gello_get_offset.py \
--start-joints 0 0 0 -1.57 0 1.57 0 \
--joint-signs 1 1 1 1 1 -1 1 \
--port /dev/serial/by-id/<GELLO_USB_ID>
```
      
To apply your configuration:
- Update the configuration in the `/experiments/test.py` file. The location is indicated by the message on line 38.

Calibrate your gello in a simulated environment:

In the real environment, keep your gello in an initial pose as shown in the figure below.
Then
```bash
python experiments/test.py
```
Calibrate the orientation of each joint based on the simulation environment.
<p align="center">
  <img src="imgs/fr3_gello_calib_pose.jpeg" width="31%"/>
</p>

#### Step 3: Real-device testing
- Update the calibrated settings in the `/experiments/gello_master_client.py` file.
Then proceed with the Franka R3 robotic arm connection test.

terminal 1:
```bash
conda activate polymetis-local
cd <Polymetis installation path>/fairo/polymetis/polymetis/python/scripts
python launch_robot.py \
  ip=0.0.0.0 port=50051 \
  robot_client=franka_hardware \
  robot_client.executable_cfg.exec=franka_panda_client \
  robot_client.executable_cfg.robot_ip=192.168.1.10 \
  robot_client.executable_cfg.use_real_time=true
  robot_client.executable_cfg.control_port=50051
```

terminal 2:
```bash
conda activate polymetis-local
cd <Polymetis installation path>/fairo/polymetis/polymetis/python/scripts
python launch_robot.py \
  ip=0.0.0.0 port=50051 \
  robot_client=franka_hardware \
  robot_client.executable_cfg.exec=franka_panda_client \
  robot_client.executable_cfg.robot_ip=192.168.1.10 \
  robot_client.executable_cfg.use_real_time=true
  robot_client.executable_cfg.control_port=50051
```
The gripper will open and close once.

terminal 3:
```bash
conda activate polymetis-local
cd /gello_software/experiments
python r3_bridge_server.py
```
The robotic arm and gripper will move to their initial position.

terminal 4:

- Note that before connecting the master and slave arms, Gello must maintain the pose and orientation used during calibration.

```bash
conda deactivate
cd /gello/gello_software
source .venv/bin/activate

cd /gello/gello_software/experiments
python gello_master_client.py
```

## Development

### Code Organization

```
├── scripts/             # Utility scripts
├── experiments/         # Entry points and launch scripts
├── gello/               # Core GELLO package
│   ├── agents/          # Teleoperation agents
│   ├── cameras/         # Camera interfaces
│   ├── data_utils/      # Data processing utilities
│   ├── dm_control_tasks/# MuJoCo environment utilities
│   ├── dynamixel/       # Dynamixel hardware interface
|   ├── factr/           # gravity compensation
│   ├── robots/          # Robot-specific interfaces
│   ├── utils/           # Shared launch and control utilities
│   └── zmq_core/        # ZMQ multiprocessing utilities
```


## Citation

```bibtex
@misc{wu2023gello,
    title={GELLO: A General, Low-Cost, and Intuitive Teleoperation Framework for Robot Manipulators},
    author={Philipp Wu and Yide Shentu and Zhongke Yi and Xingyu Lin and Pieter Abbeel},
    year={2023},
}
```

## License & Acknowledgements

This project is licensed under the MIT License (see LICENSE file).

### Third-Party Dependencies
- [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie): Robot models for MuJoCo
- [brentyi/tyro](https://github.com/brentyi/tyro): Argument parsing and configuration
- [ZMQ](https://zeromq.org/): Multiprocessing communication framework

This project uses components from ‘FACTR Teleop: Low-Cost Force-Feedback Teleoperation’ (Apache‑2.0). See `https://github.com/RaindragonD/factr_teleop/`.
