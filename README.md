# GELLO for Franka


## Quick Start

```bash
git clone https://github.com/wuphilipp/gello_software.git
cd gello_software
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
      
If not done already, follow the instructions of the `Create the GELLO configuration and determining joint ID's` section in the main README.md. Use the provided script to configure the GELLO for Franka FR3:

```bash
python3 scripts/gello_get_offset.py \
--start-joints 0 0 0 -1.57 0 1.57 0 \
--joint-signs 1 1 1 1 1 -1 1 \
--port /dev/serial/by-id/<GELLO_USB_ID>
```
      
To apply your configuration:
- Update the `/workspace/src/franka_gello_state_publisher/config/gello_config.yaml` file.
- Rebuild the project to ensure the updated configuration is applied:

```bash
colcon build
```

#### Step 3: Run Script
```bash
python scripts/example.py
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
