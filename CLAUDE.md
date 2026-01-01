# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SO101 IK/FK Module - A Python package providing Forward Kinematics (FK), Inverse Kinematics (IK), and 3D trajectory visualization for the SO101 Robot Arm. Standalone module extracted from the LeRobot ecosystem.

**Key capabilities:** Compute end-effector poses from joint angles (FK), solve for joint angles given target poses (IK via `placo`), create 3D trajectory GIFs, and load SO101 datasets from Hugging Face.

## Common Commands

```bash
# Installation (requires virtual environment - never use system Python)
uv venv && source .venv/bin/activate
pip install -e .

# Run visualization CLI
python -m so101_ik_fk.scripts.visualize_ee_in_3d --episode 5 --out outputs/traj.gif
```

No test suite or linting configuration exists yet.

## Architecture

**Layered design:**

```
lib/                    # Core kinematics (Tier 1)
├── kinematics.py       # Generic RobotKinematics wrapper around placo library
└── so101_kinematics.py # SO101-specific: SO101ForwardKinematics class, SO101Position enum

utils/                  # Utility modules (Tier 2)
├── visualization.py    # create_trajectory_gif() - matplotlib 3D animation
└── data.py            # HuggingFace dataset loading (download_and_load_data, extract_joint_values)

scripts/               # CLI entry points (Tier 3)
└── visualize_ee_in_3d.py

urdfs/                 # Robot description
├── so101_new_calib.urdf
└── assets/            # 13 STL mesh files
```

**Public API** (exported from `__init__.py`):
- `RobotKinematics` - Generic kinematics with configurable URDF/joints
- `SO101ForwardKinematics` - Pre-configured for SO101 robot
- `SO101Position` - Enum of predefined poses (HOME, FORWARD_EXTENDED, VERTICAL_UP, TUCKED)
- `create_trajectory_gif()`, `download_and_load_data()`, `extract_joint_values()`

## Robot Configuration

- **5 controllable joints:** shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
- **End-effector frame:** "gripper_frame_link"
- **Kinematics solver:** `placo` library (C++ wrapper)

## Key Dependencies

- `placo` (>=0.0.6) - FK/IK solver
- `numpy`, `torch` - numerical operations
- `matplotlib` - visualization
- `datasets`, `huggingface_hub` - data loading
