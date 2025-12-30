# SO101 IK/FK Module

This package provides **Forward Kinematics (FK)**, **Inverse Kinematics (IK)**, and **Visualization** utilities for the **SO101 Robot Arm**. It is designed to be a standalone module extracted from the LeRobot ecosystem, allowing for lightweight kinematics calculations and 3D trajectory visualization without heavy dependencies.

## Features

*   **Forward Kinematics (FK):** Compute end-effector poses from joint angles.
*   **Inverse Kinematics (IK):** Solve for joint angles given a target end-effector pose (via `placo`).
*   **Visualization:** Create 3D trajectory GIFs of the end-effector path from joint data.
*   **Data Loading:** Utilities to download and process SO101 datasets from Hugging Face.
*   **Standalone URDF:** Includes the `so101_new_calib.urdf` for self-contained operation.

## Installation

### Dependencies

Ensure you have the following installed:

```bash
pip install numpy matplotlib torch placo datasets huggingface_hub
```

*Note: `placo` is required for the underlying kinematics solver.*

### Usage

#### 1. Forward Kinematics

```python
from so101_ik_fk import SO101ForwardKinematics, SO101Position

# Initialize FK solver (uses internal URDF by default)
fk = SO101ForwardKinematics()

# Compute EE position for HOME configuration
pos = fk.get_ee_position(SO101Position.HOME)
print(f"Home Position (x, y, z): {pos}")

# Compute for custom joint angles (degrees)
# Order: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
joints = [0, 45, -45, 0, 0]
pos_custom = fk.get_ee_position(np.array(joints))
```

#### 2. Visualization Script

This package includes a script to visualize trajectories directly from a Hugging Face dataset.

```bash
# Run from the root of the repo
python -m so101_ik_fk.scripts.visualize_ee_in_3d --episode 5 --out outputs/traj.gif
```

**Arguments:**
*   `--episode`: The episode index to visualize (default: 5).
*   `--out`: Output path for the GIF (default: `outputs/episode_trajectory.gif`).

#### 3. Custom Visualization

You can visualize your own joint data (numpy array) without using the dataset loader:

```python
from so101_ik_fk import create_trajectory_gif
import numpy as np

# Shape: (N_frames, 5) or (N_frames, 6)
my_joint_data = np.random.rand(100, 5) * 45 

create_trajectory_gif(
    joint_values=my_joint_data,
    output_gif="my_trajectory.gif",
    image_key="My Custom Motion"
)
```

## Structure

```
so101_ik_fk/
├── lib/
│   ├── kinematics.py         # Generic RobotKinematics wrapper (placo)
│   └── so101_kinematics.py   # SO101 specific implementation & Enums
├── utils/
│   ├── data.py               # HF Dataset loading utilities
│   └── visualization.py      # Matplotlib 3D animation logic
├── urdfs/
│   └── so101_new_calib.urdf  # Robot description file
└── scripts/
    └── visualize_ee_in_3d.py # CLI script for generating visualizations
```

