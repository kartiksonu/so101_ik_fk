# SO101 IK/FK Module

This package provides **Forward Kinematics (FK)**, **Inverse Kinematics (IK)**, and **Visualization** utilities for the **SO101 Robot Arm**. It is designed to be a standalone module extracted from the LeRobot ecosystem, allowing for lightweight kinematics calculations and 3D visualization.

## Features

*   **Forward Kinematics (FK):** Compute end-effector poses from joint angles.
*   **Inverse Kinematics (IK):** Solve for joint angles given a target end-effector pose (via `placo`).
*   **Matplotlib Visualization:** Create 3D trajectory GIFs of the end-effector path.
*   **Rerun Visualization (NEW):** Real-time 3D visualization with robot arm, action trajectories, and camera feeds.
*   **Data Loading:** Utilities to download and process SO101 datasets from Hugging Face.
*   **Standalone URDF:** Includes the `so101_new_calib.urdf` and associated meshes for self-contained operation.

## Installation

### From Source

Clone the repository and install dependencies:

```bash
git clone https://github.com/kartiksonu/so101_ik_fk.git
cd so101_ik_fk
pip install -e .
```

### With Rerun Visualization Support

```bash
pip install -e ".[viz]"
```

Or install all optional dependencies:

```bash
pip install -e ".[all]"
```

### Dependencies

**Core:**
*   `numpy`
*   `torch`
*   `matplotlib`
*   `datasets` (Hugging Face)
*   `huggingface_hub`
*   `placo`: Required for the underlying kinematics solver.
*   `pillow`

**Optional (viz):**
*   `rerun-sdk>=0.18.0`: Real-time 3D visualization
*   `psutil`: Process management for viewer lifecycle

## Usage

#### 1. Forward Kinematics

```python
from so101_ik_fk import SO101ForwardKinematics, SO101Position
import numpy as np

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

#### 3. Custom GIF Visualization

You can visualize your own joint data (numpy array) without using the dataset loader:

```python
from so101_ik_fk import create_trajectory_gif
import numpy as np

# Shape: (N_frames, 5) or (N_frames, 6)
# Joints must be in degrees
my_joint_data = np.random.rand(100, 5) * 45

create_trajectory_gif(
    joint_values=my_joint_data,
    output_gif="my_trajectory.gif",
    image_key="My Custom Motion"
)
```

#### 4. Real-Time Rerun Visualization (NEW)

For real-time 3D visualization with robot arm, action trajectories, and camera feeds:

```python
from so101_ik_fk import RerunVisualizer
import numpy as np

# Context manager ensures proper cleanup
with RerunVisualizer(spawn=True) as viz:
    # Log a single frame
    viz.log_frame(
        joints=[0, 45, -45, 0, 0],           # Joint angles in degrees
        action_chunk=predicted_actions,       # [30, 7] action chunk (optional)
        images={"top": camera_image},         # Camera images (optional)
        target_ee=target_position,            # Target EE position (optional)
    )
```

**Features:**
- **Robot arm visualization:** Shows all joints and links based on joint angles
- **Action trajectory:** Rainbow-colored 30-step predicted trajectory
- **Camera feeds:** Display multiple camera images
- **Target visualization:** Shows raw vs clamped targets
- **Proper lifecycle:** Automatically closes Rerun viewer on exit or Ctrl+C

**Available methods:**
```python
viz.log_frame(...)           # Log complete frame (joints, actions, images, target)
viz.log_robot_arm(joints)    # Log robot arm only
viz.log_action_chunk(chunk)  # Log action trajectory only
viz.log_images(images)       # Log camera images only
viz.log_target(target)       # Log target position only
viz.close()                  # Manually close viewer
```

**Blueprint options:**
```python
RerunVisualizer(blueprint="default")       # Cameras + 3D view
RerunVisualizer(blueprint="single_camera") # One camera + 3D view
RerunVisualizer(blueprint="3d_only")       # Just 3D view
RerunVisualizer(blueprint="cameras_only")  # Just cameras
```

#### 5. Offline Dataset Playback

Combine with data loading utilities:

```python
from so101_ik_fk import RerunVisualizer, download_and_load_data, extract_joint_values

# Load dataset from HuggingFace
dataset = download_and_load_data("user/dataset")
joints = extract_joint_values(dataset, episode_idx=5)

# Playback with visualization
with RerunVisualizer(spawn=True) as viz:
    for i, joint_angles in enumerate(joints):
        viz.log_frame(joints=joint_angles)
        time.sleep(0.05)  # 20 FPS playback
```

#### 6. Interactive IK Visualizer (Plotly)

An interactive web-based visualizer to explore IK round-trip error: `||FK(IK(X_cmd)) - X_cmd||`

```bash
cd so101_ik_fk
python scripts/interactive_ik_visualizer.py
# Open http://127.0.0.1:8050 in your browser
```

**Features:**
- **Blue robot:** Baseline configuration (adjust with joint sliders)
- **Red robot:** IK solution for target end-effector position
- **Green marker:** Target end-effector position (X_cmd)
- **Real-time IK error display** in mm
- **Click-to-set:** Click anywhere on the 3D plot to set target position
- **Mini coordinate axes** at robot base (X=red, Y=green, Z=blue)

#### 7. Running Tests

```bash
cd so101_ik_fk
python -m pytest tests/ -v
```

Tests include:
- IK round-trip error verification: `||FK(IK(X_cmd)) - X_cmd|| < 1mm`
- Various robot configurations (HOME, FORWARD_EXTENDED, TUCKED)
- Random configuration tests

## Structure

```
so101_ik_fk/
├── lib/
│   ├── kinematics.py         # Generic RobotKinematics wrapper (placo)
│   ├── delta_ik.py           # Delta-based IK (prevents branch jumping)
│   └── so101_kinematics.py   # SO101 specific implementation & Enums
├── metrics/
│   └── error_metrics.py      # Kinematic error metrics (task/joint/IK error)
├── utils/
│   ├── data.py               # HF Dataset loading utilities
│   └── visualization.py      # Matplotlib 3D animation (GIF export)
├── viz/                      # Rerun visualization module
│   ├── __init__.py
│   └── rerun_visualizer.py   # RerunVisualizer class
├── tests/                    # Unit tests
│   └── test_ik_roundtrip.py  # IK round-trip error tests
├── urdfs/
│   ├── so101_new_calib.urdf  # Robot description file
│   └── assets/               # Mesh files (.stl)
├── scripts/
│   ├── visualize_ee_in_3d.py          # CLI for trajectory GIFs
│   └── interactive_ik_visualizer.py   # Plotly IK visualizer
├── pyproject.toml            # Python package configuration
└── requirements.txt          # Dependency list
```

## API Reference

### RerunVisualizer

```python
class RerunVisualizer:
    """Real-time 3D visualizer for SO101 robot using Rerun."""

    def __init__(
        self,
        name: str = "so101_viz",      # Session name
        spawn: bool = True,            # Spawn viewer
        blueprint: str = "default",    # Layout preset
    ): ...

    def log_frame(
        self,
        joints: np.ndarray = None,     # [5] degrees
        action_chunk: np.ndarray = None,  # [N, 7] or [N, 3]
        images: dict = None,           # {"name": image}
        target_ee: np.ndarray = None,  # [3] meters
        safe_target: np.ndarray = None,  # [3] meters (clamped)
    ): ...

    def close(self): ...               # Cleanup viewer
```

### SO101ForwardKinematics

```python
class SO101ForwardKinematics:
    """Forward kinematics for SO101 robot."""

    def compute(self, joint_angles: np.ndarray) -> np.ndarray:
        """Returns 4x4 transformation matrix."""

    def get_ee_position(self, joint_angles: np.ndarray) -> np.ndarray:
        """Returns [x, y, z] in meters."""

    def get_ee_orientation(self, joint_angles: np.ndarray) -> np.ndarray:
        """Returns 3x3 rotation matrix."""
```

### KinematicErrorMetrics

```python
from so101_ik_fk.metrics import KinematicErrorMetrics

metrics = KinematicErrorMetrics(fk_fn=fk.get_ee_position, ik_fn=ik_solver)

# Compute all three error metrics at once
result = metrics.compute_all(X_cmd, theta_obs)
print(result)  # task=0.0123m, joint=2.34°, ik=0.0001m

# Individual metrics:
# 1. Task Space Error: ||X_cmd - FK(θ_obs)||
# 2. Joint Space Error: ||IK(X_cmd) - θ_obs||
# 3. IK Error: ||FK(IK(X_cmd)) - X_cmd||
```
