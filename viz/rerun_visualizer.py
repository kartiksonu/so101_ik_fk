"""
Rerun-based 3D visualizer for SO101 robot.

Provides real-time visualization of:
- Robot arm (joint space) from joint angles
- Action trajectories (action space) from model predictions
- Camera feeds
- Coordinate frames
- Target positions with clamping visualization
"""

from __future__ import annotations

import atexit
import colorsys
import os
import signal
import subprocess
import sys
from typing import Any

import numpy as np

try:
    import rerun as rr
    import rerun.blueprint as rrb
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

from ..lib.so101_kinematics import SO101ForwardKinematics, SO101_JOINT_NAMES

# Frame names for robot visualization (base to EE)
SO101_FRAME_NAMES = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "gripper_frame_link",
]


class RerunVisualizer:
    """
    Reusable Rerun visualizer for SO101 robot.

    Handles:
    - 3D robot arm visualization from joint angles
    - Action chunk trajectory visualization
    - Camera image display
    - Proper lifecycle management (signal handlers, cleanup)

    Usage:
        # Context manager (recommended)
        with RerunVisualizer(spawn=True) as viz:
            viz.log_frame(joints=[0, 45, -45, 0, 0], images={"top": img})

        # Manual management
        viz = RerunVisualizer(spawn=True)
        try:
            viz.log_frame(...)
        finally:
            viz.close()
    """

    def __init__(
        self,
        name: str = "so101_viz",
        spawn: bool = True,
        blueprint: str = "default",
    ):
        """
        Initialize the visualizer.

        Args:
            name: Rerun session name
            spawn: Whether to spawn a new Rerun viewer
            blueprint: Layout preset ("default", "cameras_only", "3d_only", or custom)
        """
        if not RERUN_AVAILABLE:
            raise ImportError(
                "rerun-sdk is required for visualization. "
                "Install with: pip install rerun-sdk>=0.18.0"
            )

        self.name = name
        self.viewer_pid = None
        self._closed = False

        # Initialize kinematics for FK
        self.fk = SO101ForwardKinematics()
        self.kinematics = self.fk.kinematics

        # Initialize Rerun
        rr.init(name, spawn=spawn)

        # Try to get viewer PID for cleanup
        if spawn:
            self._find_viewer_pid()

        # Setup blueprint
        self._setup_blueprint(blueprint)

        # Register cleanup handlers
        self._register_cleanup_handlers()

        # Log static elements
        self._log_coordinate_frame()

    def _find_viewer_pid(self):
        """Try to find the spawned Rerun viewer PID."""
        try:
            import psutil
            for proc in psutil.process_iter(['pid', 'name']):
                if 'rerun' in proc.info['name'].lower():
                    self.viewer_pid = proc.info['pid']
                    break
        except ImportError:
            pass  # psutil not available
        except Exception:
            pass  # Process enumeration failed

    def _setup_blueprint(self, blueprint: str):
        """Configure Rerun layout."""
        if blueprint == "default":
            bp = rrb.Horizontal(
                rrb.Vertical(
                    rrb.Spatial2DView(name="Top Camera", origin="/cameras/top"),
                    rrb.Spatial2DView(name="Wrist Camera", origin="/cameras/wrist"),
                ),
                rrb.Spatial3DView(name="3D View", origin="/world"),
                column_shares=[1, 2],
            )
        elif blueprint == "cameras_only":
            bp = rrb.Horizontal(
                rrb.Spatial2DView(name="Top Camera", origin="/cameras/top"),
                rrb.Spatial2DView(name="Wrist Camera", origin="/cameras/wrist"),
            )
        elif blueprint == "3d_only":
            bp = rrb.Spatial3DView(name="3D View", origin="/world")
        elif blueprint == "single_camera":
            bp = rrb.Horizontal(
                rrb.Spatial2DView(name="Camera", origin="/cameras/top"),
                rrb.Spatial3DView(name="3D View", origin="/world"),
                column_shares=[1, 2],
            )
        else:
            # Assume custom blueprint object passed
            bp = blueprint

        rr.send_blueprint(bp)

    def _register_cleanup_handlers(self):
        """Register signal handlers and atexit for cleanup."""
        atexit.register(self.close)

        # Store original handlers
        self._original_sigint = signal.getsignal(signal.SIGINT)
        self._original_sigterm = signal.getsignal(signal.SIGTERM)

        # Register our handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle termination signals."""
        self.close()
        # Call original handler or exit
        if signum == signal.SIGINT and self._original_sigint:
            if callable(self._original_sigint):
                self._original_sigint(signum, frame)
        sys.exit(0)

    # =========================================================================
    # Static Elements
    # =========================================================================

    def _log_coordinate_frame(self, axis_length: float = 0.15):
        """Log XYZ coordinate axes at origin."""
        # X axis (red)
        rr.log("/world/axes/x", rr.Arrows3D(
            origins=[[0, 0, 0]],
            vectors=[[axis_length, 0, 0]],
            colors=[[255, 0, 0]],
            radii=[0.003],
        ))
        rr.log("/world/axes/x_label", rr.Points3D(
            [[axis_length + 0.02, 0, 0]],
            labels=["X"],
            radii=[0.001],
            colors=[[255, 0, 0]],
        ))

        # Y axis (green)
        rr.log("/world/axes/y", rr.Arrows3D(
            origins=[[0, 0, 0]],
            vectors=[[0, axis_length, 0]],
            colors=[[0, 255, 0]],
            radii=[0.003],
        ))
        rr.log("/world/axes/y_label", rr.Points3D(
            [[0, axis_length + 0.02, 0]],
            labels=["Y"],
            radii=[0.001],
            colors=[[0, 255, 0]],
        ))

        # Z axis (blue)
        rr.log("/world/axes/z", rr.Arrows3D(
            origins=[[0, 0, 0]],
            vectors=[[0, 0, axis_length]],
            colors=[[0, 0, 255]],
            radii=[0.003],
        ))
        rr.log("/world/axes/z_label", rr.Points3D(
            [[0, 0, axis_length + 0.02]],
            labels=["Z"],
            radii=[0.001],
            colors=[[0, 0, 255]],
        ))

    # =========================================================================
    # Robot Arm Visualization (Joint Space)
    # =========================================================================

    def get_frame_positions(self, joint_positions_deg: np.ndarray) -> list[np.ndarray]:
        """
        Get 3D positions of all robot frames for visualization.

        Args:
            joint_positions_deg: Joint positions in degrees [5 values]

        Returns:
            List of 3D positions for each frame
        """
        # Update joint positions in placo robot
        joint_rad = np.deg2rad(joint_positions_deg[:5])
        for i, joint_name in enumerate(SO101_JOINT_NAMES):
            self.kinematics.robot.set_joint(joint_name, joint_rad[i])
        self.kinematics.robot.update_kinematics()

        # Get position of each frame
        positions = []
        for frame_name in SO101_FRAME_NAMES:
            try:
                T = self.kinematics.robot.get_T_world_frame(frame_name)
                pos = T[:3, 3]
                positions.append(pos)
            except Exception:
                # If frame doesn't exist, use last known position
                if positions:
                    positions.append(positions[-1].copy())
                else:
                    positions.append(np.zeros(3))

        return positions

    def log_robot_arm(
        self,
        joint_positions_deg: np.ndarray,
        show_labels: bool = True,
        namespace: str = "/world/robot",
    ):
        """
        Log robot arm visualization based on joint angles.

        Args:
            joint_positions_deg: Joint positions in degrees [5 or 6 values]
            show_labels: Whether to show frame labels
            namespace: Rerun path namespace
        """
        # Get all frame positions
        positions = np.array(self.get_frame_positions(joint_positions_deg))

        # Joint positions as spheres (gradient)
        joint_colors = []
        for i in range(len(positions)):
            intensity = 150 + int(105 * i / max(1, len(positions) - 1))
            joint_colors.append([intensity, intensity, intensity])

        rr.log(f"{namespace}/joints", rr.Points3D(
            positions,
            colors=joint_colors,
            radii=[0.012] * len(positions),
        ))

        # Links as lines (cyan)
        rr.log(f"{namespace}/links", rr.LineStrips3D(
            [positions],
            colors=[[0, 200, 255]],
            radii=[0.006],
        ))

        # Labels for each frame
        if show_labels:
            for i, (frame_name, pos) in enumerate(zip(SO101_FRAME_NAMES, positions)):
                label_pos = pos + np.array([0.02, 0, 0.02])
                rr.log(f"{namespace}/labels/{frame_name}", rr.Points3D(
                    [label_pos],
                    labels=[frame_name],
                    radii=[0.001],
                    colors=[[200, 200, 200]],
                ))

        # Highlight end-effector (green)
        ee_pos = positions[-1]
        rr.log(f"{namespace}/ee", rr.Points3D(
            [ee_pos],
            colors=[[0, 255, 0]],
            radii=[0.018],
        ))

    # =========================================================================
    # Action Trajectory Visualization (Action Space)
    # =========================================================================

    def log_action_chunk(
        self,
        action_chunk: np.ndarray,
        current_ee: np.ndarray | None = None,
        show_labels: bool = True,
        namespace: str = "/world/action",
    ):
        """
        Log action chunk trajectory visualization.

        Args:
            action_chunk: Action chunk [N, 7] or [N, 3] (xyz positions)
            current_ee: Current EE position for connection line
            show_labels: Whether to show position labels
            namespace: Rerun path namespace
        """
        # Extract XYZ positions
        if action_chunk.ndim == 1:
            action_chunk = action_chunk[np.newaxis, :]

        if action_chunk.shape[1] >= 3:
            positions = action_chunk[:, :3]
        else:
            positions = action_chunk

        n_points = len(positions)

        # Generate rainbow colors
        colors = []
        for i in range(n_points):
            t = i / max(1, n_points - 1)
            h = t * 0.8  # Red to purple
            r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
            colors.append([int(r * 255), int(g * 255), int(b * 255)])

        # Action points
        rr.log(f"{namespace}/points", rr.Points3D(
            positions,
            colors=colors,
            radii=[0.008] * n_points,
        ))

        # Trajectory line (orange)
        rr.log(f"{namespace}/trajectory", rr.LineStrips3D(
            [positions],
            colors=[[255, 165, 0]],
            radii=[0.003],
        ))

        # Connection from current EE to first action
        if current_ee is not None:
            rr.log(f"{namespace}/connection", rr.LineStrips3D(
                [[current_ee, positions[0]]],
                colors=[[255, 255, 0]],
                radii=[0.002],
            ))

        # Labels
        if show_labels and n_points > 0:
            rr.log(f"{namespace}/label_start", rr.Points3D(
                [positions[0] + np.array([0.01, 0.01, 0.01])],
                labels=["Action[0]"],
                radii=[0.001],
                colors=[[255, 0, 0]],
            ))

            if n_points > 2:
                mid_idx = n_points // 2
                rr.log(f"{namespace}/label_mid", rr.Points3D(
                    [positions[mid_idx] + np.array([0.01, 0.01, 0.01])],
                    labels=[f"Action[{mid_idx}]"],
                    radii=[0.001],
                    colors=[[0, 255, 0]],
                ))

            if n_points > 1:
                rr.log(f"{namespace}/label_end", rr.Points3D(
                    [positions[-1] + np.array([0.01, 0.01, 0.01])],
                    labels=[f"Action[{n_points-1}]"],
                    radii=[0.001],
                    colors=[[128, 0, 255]],
                ))

    # =========================================================================
    # Target Visualization
    # =========================================================================

    def log_target(
        self,
        target_ee: np.ndarray,
        safe_target: np.ndarray | None = None,
        namespace: str = "/world/target",
    ):
        """
        Log target position with optional clamping visualization.

        Args:
            target_ee: Raw target position from model
            safe_target: Clamped/safe target (if different)
            namespace: Rerun path namespace
        """
        # Safe target (yellow)
        safe = safe_target if safe_target is not None else target_ee
        rr.log(f"{namespace}/safe", rr.Points3D(
            [safe],
            colors=[[255, 255, 0]],
            radii=[0.015],
        ))

        # If raw target differs, show it and the clamping line
        if safe_target is not None:
            diff = np.linalg.norm(target_ee - safe_target)
            if diff > 0.01:
                rr.log(f"{namespace}/raw", rr.Points3D(
                    [target_ee],
                    colors=[[255, 0, 0]],
                    radii=[0.010],
                ))
                rr.log(f"{namespace}/clamp_line", rr.LineStrips3D(
                    [[target_ee, safe_target]],
                    colors=[[255, 100, 100]],
                    radii=[0.002],
                ))

    # =========================================================================
    # Image Visualization
    # =========================================================================

    def log_images(self, images: dict[str, np.ndarray]):
        """
        Log camera images.

        Args:
            images: Dict mapping camera names to images.
                    Supported keys: "top", "wrist", or custom names.
        """
        for name, img in images.items():
            if img is None:
                continue

            # Convert PIL Image to numpy if needed
            if hasattr(img, 'convert'):
                img = np.array(img)

            # Map common names to paths
            if name in ("top", "image", "main"):
                path = "/cameras/top/image"
            elif name in ("wrist", "image2", "side"):
                path = "/cameras/wrist/image"
            else:
                path = f"/cameras/{name}/image"

            rr.log(path, rr.Image(img))

    # =========================================================================
    # Main API
    # =========================================================================

    def log_frame(
        self,
        joints: np.ndarray | list | None = None,
        action_chunk: np.ndarray | None = None,
        images: dict[str, np.ndarray] | None = None,
        target_ee: np.ndarray | None = None,
        safe_target: np.ndarray | None = None,
        show_labels: bool = True,
    ):
        """
        Log a complete visualization frame.

        Args:
            joints: Joint positions in degrees [5 or 6 values]
            action_chunk: Action chunk [N, 7] or [N, 3]
            images: Dict of camera images {"name": image}
            target_ee: Raw target EE position
            safe_target: Clamped target EE position
            show_labels: Whether to show frame/action labels
        """
        # Current EE position (computed from joints)
        current_ee = None

        # Log robot arm
        if joints is not None:
            joints = np.asarray(joints, dtype=np.float64)
            self.log_robot_arm(joints, show_labels=show_labels)
            # Compute current EE for action visualization
            current_ee = self.fk.get_ee_position(joints[:5])

        # Log action chunk
        if action_chunk is not None:
            self.log_action_chunk(
                action_chunk,
                current_ee=current_ee,
                show_labels=show_labels,
            )

        # Log images
        if images is not None:
            self.log_images(images)

        # Log target
        if target_ee is not None:
            self.log_target(target_ee, safe_target)

    # =========================================================================
    # Lifecycle Management
    # =========================================================================

    def close(self):
        """Close the visualizer and kill the viewer process."""
        if self._closed:
            return

        self._closed = True

        # Kill viewer process
        if self.viewer_pid:
            try:
                os.kill(self.viewer_pid, signal.SIGTERM)
            except (OSError, ProcessLookupError):
                pass
            self.viewer_pid = None
        else:
            # Try to kill any rerun process
            try:
                subprocess.run(
                    ["pkill", "-f", "rerun"],
                    capture_output=True,
                    timeout=2,
                )
            except Exception:
                pass

        # Restore original signal handlers
        try:
            signal.signal(signal.SIGINT, self._original_sigint or signal.SIG_DFL)
            signal.signal(signal.SIGTERM, self._original_sigterm or signal.SIG_DFL)
        except Exception:
            pass

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup."""
        self.close()
        return False

    def __del__(self):
        """Destructor - fallback cleanup."""
        self.close()
