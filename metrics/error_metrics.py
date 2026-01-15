"""
Kinematic Error Metrics Module.

Provides three error metrics for evaluating robot control accuracy:
1. Task Space Error: ||X_cmd - FK(θ_obs)||
2. Joint Space Error: ||IK(X_cmd) - θ_obs||
3. IK Error: ||FK(IK(X_cmd)) - X_cmd||

These metrics are generic and work with any robot that has FK/IK functions.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol

import numpy as np


class FKFunction(Protocol):
    """Protocol for Forward Kinematics function."""

    def __call__(self, joint_angles_deg: np.ndarray) -> np.ndarray:
        """
        Compute forward kinematics.

        Args:
            joint_angles_deg: Joint angles in degrees.

        Returns:
            4x4 transformation matrix or 3D position vector.
        """
        ...


class IKFunction(Protocol):
    """Protocol for Inverse Kinematics function."""

    def __call__(
        self, current_joints_deg: np.ndarray, target_pose: np.ndarray
    ) -> np.ndarray:
        """
        Compute inverse kinematics.

        Args:
            current_joints_deg: Current joint angles in degrees (initial guess).
            target_pose: Target EE pose (4x4 matrix or 3D position).

        Returns:
            Joint angles in degrees.
        """
        ...


@dataclass
class ErrorMetrics:
    """Container for all three error metrics."""

    task_space_error: float  # meters
    joint_space_error: float  # degrees
    ik_error: float  # meters

    def to_dict(self) -> dict[str, float]:
        """Convert to dictionary."""
        return {
            "task_space_error_m": self.task_space_error,
            "joint_space_error_deg": self.joint_space_error,
            "ik_error_m": self.ik_error,
        }

    def __str__(self) -> str:
        return (
            f"task={self.task_space_error:.4f}m, "
            f"joint={self.joint_space_error:.2f}°, "
            f"ik={self.ik_error:.4f}m"
        )


class KinematicErrorMetrics:
    """
    Compute kinematic error metrics for robot control evaluation.

    This class provides three error metrics that diagnose where errors
    originate in the VLA → IK → Robot pipeline:

    1. Task Space Error: Distance between commanded EE position and
       actual EE position (computed via FK from observed joints).
       ||X_cmd - FK(θ_obs)||

    2. Joint Space Error: Distance between target joint configuration
       (from IK) and actual joint configuration.
       ||IK(X_cmd) - θ_obs||

    3. IK Error: Round-trip reconstruction error of the IK solver.
       ||FK(IK(X_cmd)) - X_cmd||

    Example:
        >>> from so101_ik_fk import SO101ForwardKinematics, RobotKinematics
        >>> from so101_ik_fk.metrics import KinematicErrorMetrics
        >>>
        >>> fk = SO101ForwardKinematics()
        >>> ik = RobotKinematics(urdf_path, ...)
        >>>
        >>> metrics = KinematicErrorMetrics(
        ...     fk_fn=fk.get_ee_position,
        ...     ik_fn=ik.inverse_kinematics,
        ... )
        >>>
        >>> errors = metrics.compute_all(X_cmd, theta_obs)
        >>> print(errors)  # task=0.0123m, joint=2.34°, ik=0.0012m
    """

    def __init__(
        self,
        fk_fn: Callable[[np.ndarray], np.ndarray],
        ik_fn: Callable[[np.ndarray, np.ndarray], np.ndarray],
    ):
        """
        Initialize the error metrics calculator.

        Args:
            fk_fn: Forward kinematics function.
                   Signature: (joint_angles_deg) -> position_3d or pose_4x4
            ik_fn: Inverse kinematics function.
                   Signature: (current_joints_deg, target_pose) -> joint_angles_deg
        """
        self._fk = fk_fn
        self._ik = ik_fn

    def _extract_position(self, pose: np.ndarray) -> np.ndarray:
        """Extract 3D position from pose (handles both 3D vector and 4x4 matrix)."""
        if pose.shape == (4, 4):
            return pose[:3, 3]
        elif pose.shape == (3,):
            return pose
        else:
            raise ValueError(f"Unexpected pose shape: {pose.shape}")

    def _build_pose_matrix(self, position: np.ndarray) -> np.ndarray:
        """Build a 4x4 pose matrix from 3D position (identity rotation)."""
        if position.shape == (4, 4):
            return position
        pose = np.eye(4)
        pose[:3, 3] = position[:3]
        return pose

    def task_space_error(
        self, X_cmd: np.ndarray, theta_obs: np.ndarray
    ) -> float:
        """
        Compute task space error: ||X_cmd - FK(θ_obs)||

        Measures how far the robot's actual EE position is from where
        the model commanded it to go.

        Args:
            X_cmd: Commanded EE position (3D) or pose (4x4).
            theta_obs: Observed joint angles in degrees.

        Returns:
            Euclidean distance in meters.
        """
        X_cmd_pos = self._extract_position(X_cmd)
        fk_result = self._fk(theta_obs)
        X_obs = self._extract_position(fk_result)
        return float(np.linalg.norm(X_cmd_pos - X_obs))

    def joint_space_error(
        self, X_cmd: np.ndarray, theta_obs: np.ndarray
    ) -> float:
        """
        Compute joint space error: ||IK(X_cmd) - θ_obs||

        Measures the difference between target joint configuration
        (derived from IK) and what the motors actually report.

        Args:
            X_cmd: Commanded EE position (3D) or pose (4x4).
            theta_obs: Observed joint angles in degrees.

        Returns:
            L2 norm of joint angle difference in degrees.
        """
        target_pose = self._build_pose_matrix(X_cmd)
        theta_cmd = self._ik(theta_obs, target_pose)
        return float(np.linalg.norm(theta_cmd - theta_obs))

    def ik_error(
        self, X_cmd: np.ndarray, theta_obs: np.ndarray
    ) -> float:
        """
        Compute IK reconstruction error: ||FK(IK(X_cmd)) - X_cmd||

        Measures the numerical accuracy of the IK solver itself.
        Even with perfect motor execution, IK may not perfectly
        achieve the target pose due to singularities, solver
        tolerances, or kinematic constraints.

        Args:
            X_cmd: Commanded EE position (3D) or pose (4x4).
            theta_obs: Observed joint angles in degrees (used as IK initial guess).

        Returns:
            Euclidean distance in meters.
        """
        X_cmd_pos = self._extract_position(X_cmd)
        target_pose = self._build_pose_matrix(X_cmd)

        # IK: X_cmd -> theta_cmd
        theta_cmd = self._ik(theta_obs, target_pose)

        # FK: theta_cmd -> X_reconstructed
        fk_result = self._fk(theta_cmd)
        X_reconstructed = self._extract_position(fk_result)

        return float(np.linalg.norm(X_reconstructed - X_cmd_pos))

    def compute_all(
        self, X_cmd: np.ndarray, theta_obs: np.ndarray
    ) -> ErrorMetrics:
        """
        Compute all three error metrics at once.

        More efficient than calling each method separately when all
        metrics are needed, as it reuses intermediate computations.

        Args:
            X_cmd: Commanded EE position (3D) or pose (4x4).
            theta_obs: Observed joint angles in degrees.

        Returns:
            ErrorMetrics dataclass with all three errors.
        """
        X_cmd_pos = self._extract_position(X_cmd)
        target_pose = self._build_pose_matrix(X_cmd)

        # FK(theta_obs) -> X_obs
        fk_obs_result = self._fk(theta_obs)
        X_obs = self._extract_position(fk_obs_result)

        # IK(X_cmd) -> theta_cmd
        theta_cmd = self._ik(theta_obs, target_pose)

        # FK(theta_cmd) -> X_reconstructed
        fk_cmd_result = self._fk(theta_cmd)
        X_reconstructed = self._extract_position(fk_cmd_result)

        return ErrorMetrics(
            task_space_error=float(np.linalg.norm(X_cmd_pos - X_obs)),
            joint_space_error=float(np.linalg.norm(theta_cmd - theta_obs)),
            ik_error=float(np.linalg.norm(X_reconstructed - X_cmd_pos)),
        )
