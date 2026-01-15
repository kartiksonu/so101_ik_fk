"""
Delta-based Inverse Kinematics.

Instead of directly solving IK for an absolute target (which can jump branches),
this approach:
1. Computes delta from current EE to target EE
2. Clamps delta to max step size
3. Solves IK for intermediate target (close to current)

This ensures IK always finds a solution near the current configuration,
preventing kinematic branch jumping (e.g., elbow-up to elbow-down).

Usage:
    >>> from so101_ik_fk import DeltaIK, SO101ForwardKinematics
    >>> fk = SO101ForwardKinematics()
    >>> delta_ik = DeltaIK(fk.kinematics, max_ee_delta=0.02)
    >>> target_joints, info = delta_ik.solve(current_joints, target_ee)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .kinematics import RobotKinematics


@dataclass
class DeltaIKResult:
    """Result from delta IK solver."""

    target_joints_deg: np.ndarray  # Joint angles to command
    current_ee: np.ndarray  # Current EE position [x,y,z]
    target_ee: np.ndarray  # Original target EE
    clamped_ee: np.ndarray  # Actual target after clamping
    delta: np.ndarray  # Original delta vector
    delta_clamped: np.ndarray  # Delta after clamping
    delta_norm: float  # Original delta magnitude (meters)
    was_clamped: bool  # Whether clamping occurred

    def __str__(self) -> str:
        status = "CLAMPED" if self.was_clamped else "OK"
        return (
            f"DeltaIK[{status}]: delta={self.delta_norm*100:.1f}cm, "
            f"ee=[{self.clamped_ee[0]:.3f}, {self.clamped_ee[1]:.3f}, {self.clamped_ee[2]:.3f}]"
        )


class DeltaIK:
    """
    Delta-based IK solver that prevents kinematic branch jumping.

    The Problem:
        Standard IK solvers can find wildly different joint configurations
        for the same EE pose (e.g., elbow-up vs elbow-down). When the target
        is far from the current pose, the solver might jump to a different
        kinematic branch, causing erratic robot motion.

    The Solution:
        By limiting how far the EE moves per step, we ensure:
        1. The intermediate target is always close to current EE
        2. The IK solution stays close to current joint configuration
        3. No branch jumping occurs

    Mathematical Formulation:
        1. X_current = FK(θ_current)           # Current EE from FK
        2. Δx = X_target - X_current           # Delta to target
        3. if ||Δx|| > max_delta:
               Δx_clamped = max_delta × (Δx / ||Δx||)  # Clamp magnitude
           else:
               Δx_clamped = Δx
        4. X_intermediate = X_current + Δx_clamped  # Close to current
        5. θ_target = IK(θ_current, X_intermediate) # IK with good seed

    Example:
        >>> delta_ik = DeltaIK(kinematics, max_ee_delta=0.02)  # 2cm/step
        >>> result = delta_ik.solve(current_joints, target_ee=[0.3, 0.1, 0.2])
        >>> if result.was_clamped:
        ...     print(f"Target too far, clamped from {result.delta_norm*100:.1f}cm to 2cm")
        >>> robot.send_joints(result.target_joints_deg)
    """

    def __init__(
        self,
        kinematics: RobotKinematics,
        max_ee_delta: float = 0.02,  # 2cm default
    ):
        """
        Initialize delta IK solver.

        Args:
            kinematics: RobotKinematics instance (handles FK and IK)
            max_ee_delta: Maximum EE movement per step (meters).
                          Smaller = more stable but slower convergence.
                          Recommended: 0.01-0.03m (1-3cm)
        """
        self.kinematics = kinematics
        self.max_ee_delta = max_ee_delta

    def solve(
        self,
        current_joints_deg: np.ndarray,
        target_ee: np.ndarray,
        max_delta: Optional[float] = None,
    ) -> DeltaIKResult:
        """
        Solve IK using delta approach.

        Args:
            current_joints_deg: Current joint angles in degrees (5 joints for SO101)
            target_ee: Target EE position [x, y, z] in meters
            max_delta: Override max_ee_delta for this call (optional)

        Returns:
            DeltaIKResult with target joints and diagnostic info
        """
        max_delta = max_delta if max_delta is not None else self.max_ee_delta

        # =====================================================================
        # Step 1: FK to get current EE position
        # =====================================================================
        current_pose = self.kinematics.forward_kinematics(current_joints_deg)
        current_ee = current_pose[:3, 3]

        # =====================================================================
        # Step 2: Compute delta to target
        # =====================================================================
        target_ee = np.asarray(target_ee)
        delta = target_ee - current_ee
        delta_norm = np.linalg.norm(delta)

        # =====================================================================
        # Step 3: Clamp delta if it exceeds max_delta
        # This is the KEY step that prevents branch jumping!
        # =====================================================================
        was_clamped = False
        if delta_norm > max_delta:
            # Scale delta to max_delta while preserving direction
            delta_clamped = delta * (max_delta / delta_norm)
            was_clamped = True
        else:
            delta_clamped = delta

        # =====================================================================
        # Step 4: Compute intermediate target (guaranteed close to current)
        # =====================================================================
        clamped_ee = current_ee + delta_clamped

        # =====================================================================
        # Step 5: Build target pose matrix
        # Keep current orientation, only change position
        # =====================================================================
        target_pose = np.eye(4)
        target_pose[:3, :3] = current_pose[:3, :3]  # Preserve orientation
        target_pose[:3, 3] = clamped_ee  # New position

        # =====================================================================
        # Step 6: Solve IK with current joints as seed
        # Because clamped_ee is close to current_ee, IK will find a solution
        # close to current_joints (no branch jumping!)
        # =====================================================================
        target_joints_deg = self.kinematics.inverse_kinematics(
            current_joints_deg, target_pose
        )

        return DeltaIKResult(
            target_joints_deg=target_joints_deg,
            current_ee=current_ee,
            target_ee=target_ee,
            clamped_ee=clamped_ee,
            delta=delta,
            delta_clamped=delta_clamped,
            delta_norm=delta_norm,
            was_clamped=was_clamped,
        )
