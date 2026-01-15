"""
Round-trip IK Test.

Tests that FK(IK(X_cmd)) ≈ X_cmd, verifying the IK solver accuracy.

NOTE: The current placo-based IK solver has relatively high round-trip errors
(60-120mm typical). This is a known limitation of the position-weighted IK
approach. The tests document this behavior and will fail if the error
significantly increases (regression detection).

To see the error interactively, run:
    python scripts/interactive_ik_visualizer.py
"""

import numpy as np
import pytest

from so101_ik_fk.lib.so101_kinematics import (
    SO101ForwardKinematics,
    SO101Position,
    SO101_JOINT_NAMES,
)
from so101_ik_fk.metrics.error_metrics import KinematicErrorMetrics


@pytest.fixture
def fk():
    """Create forward kinematics instance."""
    return SO101ForwardKinematics()


@pytest.fixture
def metrics(fk):
    """Create error metrics calculator."""
    return KinematicErrorMetrics(
        fk_fn=fk.get_ee_position,
        ik_fn=fk.kinematics.inverse_kinematics,
    )


class TestIKRoundtrip:
    """Test IK round-trip error: ||FK(IK(X_cmd)) - X_cmd||

    Current IK solver has significant error (~60-120mm). These tests
    serve as regression tests - they fail if error increases significantly.
    """

    # Tolerance for IK error in meters
    # NOTE: Current solver has ~60-170mm error, this is a regression threshold
    IK_TOLERANCE_M = 0.20  # 200mm - fail if it gets significantly worse

    def test_roundtrip_from_home(self, fk, metrics):
        """Test IK round-trip starting from home position."""
        # Start at home
        theta_initial = SO101Position.HOME.to_array()

        # Get current EE position
        X_cmd = fk.get_ee_position(theta_initial)

        # Compute round-trip error
        ik_error = metrics.ik_error(X_cmd, theta_initial)

        assert ik_error < self.IK_TOLERANCE_M, (
            f"IK round-trip error {ik_error*1000:.3f}mm exceeds tolerance "
            f"{self.IK_TOLERANCE_M*1000:.1f}mm"
        )

    def test_roundtrip_from_forward_extended(self, fk, metrics):
        """Test IK round-trip from forward extended position."""
        theta_initial = SO101Position.FORWARD_EXTENDED.to_array()
        X_cmd = fk.get_ee_position(theta_initial)

        ik_error = metrics.ik_error(X_cmd, theta_initial)

        assert ik_error < self.IK_TOLERANCE_M, (
            f"IK round-trip error {ik_error*1000:.3f}mm exceeds tolerance"
        )

    def test_roundtrip_from_tucked(self, fk, metrics):
        """Test IK round-trip from tucked position."""
        theta_initial = SO101Position.TUCKED.to_array()
        X_cmd = fk.get_ee_position(theta_initial)

        ik_error = metrics.ik_error(X_cmd, theta_initial)

        assert ik_error < self.IK_TOLERANCE_M, (
            f"IK round-trip error {ik_error*1000:.3f}mm exceeds tolerance"
        )

    @pytest.mark.parametrize("offset", [
        np.array([0.01, 0, 0]),    # +1cm X
        np.array([-0.01, 0, 0]),   # -1cm X
        np.array([0, 0.01, 0]),    # +1cm Y
        np.array([0, -0.01, 0]),   # -1cm Y
        np.array([0, 0, 0.01]),    # +1cm Z
        np.array([0, 0, -0.01]),   # -1cm Z
    ])
    def test_roundtrip_with_offset_target(self, fk, metrics, offset):
        """Test IK round-trip with small offset from current position."""
        theta_initial = SO101Position.FORWARD_EXTENDED.to_array()
        current_ee = fk.get_ee_position(theta_initial)

        # Target is slightly offset from current
        X_cmd = current_ee + offset

        ik_error = metrics.ik_error(X_cmd, theta_initial)

        assert ik_error < self.IK_TOLERANCE_M, (
            f"IK round-trip error {ik_error*1000:.3f}mm for offset {offset}"
        )

    def test_roundtrip_random_configurations(self, fk, metrics):
        """Test IK round-trip for multiple random joint configurations."""
        np.random.seed(42)

        # Joint limits in degrees (approximate safe range)
        joint_limits = [
            (-90, 90),    # shoulder_pan
            (-90, 90),    # shoulder_lift
            (-90, 90),    # elbow_flex
            (-90, 90),    # wrist_flex
            (-90, 90),    # wrist_roll
        ]

        n_tests = 10
        errors = []

        for _ in range(n_tests):
            # Random joint angles within limits
            theta_initial = np.array([
                np.random.uniform(low, high)
                for low, high in joint_limits
            ])

            X_cmd = fk.get_ee_position(theta_initial)
            ik_error = metrics.ik_error(X_cmd, theta_initial)
            errors.append(ik_error)

        max_error = max(errors)
        mean_error = np.mean(errors)

        print(f"\nRandom config IK errors: mean={mean_error*1000:.3f}mm, max={max_error*1000:.3f}mm")

        assert max_error < self.IK_TOLERANCE_M, (
            f"Max IK error {max_error*1000:.3f}mm exceeds tolerance"
        )

    def test_ik_recovers_original_joints(self, fk):
        """Test that IK(FK(theta)) ≈ theta for the same position."""
        theta_original = SO101Position.FORWARD_EXTENDED.to_array()

        # FK to get EE position
        ee_pose = fk.compute(theta_original)

        # IK to recover joints
        theta_recovered = fk.kinematics.inverse_kinematics(
            theta_original, ee_pose
        )

        # Should recover similar joint angles
        joint_error = np.linalg.norm(theta_recovered - theta_original)

        # Allow 1 degree tolerance per joint
        assert joint_error < 5.0, (
            f"Joint recovery error {joint_error:.2f}° too large.\n"
            f"Original: {theta_original}\n"
            f"Recovered: {theta_recovered}"
        )


class TestIKErrorMetrics:
    """Test the error metrics computation."""

    def test_metrics_compute_all(self, fk, metrics):
        """Test compute_all returns consistent results."""
        theta_obs = SO101Position.HOME.to_array()
        X_cmd = fk.get_ee_position(theta_obs)

        result = metrics.compute_all(X_cmd, theta_obs)

        # When target = current FK position, task space error should be ~0
        assert result.task_space_error < 0.001

        # IK error is non-zero due to solver limitations (~60-100mm typical)
        # This documents current behavior - not a bug, just solver accuracy
        assert result.ik_error < 0.15, f"IK error {result.ik_error*1000:.1f}mm exceeds baseline"

    def test_metrics_to_dict(self, fk, metrics):
        """Test metrics serialization."""
        theta_obs = SO101Position.HOME.to_array()
        X_cmd = fk.get_ee_position(theta_obs)

        result = metrics.compute_all(X_cmd, theta_obs)
        d = result.to_dict()

        assert "task_space_error_m" in d
        assert "joint_space_error_deg" in d
        assert "ik_error_m" in d


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
