from .lib.kinematics import RobotKinematics
from .lib.so101_kinematics import SO101ForwardKinematics, SO101Position
from .lib.delta_ik import DeltaIK, DeltaIKResult
from .utils.data import download_and_load_data, extract_joint_values

# Metrics module
from .metrics import KinematicErrorMetrics


def __getattr__(name):
    """Lazy imports for heavy dependencies (matplotlib, rerun)."""
    if name == "create_trajectory_gif":
        from .utils.visualization import create_trajectory_gif
        return create_trajectory_gif
    elif name == "RerunVisualizer":
        try:
            from .viz import RerunVisualizer
            return RerunVisualizer
        except ImportError:
            return None
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    # Kinematics
    "RobotKinematics",
    "SO101ForwardKinematics",
    "SO101Position",
    # Delta IK (prevents branch jumping)
    "DeltaIK",
    "DeltaIKResult",
    # Matplotlib visualization (lazy)
    "create_trajectory_gif",
    # Data utilities
    "download_and_load_data",
    "extract_joint_values",
    # Rerun visualization (lazy, optional)
    "RerunVisualizer",
    # Metrics
    "KinematicErrorMetrics",
]
