"""
Rerun-based visualization module for SO101 robot.

Provides a reusable RerunVisualizer class for live 3D visualization
of robot arm, action trajectories, and camera feeds.

Example:
    >>> from so101_ik_fk.viz import RerunVisualizer
    >>>
    >>> with RerunVisualizer(spawn=True) as viz:
    ...     viz.log_frame(
    ...         joints=[0, 45, -45, 0, 0],
    ...         action_chunk=actions,
    ...         images={"top": camera_image}
    ...     )
"""

from .rerun_visualizer import RerunVisualizer

__all__ = ["RerunVisualizer"]

